#!/usr/bin/env python3
# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""An imaging sonar from a fan of rays: what a forward-looking multibeam would show.

Gazebo has no acoustic sensor. The submarine carries a gpu_lidar that covers the sonar's field of
view (tools/marine_sensors.py, SONAR) more finely than its beams; this node turns each ray that
hits something into an echo, as a multibeam sonar forms its image:

* the strength of an echo is that of the surface's backscatter, Lambert's law: cos^2 of the angle
  between the ray and the surface's normal (worked out from the neighbouring rays), weighted by the
  vertical beam pattern (a Gaussian, -3 dB at the edges of the vertical aperture) and the two-way
  absorption of the water (dB/m). The spreading loss is taken as compensated by the sonar's
  time-varied gain, as real sonars do;
* the sonar cannot tell the elevation: every ray of a beam that comes back at the same range adds
  to the same cell (range bin, beam). Behind what the rays hit there is nothing: acoustic shadows;
* the pulse spreads the echo over a few range bins and the beam's width over neighbouring beams
  (Gaussians), speckle multiplies each cell (exponential with mean 1, that of the intensity of a
  Rayleigh echo) and a noise floor is added;
* the intensity is log-compressed to 8 bits over `--dynamic-range` dB.

Out: /sonar/image (marine_acoustic_msgs/ProjectedSonarImage, in sonar_link: z forward, x up, y to
starboard; data[range * beams + beam], beam 0 at port), /sonar/image_fan (sensor_msgs/Image mono8,
the fan as seen from above, what RViz can show) and /sonar/points (sensor_msgs/PointCloud2 in
sonar_rays_link: the strong echoes, at zero elevation, which is all a sonar knows of them).
In: /sonar/rays (sensor_msgs/PointCloud2 of the gpu_lidar, in sonar_rays_link: x forward, y left).
"""

import argparse
import math
import os
import sys

import numpy as np

SOUND_SPEED = 1500.0      # m/s


def gaussian_kernel(sigma):
    """A normalised 1D Gaussian of standard deviation `sigma` samples (at least 1 tap)."""
    if sigma <= 0.0:
        return np.ones(1)
    half = max(1, int(math.ceil(3.0 * sigma)))
    x = np.arange(-half, half + 1, dtype=float)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    return k / k.sum()


def blur(image, sigma, axis):
    """`image` convolved with a Gaussian of `sigma` samples along `axis` (same size)."""
    k = gaussian_kernel(sigma)
    if k.size == 1:
        return image
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode='same'), axis, image)


class Sonar:
    """The geometry and the echo model of a forward-looking multibeam."""

    def __init__(self, beams=256, horizontal_fov=math.radians(130.0),
                 vertical_fov=math.radians(20.0), min_range=0.5, max_range=50.0,
                 range_resolution=0.1, frequency=750e3, absorption_db_per_m=0.2,
                 horizontal_beamwidth=math.radians(1.0), pulse_bins=1.0, noise_floor=2e-4,
                 dynamic_range_db=40.0, speckle=True, seed=None):
        if beams < 2 or not (0.0 < min_range < max_range) or range_resolution <= 0.0:
            raise ValueError('a sonar needs at least two beams and min_range < max_range')
        self.beams = beams
        self.half_h = 0.5 * horizontal_fov
        self.half_v = 0.5 * vertical_fov
        self.min_range, self.max_range = min_range, max_range
        self.resolution = range_resolution
        self.bins = int(math.ceil((max_range - min_range) / range_resolution))
        self.ranges = min_range + (np.arange(self.bins) + 0.5) * range_resolution
        # beam 0 at port (azimuth +half_h, to the left), the last one at starboard
        self.azimuths = np.linspace(self.half_h, -self.half_h, beams)
        self.frequency = frequency
        self.absorption = absorption_db_per_m
        self.horizontal_beamwidth = horizontal_beamwidth
        self.vertical_sigma = self.half_v / math.sqrt(2.0 * math.log(2.0))   # -3 dB at the edge
        self.pulse_bins = pulse_bins
        self.noise_floor = noise_floor
        self.dynamic_range = dynamic_range_db
        self.speckle = speckle
        self.rng = np.random.default_rng(seed)

    def beam_of(self, azimuth):
        """Index of the beam of each azimuth (rad, positive to port)."""
        step = 2.0 * self.half_h / (self.beams - 1)
        return np.rint((self.half_h - azimuth) / step).astype(int)

    def beam_directions(self):
        """Unit vector of each beam in sonar_link (z forward, x up, y to starboard)."""
        a = self.azimuths
        return np.stack([np.zeros_like(a), -np.sin(a), np.cos(a)], axis=1)

    def backscatter(self, grid):
        """Lambert's cos^2 of each ray of an organised grid (V, H, 3) of hits (NaN where none):
        the normal from the neighbouring hits; 1 where it cannot be told (a lone hit)."""
        dv = np.gradient(grid, axis=0)
        dh = np.gradient(grid, axis=1)
        normal = np.cross(dv, dh)
        norm = np.linalg.norm(normal, axis=2)
        ray = grid / np.linalg.norm(grid, axis=2, keepdims=True)
        cos = np.abs(np.einsum('vhk,vhk->vh', normal, ray)) / np.where(norm > 0, norm, np.nan)
        return np.where(np.isfinite(cos), np.clip(cos, 0.0, 1.0) ** 2, 1.0)

    def grid(self, points, rows, cols):
        """The hits (N, 3) put on an organised grid (rows x cols) by their elevation and azimuth,
        as the rays were cast; the cells of no hit are NaN."""
        grid = np.full((rows, cols, 3), np.nan)
        if len(points) == 0:
            return grid
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        az = np.arctan2(y, x)
        el = np.arctan2(z, np.hypot(x, y))
        i = np.rint((el + self.half_v) / (2.0 * self.half_v) * (rows - 1)).astype(int)
        j = np.rint((az + self.half_h) / (2.0 * self.half_h) * (cols - 1)).astype(int)
        ok = (i >= 0) & (i < rows) & (j >= 0) & (j < cols)
        grid[i[ok], j[ok]] = points[ok]
        return grid

    def intensity(self, points, rows, cols):
        """Linear intensity (bins x beams) of the echoes of the hits (N, 3) in the frame of the
        rays (x forward, y left, z up), cast as a grid of rows x cols."""
        image = np.zeros((self.bins, self.beams))
        points = np.asarray(points, float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        grid = self.grid(points, rows, cols)
        hit = np.all(np.isfinite(grid), axis=2)
        if hit.any():
            weight = self.backscatter(grid)[hit]
            p = grid[hit]
            r = np.linalg.norm(p, axis=1)
            az = np.arctan2(p[:, 1], p[:, 0])
            el = np.arctan2(p[:, 2], np.hypot(p[:, 0], p[:, 1]))
            weight = weight * np.exp(-0.5 * (el / self.vertical_sigma) ** 2)
            weight = weight * 10.0 ** (-2.0 * self.absorption * r / 10.0)
            b = np.floor((r - self.min_range) / self.resolution).astype(int)
            beam = self.beam_of(az)
            ok = (b >= 0) & (b < self.bins) & (beam >= 0) & (beam < self.beams)
            np.add.at(image, (b[ok], beam[ok]), weight[ok])
        image = blur(image, self.pulse_bins, axis=0)
        step = 2.0 * self.half_h / (self.beams - 1)
        image = blur(image, self.horizontal_beamwidth / 2.355 / step, axis=1)
        if self.speckle:
            image = image * self.rng.exponential(1.0, image.shape)
        if self.noise_floor > 0.0:
            image = image + self.noise_floor * self.rng.exponential(1.0, image.shape)
        return image

    def to_uint8(self, image):
        """Log compression: 0 dB (one full echo) is 255, -dynamic_range dB and below is 0."""
        db = 10.0 * np.log10(np.maximum(image, 1e-12))
        scaled = (db + self.dynamic_range) / self.dynamic_range
        return (np.clip(scaled, 0.0, 1.0) * 255.0).astype(np.uint8)

    def fan_map(self, width):
        """For an image of the fan seen from above (`width` px across, the sonar at the bottom
        centre, forward up), the (bin, beam) of each pixel, -1 out of the fan."""
        scale = width / (2.0 * self.max_range * math.sin(self.half_h))
        height = int(math.ceil(self.max_range * scale)) + 1
        v, u = np.mgrid[0:height, 0:width]
        forward = (height - 1 - v) / scale
        left = ((width - 1) / 2.0 - u) / scale
        r = np.hypot(forward, left)
        az = np.arctan2(left, forward)
        b = np.floor((r - self.min_range) / self.resolution).astype(int)
        beam = self.beam_of(az)
        inside = ((b >= 0) & (b < self.bins) & (np.abs(az) <= self.half_h)
                  & (beam >= 0) & (beam < self.beams))
        return np.where(inside, b, -1), np.where(inside, beam, -1)

    def detections(self, image8, threshold):
        """(x, y, 0) in the frame of the rays of each cell at or over `threshold` (0-255)."""
        b, beam = np.nonzero(image8 >= threshold)
        r, az = self.ranges[b], self.azimuths[beam]
        return np.stack([r * np.cos(az), r * np.sin(az), np.zeros_like(r)], axis=1)


def fan_image(image8, fan):
    rows, cols = fan
    out = np.zeros(rows.shape, np.uint8)
    inside = rows >= 0
    out[inside] = image8[rows[inside], cols[inside]]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--dynamic-range', type=float, default=40.0, help='dB shown')
    parser.add_argument('--range-resolution', type=float, default=0.1, help='m per range bin')
    parser.add_argument('--fan-width', type=int, default=640, help='px of /sonar/image_fan')
    parser.add_argument('--threshold', type=int, default=170,
                        help='0-255: cells this strong are /sonar/points')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--sim-time', action='store_true')
    args, ros_args = parser.parse_known_args(argv)

    import importlib.util
    import rclpy
    from geometry_msgs.msg import Vector3
    from marine_acoustic_msgs.msg import ProjectedSonarImage, SonarImageData
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from sensor_msgs_py import point_cloud2

    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        'marine_sensors', os.path.join(here, 'marine_sensors.py'))
    table = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(table)
    s = table.SONAR
    sonar = Sonar(s['beams'], s['horizontal_fov'], s['vertical_fov'], s['min_range'],
                  s['max_range'], args.range_resolution, s['frequency'],
                  dynamic_range_db=args.dynamic_range, seed=args.seed)
    fan = sonar.fan_map(args.fan_width)
    directions = [Vector3(x=float(d[0]), y=float(d[1]), z=float(d[2]))
                  for d in sonar.beam_directions()]

    rclpy.init(args=ros_args)
    node = rclpy.create_node(
        'sonar', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    image_pub = node.create_publisher(ProjectedSonarImage, '/sonar/image', 10)
    fan_pub = node.create_publisher(Image, '/sonar/image_fan', 10)
    points_pub = node.create_publisher(PointCloud2, '/sonar/points', 10)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate('xyz')]

    def on_rays(msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        image8 = sonar.to_uint8(sonar.intensity(xyz, s['rays_vertical'], s['rays_horizontal']))

        out = ProjectedSonarImage()
        out.header.stamp, out.header.frame_id = msg.header.stamp, 'sonar_link'
        out.ping_info.frequency = float(sonar.frequency)
        out.ping_info.sound_speed = SOUND_SPEED
        out.ping_info.tx_beamwidths = [float(s['vertical_fov'])] * sonar.beams
        out.ping_info.rx_beamwidths = [float(sonar.horizontal_beamwidth)] * sonar.beams
        out.beam_directions = directions
        out.ranges = sonar.ranges.astype(float).tolist()
        out.image = SonarImageData(is_bigendian=False, dtype=SonarImageData.DTYPE_UINT8,
                                   beam_count=sonar.beams, data=image8.tobytes())
        image_pub.publish(out)

        picture = fan_image(image8, fan)
        img = Image()
        img.header = out.header
        img.height, img.width = picture.shape
        img.encoding, img.is_bigendian, img.step = 'mono8', 0, picture.shape[1]
        img.data = picture.tobytes()
        fan_pub.publish(img)

        cloud = point_cloud2.create_cloud(
            msg.header, fields, sonar.detections(image8, args.threshold).astype(np.float32))
        cloud.header.frame_id = 'sonar_rays_link'
        points_pub.publish(cloud)

    node.create_subscription(PointCloud2, '/sonar/rays', on_rays, qos_profile_sensor_data)
    node.get_logger().info('imaging sonar: %d beams over %.0f deg, %.1f-%.0f m in %d bins' % (
        sonar.beams, math.degrees(2 * sonar.half_h), sonar.min_range, sonar.max_range,
        sonar.bins))
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                # Ctrl+C, or a second one while shutting down: nothing left to close
    return 0


if __name__ == '__main__':
    sys.exit(main())
