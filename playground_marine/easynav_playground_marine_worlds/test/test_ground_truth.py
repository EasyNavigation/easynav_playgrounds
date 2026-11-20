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

"""The true pose of a vehicle whose model is turned from its base_link (tools/ground_truth.py)."""

import importlib.util
import math
import os

import pytest

nav_msgs = pytest.importorskip('nav_msgs.msg')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def tool():
    spec = importlib.util.spec_from_file_location(
        'ground_truth_under_test', os.path.join(ROOT, 'tools', 'ground_truth.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def odometry(yaw=0.0, pitch=0.0, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), xyz=(1.0, 2.0, -3.0)):
    msg = nav_msgs.Odometry()
    p = msg.pose.pose
    p.position.x, p.position.y, p.position.z = xyz
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    p.orientation.x, p.orientation.y = -sy * sp, cy * sp
    p.orientation.z, p.orientation.w = sy * cp, cy * cp
    t = msg.twist.twist
    t.linear.x, t.linear.y, t.linear.z = linear
    t.angular.x, t.angular.y, t.angular.z = angular
    return msg


def yaw_pitch(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw, math.asin(2 * (q.w * q.y - q.z * q.x))


def angle(a, b):
    return abs(math.remainder(a - b, 2 * math.pi))


def test_a_model_turned_half_a_turn_moving_stern_ahead_is_a_base_link_going_forward():
    # the Tethys going east bow first: its model looks west and moves along its -x
    msg = tool().turn(odometry(yaw=math.pi, linear=(-1.5, 0.0, 0.2), angular=(0.0, 0.0, 0.05)), math.pi)
    yaw, _ = yaw_pitch(msg.pose.pose.orientation)
    assert angle(yaw, 0.0) < 1e-9
    t = msg.twist.twist
    assert (t.linear.x, t.linear.y, t.linear.z) == pytest.approx((1.5, 0.0, 0.2))
    assert t.angular.z == pytest.approx(0.05)                     # yaw rate: the same axis


def test_the_pitch_and_its_rate_change_sign_and_the_position_stays():
    # bow down is the model's stern down: its pitch is the opposite
    msg = tool().turn(odometry(yaw=math.pi, pitch=-0.2, angular=(0.0, -0.1, 0.0)), math.pi)
    _, pitch = yaw_pitch(msg.pose.pose.orientation)
    assert pitch == pytest.approx(0.2)
    assert msg.twist.twist.angular.y == pytest.approx(0.1)
    p = msg.pose.pose.position
    assert (p.x, p.y, p.z) == (1.0, 2.0, -3.0)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.0, math.pi])
def test_the_orientation_stays_a_unit_quaternion_turned_by_the_yaw(yaw):
    msg = tool().turn(odometry(yaw=0.4), yaw)
    q = msg.pose.pose.orientation
    assert q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w == pytest.approx(1.0)
    assert angle(yaw_pitch(q)[0], 0.4 - yaw) < 1e-9


def test_with_no_turn_nothing_changes():
    before = odometry(yaw=0.3, pitch=0.1, linear=(1.0, 0.5, 0.1), angular=(0.01, 0.02, 0.03))
    after = tool().turn(odometry(yaw=0.3, pitch=0.1, linear=(1.0, 0.5, 0.1),
                                 angular=(0.01, 0.02, 0.03)), 0.0)
    assert after.pose.pose.orientation.z == pytest.approx(before.pose.pose.orientation.z)
    assert after.twist.twist.linear.y == pytest.approx(0.5)
    assert after.twist.twist.angular.x == pytest.approx(0.01)
