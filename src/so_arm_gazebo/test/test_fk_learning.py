"""Independent derivative and reference checks for the educational FK API."""

import json
import math
from pathlib import Path
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from so_arm_gazebo.kinematics import (  # noqa: E402
    ARM_JOINTS, TCP_OFFSET, forward_kinematics, forward_kinematics_chain,
    position_jacobian,
)


POSES = (
    [0.0, 1.6569670531, -1.5806712649, 1.1529205386, 0.0],
    [0.6, 1.5, -1.4, 0.9, -0.7],
    [-1.2, 2.7, -2.0, 0.1, 2.3],
)


def multiply(left, right):
    """Test-side matrix product, independent of the module's private helpers."""
    return [[sum(a*b for a, b in zip(row, column)) for column in zip(*right)]
            for row in left]


def assert_matrix_close(actual, expected, tolerance=1e-13):
    assert len(actual) == len(expected)
    for actual_row, expected_row in zip(actual, expected):
        assert actual_row == pytest.approx(expected_row, abs=tolerance, rel=0.0)


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('offset', [TCP_OFFSET, (0.015, -0.070, 0.020)])
def test_analytic_jacobian_matches_central_finite_differences(pose, offset):
    analytic = position_jacobian(pose, offset)
    assert len(analytic) == 3
    assert all(len(row) == 5 for row in analytic)
    epsilon = 1e-6
    for joint in range(5):
        positive, negative = list(pose), list(pose)
        positive[joint] += epsilon
        negative[joint] -= epsilon
        p_positive = forward_kinematics(positive, offset)[0]
        p_negative = forward_kinematics(negative, offset)[0]
        numerical = [(plus - minus)/(2*epsilon)
                     for plus, minus in zip(p_positive, p_negative)]
        actual = [analytic[row][joint] for row in range(3)]
        assert actual == pytest.approx(numerical, abs=2e-9, rel=0.0)


@pytest.mark.parametrize('pose,point,rotation', [
    (
        [0, 0, 0, 0, 0],
        (0.0, -0.24085227230255146, 0.10277657721848876),
        ((6.326794896668469e-06, 0.0, 0.9999999999799858),
         (0.3349762276273899, 0.9422265793958646, -2.1193258875006424e-06),
         (-0.9422265793770067, 0.3349762276340942, 5.961274314027145e-06)),
    ),
    (
        [0.6, 1.5, -1.4, 0.9, -0.7],
        (0.10929457022183892, -0.20495489291304492, 0.10144365096870972),
        ((0.1111344969218479, -0.12829143165646412, 0.9854899452340762),
         (0.97849289256937, 0.18752922701847435, -0.08593281215624743),
         (-0.17378372416643384, 0.9738450069581855, 0.14637322035624026)),
    ),
    (
        [-1.2, 2.7, -2.0, 0.1, 2.3],
        (-0.30327913356734426, -0.16310905735428474, -0.03509093281649604),
        ((-0.8348773829937892, 0.38787047911916145, 0.39055889030121166),
         (0.4755023426235175, 0.15080187076123064, 0.866692747133873),
         (0.27726751978006475, 0.9092938398574936, -0.31033439266737145)),
    ),
])
def test_existing_fk_api_keeps_reference_results(pose, point, rotation):
    # Reference values were recorded from the original FK before its refactor.
    actual_point, actual_rotation = forward_kinematics(pose)
    assert isinstance(actual_point, tuple)
    assert isinstance(actual_rotation, tuple)
    assert all(isinstance(row, tuple) for row in actual_rotation)
    assert actual_point == pytest.approx(point, abs=1e-14, rel=0.0)
    assert_matrix_close(actual_rotation, rotation, tolerance=1e-14)
    chain = forward_kinematics_chain(pose)
    assert chain['position'] == list(actual_point)
    assert chain['rotation'] == [list(row) for row in actual_rotation]


def test_chain_is_json_serializable_and_exposes_column_vector_products():
    pose = POSES[1]
    chain = forward_kinematics_chain(pose)
    assert json.loads(json.dumps(chain, allow_nan=False)) == chain
    assert [step['name'] for step in chain['steps']] == list(ARM_JOINTS)
    assert [frame['name'] for frame in chain['joint_frames']] == list(ARM_JOINTS)
    previous = [[float(i == j) for j in range(4)] for i in range(4)]
    for step, frame, angle in zip(chain['steps'], chain['joint_frames'], pose):
        assert step['angle'] == angle
        for key in ('origin', 'motion', 'local', 'cumulative'):
            assert len(step[key]) == 4
            assert all(len(row) == 4 for row in step[key])
            assert step[key][3] == [0.0, 0.0, 0.0, 1.0]
        assert_matrix_close(step['local'], multiply(step['origin'], step['motion']))
        assert_matrix_close(step['cumulative'], multiply(previous, step['local']))
        assert frame['joint_origin_in_base'] == [step['cumulative'][i][3] for i in range(3)]
        assert sum(value*value for value in frame['axis_in_base']) == pytest.approx(1.0)
        previous = step['cumulative']
    translation = [[float(i == j) for j in range(4)] for i in range(4)]
    for axis, value in enumerate(TCP_OFFSET):
        translation[axis][3] = value
    assert_matrix_close(chain['base_tool'], multiply(previous, translation))
    assert chain['position'] == [chain['base_tool'][i][3] for i in range(3)]


def test_zero_pose_joint_frames_follow_original_urdf_axes_and_origins():
    frames = forward_kinematics_chain([0.0]*5)['joint_frames']
    sine, cosine = math.sin(1.57079), math.cos(1.57079)
    assert frames[0]['joint_origin_in_base'] == [0.0, -0.0452, 0.0165]
    assert frames[0]['axis_in_base'] == pytest.approx([0.0, cosine, sine], abs=1e-15)
    assert frames[1]['joint_origin_in_base'] == pytest.approx([
        0.0, -0.0452 + 0.1025*cosine - 0.0306*sine,
        0.0165 + 0.1025*sine + 0.0306*cosine,
    ], abs=1e-15)
    assert frames[1]['axis_in_base'] == [1.0, 0.0, 0.0]


def test_default_tcp_roll_changes_orientation_without_changing_position():
    pose, rolled = list(POSES[1]), list(POSES[1])
    rolled[4] += 0.8
    position, rotation = forward_kinematics(pose)
    new_position, new_rotation = forward_kinematics(rolled)
    assert new_position == pytest.approx(position, abs=1e-14, rel=0.0)
    assert math.dist(rotation[0], new_rotation[0]) > 0.1
    jacobian = position_jacobian(pose)
    assert [row[4] for row in jacobian] == pytest.approx([0.0]*3, abs=1e-14)
    off_axis_jacobian = position_jacobian(pose, (0.015, -0.070, 0.020))
    assert math.sqrt(sum(row[4]**2 for row in off_axis_jacobian)) == pytest.approx(0.025)


@pytest.mark.parametrize('offset', [(0.0, 0.0), (0.0, float('inf'), 0.0),
                                    (0.0, float('nan'), 0.0)])
def test_nonphysical_tcp_offsets_are_rejected(offset):
    with pytest.raises(ValueError, match='three finite coordinates'):
        forward_kinematics_chain(POSES[0], offset)
    with pytest.raises(ValueError):
        position_jacobian(POSES[0], offset)
