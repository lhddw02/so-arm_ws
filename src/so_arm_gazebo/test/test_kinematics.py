"""Check physical task geometry and original-model/controller limits offline."""

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from so_arm_gazebo.kinematics import (  # noqa: E402
    ARM_JOINTS, GRIPPER_JOINT, JOINT_LIMITS, JOINT_ORIGINS,
    forward_kinematics, trajectory_duration, validate_joint_positions, validate_plan,
)


@pytest.fixture
def parameters():
    with (PACKAGE / 'config' / 'demo.yaml').open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['pick_and_place']['ros__parameters']


def test_kinematics_matches_supplied_urdf():
    original = PACKAGE.parents[2] / 'Simulation' / 'SO100' / 'so100.urdf'
    # A copied standalone workspace still checks against its simulation description.
    source = original if original.is_file() else PACKAGE / 'urdf' / 'so100.urdf.xacro'
    robot = ET.parse(source).getroot()
    for name, expected_origin in zip(ARM_JOINTS, JOINT_ORIGINS):
        joint = robot.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        axis = joint.find('axis')
        actual = (tuple(map(float, origin.get('xyz').split())),
                  tuple(map(float, origin.get('rpy').split())),
                  tuple(map(float, axis.get('xyz').split())))
        assert actual == expected_origin
    for name in (*ARM_JOINTS, GRIPPER_JOINT):
        limit = robot.find(f"joint[@name='{name}']/limit")
        assert (float(limit.get('lower')), float(limit.get('upper'))) == JOINT_LIMITS[name]
        assert float(limit.get('velocity')) == 1.0


def test_all_waypoints_hit_cube_and_approach_targets(parameters):
    waypoints = {name: parameters[name]
                 for name in ('home', 'pick', 'lift', 'transfer', 'place', 'retreat')}
    validate_plan(waypoints, parameters['source_position'], parameters['destination_position'],
                  parameters['base_height'], parameters['approach_height'],
                  parameters['release_clearance'])
    for name, target in (
        ('pick', parameters['source_position']),
        ('place', [*parameters['destination_position'][:2],
                   parameters['destination_position'][2] + parameters['release_clearance']]),
    ):
        position, _ = forward_kinematics(parameters[name])
        actual = (*position[:2], position[2] + parameters['base_height'])
        assert math.dist(actual, target) < 0.0002


def test_descents_and_transfers_keep_cube_above_table(parameters):
    # Evaluate the whole interpolated path, rather than only its endpoints.
    stages = [('home', 'pick'), ('pick', 'lift'), ('lift', 'transfer'),
              ('transfer', 'place'), ('place', 'retreat'), ('retreat', 'home')]
    for start_name, end_name in stages:
        for sample in range(101):
            t = sample / 100.0
            blend = 6*t**5 - 15*t**4 + 10*t**3
            positions = [start + (end - start)*blend
                         for start, end in zip(parameters[start_name], parameters[end_name])]
            point, rotation = forward_kinematics(positions)
            # Cube half-height is 10 mm. The 6 mm distal finger extension clears the table.
            assert point[2] >= 0.010 - 1e-8
            assert point[2] - 0.006 >= 0.0039
            assert math.dist(tuple(-rotation[i][1] for i in range(3)), (0, 0, -1)) < 1e-4
            if (start_name, end_name) in [('lift', 'transfer'), ('retreat', 'home')]:
                assert point[2] >= parameters['approach_height'] - 1e-5


def test_quintic_peak_speed_respects_urdf_and_requested_limit(parameters):
    movements = [(parameters[a], parameters[b]) for a, b in
                 [('home', 'pick'), ('pick', 'lift'), ('lift', 'transfer'),
                  ('transfer', 'place'), ('place', 'retreat'), ('retreat', 'home')]]
    movements.append(([parameters['gripper_open']], [parameters['gripper_closed']]))
    for start, goal in movements:
        duration = trajectory_duration(start, goal, parameters['max_joint_velocity'],
                                       parameters['minimum_motion_duration'])
        for sample in range(101):
            t = sample / 100.0
            derivative = 30*t**2*(1-t)**2
            for begin, end in zip(start, goal):
                speed = abs(end - begin)*derivative/duration
                assert speed <= parameters['max_joint_velocity']
                assert speed <= 1.0


def test_gripper_commands_stay_in_original_limits(parameters):
    for name in ('gripper_open', 'gripper_closed'):
        validate_joint_positions([GRIPPER_JOINT], [parameters[name]])
    assert parameters['gripper_open'] > parameters['gripper_closed']


@pytest.mark.parametrize('positions', [
    [0, 0, 0, 0],
    [0, -0.01, 0, 0, 0],
    [0, 0, 0, 1.21, 0],
    [float('nan'), 0, 0, 0, 0],
])
def test_invalid_joint_commands_fail_before_motion(positions):
    with pytest.raises(ValueError):
        forward_kinematics(positions)


def test_config_edits_that_miss_scene_are_rejected(parameters):
    waypoints = {name: parameters[name]
                 for name in ('home', 'pick', 'lift', 'transfer', 'place', 'retreat')}
    moved_target = list(parameters['destination_position'])
    moved_target[0] += 0.05
    with pytest.raises(ValueError, match='misses configured target'):
        validate_plan(waypoints, parameters['source_position'], moved_target,
                      parameters['base_height'], parameters['approach_height'],
                      parameters['release_clearance'])


@pytest.mark.parametrize('velocity', [0.0, -0.1, 1.01, float('nan')])
def test_invalid_velocity_limits_are_rejected(velocity):
    with pytest.raises(ValueError):
        trajectory_duration([0.0], [0.8], velocity)
