"""Pure validation and postconditions shared by arm adapters and tests."""
import math

SKILLS = ('reset', 'approach', 'grasp', 'recover', 'transport', 'verify')


def validate(command):
    if command.get('skill') not in SKILLS:
        raise ValueError('unknown_skill')
    if command.get('object_id', 'cube') != 'cube' or command.get('frame_id', 'world') != 'world':
        raise ValueError('unsupported_object_or_frame')
    timeout = command.get('timeout_sec', 60)
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 0.1 <= timeout <= 120:
        raise ValueError('invalid_timeout')
    seed = command.get('seed', 0)
    if type(seed) is not int or not 0 <= seed <= 9999:
        raise ValueError('invalid_seed')
    if type(command.get('inject_fault', False)) is not bool:
        raise ValueError('invalid_fault_flag')
    if command.get('inject_fault') and command['skill'] != 'reset':
        raise ValueError('fault_can_only_be_armed_on_reset')


def verified(state):
    return bool(state and state.get('native_success') and state.get('is_grasped')
                and state.get('ever_grasped') and state.get('is_robot_static')
                and 0 <= state.get('distance_to_goal', math.inf) <= 0.025
                and state.get('lift_height', 0) > 0.02)
