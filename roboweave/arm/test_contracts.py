import math
import unittest
from roboweave.arm.contracts import validate, verified


class ContractTests(unittest.TestCase):
    def test_valid_command(self):
        validate(dict(skill='grasp',object_id='cube',frame_id='world',timeout_sec=60,seed=0))

    def test_invalid_boundary(self):
        for patch in ({'skill':'shell'},{'timeout_sec':float('nan')},{'timeout_sec':float('inf')},
                      {'timeout_sec':-1},{'timeout_sec':121},{'seed':True},{'seed':-1},{'seed':10000},
                      {'frame_id':'camera'},{'object_id':'arbitrary'},{'inject_fault':'yes'},
                      {'skill':'grasp','inject_fault':True}):
            with self.subTest(patch=patch),self.assertRaises(ValueError):validate({'skill':'reset',**patch})

    def test_success_requires_all_native_evidence(self):
        state=dict(native_success=True,is_grasped=True,ever_grasped=True,is_robot_static=True,distance_to_goal=.01,lift_height=.1)
        self.assertTrue(verified(state))
        for key in ('native_success','is_grasped','ever_grasped','is_robot_static'):
            self.assertFalse(verified({**state,key:False}))
        for patch in ({'distance_to_goal':.03},{'lift_height':0},{'distance_to_goal':math.nan}):
            self.assertFalse(verified({**state,**patch}))

if __name__=='__main__':unittest.main()
