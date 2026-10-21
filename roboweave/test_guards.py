import threading
from unittest.mock import Mock

from roboweave.runtime import Runtime


def runtime():
    r=Runtime.__new__(Runtime)
    r.skill_lock=threading.Lock();r.failed_skill=None
    r.task={'steps':[],'expected_goals':[{'x':8.,'y':7.}]}
    r.emit=Mock();r._move=Mock(return_value={'success':True})
    return r


def test_out_of_plan_never_executes():
    r=runtime()
    assert r.move(3,4)['reason']=='goal_not_in_ordered_plan'
    r._move.assert_not_called()


def test_unknown_execution_state_halts_further_skills():
    r=runtime();r._move.return_value={'success':False,'reason':'unknown_execution_state'}
    assert not r.move(8,7)['success']
    assert r.move(8,7)['reason']=='execution_halted'
    assert r._move.call_count==1


def test_concurrent_skill_is_rejected():
    r=runtime();r.skill_lock.acquire()
    assert r.move(8,7)['reason']=='concurrent_skill_rejected'
    r._move.assert_not_called();r.skill_lock.release()
