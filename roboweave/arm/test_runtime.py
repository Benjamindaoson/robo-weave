"""Guard tests complement, but do not replace, live ROS / physics checks."""
import threading
from unittest.mock import Mock
from roboweave.arm.runtime import Runtime


def runtime():
    r=Runtime.__new__(Runtime);r.skill_lock=threading.Lock();r.cancel=threading.Event();r.failed=None
    r.task={'steps':[],'recoveries':0};r.action=Mock(return_value={'success':True});r.emit=Mock();r.observe=Mock(return_value={})
    return r


def test_order_guard():
    r=runtime();assert not r.skill('transport')['success'];r.action.assert_not_called()


def test_recovery_is_bounded():
    r=runtime();r.task['steps']=[{'skill':'approach'}]
    r.action.side_effect=[{'success':False,'reason':'grasp_contact_missing','ros_status':6},
                         {'success':True}, {'success':False,'reason':'grasp_contact_missing','ros_status':6}]
    assert not r.skill('grasp')['success'];assert r.task['recoveries']==1;assert r.action.call_count==3
    assert not r.skill('grasp')['success'];assert r.action.call_count==3


def test_unknown_state_does_not_retry():
    r=runtime();r.action.return_value={'success':False,'reason':'unknown_execution_state'}
    assert not r.skill('approach')['success'];assert not r.skill('grasp')['success'];assert r.action.call_count==1


def test_cancel_prevents_motion():
    r=runtime();r.cancel.set();assert not r.skill('approach')['success'];r.action.assert_not_called()


def test_concurrent_call_halts_sequence():
    r=runtime();r.skill_lock.acquire();assert not r.skill('approach')['success'];r.skill_lock.release();r.action.assert_not_called()


def test_lost_submission_acknowledgement_latches_adapter(monkeypatch):
    from roboweave.arm.ros_adapter import ArmAdapter
    adapter=ArmAdapter.__new__(ArmAdapter)
    adapter.lock=threading.Lock();adapter.busy=True;adapter.unknown=False
    goal=Mock();goal.request.skill='approach';goal.request.timeout_sec=30
    def lost_ack(*args,**kwargs):raise TimeoutError('lost acknowledgement')
    monkeypatch.setattr('roboweave.arm.ros_adapter.request',lost_ack)
    result=adapter.perform(goal)
    assert not result.success and result.reason=='unknown_execution_state'
    assert adapter.unknown and not adapter.busy
    goal.abort.assert_called_once()
