"""Live ROS checks: rejected goals, deadline abort, cancel and no further steps."""
import json
import time
import urllib.request
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from roboweave_interfaces.action import Manipulate


def main():
    with urllib.request.urlopen('http://127.0.0.1:8770/api/roboweave/status') as r:
        assert not json.load(r)['busy'],'another task is active'
    rclpy.init();node=Node('arm_boundary_probe');client=ActionClient(node,Manipulate,'/roboweave/arm/manipulate')
    assert client.wait_for_server(timeout_sec=5)
    def wait(future,timeout=10):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        assert future.done(),'ROS timeout'
        return future.result()
    def send(skill,timeout=30,**extra):
        return wait(client.send_goal_async(Manipulate.Goal(skill=skill,object_id='cube',frame_id='world',timeout_sec=float(timeout),seed=0,**extra)))
    def action(skill,timeout=30):
        goal=send(skill,timeout);assert goal.accepted
        return wait(goal.get_result_async(),timeout+5)
    rejected=send('arbitrary_shell');assert not rejected.accepted
    assert action('reset').status==4
    expired=action('approach',.1)
    assert expired.status==6 and expired.result.reason=='execution_timeout'
    assert action('reset').status==4
    feedback=[]
    goal=wait(client.send_goal_async(Manipulate.Goal(skill='approach',object_id='cube',frame_id='world',timeout_sec=30.,seed=0),feedback_callback=lambda f:feedback.append(f.feedback)))
    assert goal.accepted
    deadline=time.monotonic()+4
    while not feedback and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    assert feedback
    wait(goal.cancel_goal_async())
    cancelled=wait(goal.get_result_async());assert cancelled.status==5
    def state():
        with urllib.request.urlopen('http://127.0.0.1:8772/state') as r:return json.load(r)
    a=state();time.sleep(.5);b=state()
    assert a['steps']==b['steps'] and a['joint_positions']==b['joint_positions'] and b['active_job'] is None
    receipt=dict(invalid_goal_rejected=True,timeout_ros_status=expired.status,timeout_reason=expired.result.reason,
        cancellation_ros_status=cancelled.status,cancellation_reason=cancelled.result.reason,feedback_samples=len(feedback),
        paused_steps=[a['steps'],b['steps']],joint_positions_unchanged=True,
        note='Simulator pauses physics after cancel; this is not a validated physical robot emergency stop.')
    Path('artifacts/arm/boundaries.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
