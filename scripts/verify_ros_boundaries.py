"""Exercise rejection and timeout against the live native ROS Action server."""
import json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from roboweave_interfaces.action import MoveToPose
from turtlesim.msg import Pose

rclpy.init();node=Node('roboweave_boundary_probe');poses=[]
node.create_subscription(Pose,'/turtle1/pose',poses.append,10)
client=ActionClient(node,MoveToPose,'/roboweave/move_to_pose')
assert client.wait_for_server(timeout_sec=5)
def wait(future,timeout=6):
    rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
    assert future.done(),'ROS future timed out'
    return future.result()
invalid=wait(client.send_goal_async(MoveToPose.Goal(x=999.,y=2.,timeout_sec=5.)))
assert not invalid.accepted
feedback=[]
goal=wait(client.send_goal_async(MoveToPose.Goal(x=9.,y=9.,timeout_sec=1.),feedback_callback=lambda f:feedback.append(f.feedback)))
assert goal.accepted
result=wait(goal.get_result_async())
assert result.status==6 and result.result.reason=='execution_timeout'
deadline=time.monotonic()+1
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
assert poses and abs(poses[-1].linear_velocity)<.001 and abs(poses[-1].angular_velocity)<.001
receipt={'invalid_goal_rejected':True,'timeout_ros_status':result.status,'timeout_reason':result.result.reason,'feedback_count':len(feedback),'stopped_velocity':[poses[-1].linear_velocity,poses[-1].angular_velocity]}
Path('artifacts/roboweave/boundaries.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
node.destroy_node();rclpy.shutdown()
