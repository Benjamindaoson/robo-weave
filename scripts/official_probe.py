"""Send a task to the unmodified official react_ros2.py and record ROS evidence."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rai_interfaces.msg import HRIMessage
from turtlesim.msg import Pose
from action_msgs.msg import GoalStatusArray

rclpy.init();node=Node('roboweave_official_probe')
record={'request':'Use ROS2 tools to inspect /turtle1/pose, then start /turtle1/rotate_absolute with type turtlesim/action/RotateAbsolute and theta=1.2. Read the action feedback and final result, then read /turtle1/pose again and report whether theta is near 1.2. Do not call any other motion or reset tool.','responses':[],'poses':[],'action_statuses':[]}
node.create_subscription(HRIMessage,'/to_human',lambda m:record['responses'].append({'text':m.text,'seq_end':m.seq_end,'time':time.time()}),10)
def pose(m):
 if not record['poses'] or abs(m.theta-record['poses'][-1]['theta'])>.01:record['poses'].append({'x':m.x,'y':m.y,'theta':m.theta,'time':time.time()})
node.create_subscription(Pose,'/turtle1/pose',pose,10)
node.create_subscription(GoalStatusArray,'/turtle1/rotate_absolute/_action/status',lambda m:record['action_statuses'].append([s.status for s in m.status_list]),10)
pub=node.create_publisher(HRIMessage,'/from_human',10)
deadline=time.monotonic()+15
while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
assert pub.get_subscription_count()>0,'official_agent_not_subscribed'
msg=HRIMessage();msg.text=record['request'];msg.communication_id='official-baseline-001';msg.seq_end=True;pub.publish(msg)
deadline=time.monotonic()+120;settled=None
while time.monotonic()<deadline:
 rclpy.spin_once(node,timeout_sec=.1)
 native_done=record['poses'] and abs(record['poses'][-1]['theta']-1.2)<.03
 if native_done and record['responses']:
  if settled is None:settled=time.monotonic()
  if time.monotonic()-settled>15:break
record['passed']=bool(record['poses'] and abs(record['poses'][-1]['theta']-1.2)<.03 and record['responses'])
Path('artifacts/roboweave/official-example.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
print(json.dumps({'passed':record['passed'],'pose_samples':len(record['poses']),'responses':len(record['responses']),'last_pose':record['poses'][-1] if record['poses'] else None,'response_tail':record['responses'][-2:]}))
node.destroy_node();rclpy.shutdown()
assert record['passed'],'official_example_failed'
