"""ROS 2 skill adapter driving the real turtlesim process through Twist/Pose.

This is a 2-D simulator adapter, not Nav2 or a physical robot controller.
The Agent knows only the action contract; hardware migration replaces this node.
"""
import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose
from roboweave_interfaces.action import MoveToPose


def control(x, y, theta, gx, gy):
    distance = math.hypot(gx-x, gy-y)
    angle = math.atan2(math.sin(math.atan2(gy-y, gx-x)-theta), math.cos(math.atan2(gy-y, gx-x)-theta))
    return min(1.4, distance*1.8)*max(0.0, math.cos(angle)), max(-2.5,min(2.5,angle*4)), distance


class MotionNode(Node):
    def __init__(self):
        super().__init__('roboweave_motion_adapter')
        self.group = ReentrantCallbackGroup()
        self.pose = None
        self.pose_time = 0.0
        self.lock = threading.Lock()
        self.busy = False
        self.fail_next = False
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10, callback_group=self.group)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_service(Trigger, '/roboweave/inject_failure', self.inject, callback_group=self.group)
        self.action = ActionServer(self, MoveToPose, '/roboweave/move_to_pose', self.execute, callback_group=self.group, goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.log = Path('artifacts/roboweave/motion.jsonl')
        self.log.parent.mkdir(parents=True,exist_ok=True)

    def event(self, kind, **payload):
        with self.log.open('a') as f:
            f.write(json.dumps(dict(time=time.time(),kind=kind,**payload))+'\n')

    def on_pose(self, pose):
        self.pose = pose
        self.pose_time = time.monotonic()

    def inject(self, _, response):
        if os.environ.get('ROBOWEAVE_ENABLE_FAULTS') != '1':
            response.success=False; response.message='fault injection disabled'; return response
        with self.lock:
            response.success = not self.busy
            if response.success: self.fail_next = True
        response.message = 'next accepted action aborts after feedback' if response.success else 'busy'
        self.event('fault_armed',success=response.success)
        return response

    def goal(self, request):
        with self.lock:
            valid = all(math.isfinite(v) for v in (request.x,request.y,request.timeout_sec)) and 1 <= request.x <= 10 and 1 <= request.y <= 10 and 1 <= request.timeout_sec <= 30
            if self.busy or not valid: return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, goal):
        result_future=Future()
        def worker():
            result_future.set_result(self.execute_motion(goal))
        threading.Thread(target=worker,daemon=True).start()
        return await result_future

    def execute_motion(self, goal):
        request=goal.request; start=time.monotonic(); result=MoveToPose.Result()
        with self.lock:
            fault=self.fail_next; self.fail_next=False
        self.event('accepted',x=request.x,y=request.y,fault_injection=fault)
        try:
            while rclpy.ok():
                pose=self.pose; elapsed=time.monotonic()-start
                if goal.is_cancel_requested:
                    result.reason='cancelled'; goal.canceled(); break
                if pose is None or time.monotonic()-self.pose_time > 1.0:
                    result.reason='stale_observation'; goal.abort(); break
                linear,angular,distance=control(pose.x,pose.y,pose.theta,request.x,request.y)
                feedback=MoveToPose.Feedback(x=float(pose.x),y=float(pose.y),theta=float(pose.theta),distance_remaining=distance,elapsed_sec=elapsed)
                goal.publish_feedback(feedback)
                if fault and elapsed >= 0.7:
                    result.reason='injected_transient_failure'; goal.abort(); break
                if distance <= 0.08 and not fault:
                    result.success=True; result.reason='target_reached'; goal.succeed(); break
                if elapsed >= request.timeout_sec:
                    result.reason='execution_timeout'; goal.abort(); break
                msg=Twist(); msg.linear.x=linear; msg.angular.z=angular
                self.publisher.publish(msg); time.sleep(0.05)
        except Exception as error:
            result.reason=type(error).__name__
            if goal.is_active: goal.abort()
        finally:
            self.publisher.publish(Twist())
            pose=self.pose
            if pose:
                result.final_x=float(pose.x);result.final_y=float(pose.y)
                result.distance=math.hypot(pose.x-request.x,pose.y-request.y)
            self.event('result',success=result.success,reason=result.reason,x=result.final_x,y=result.final_y,distance=result.distance)
            with self.lock: self.busy=False
        return result


def main():
    rclpy.init(); node=MotionNode(); executor=SingleThreadedExecutor();executor.add_node(node)
    try: executor.spin()
    finally: node.publisher.publish(Twist());executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__ == '__main__': main()
