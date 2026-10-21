"""ROS 2 action boundary. Replace this adapter for real robot deployment."""
import json
import threading
import time
import urllib.request

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from roboweave_interfaces.action import Manipulate
from .contracts import validate


def request(path, data=None):
    req=urllib.request.Request('http://127.0.0.1:8772'+path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=3) as response:
        return json.load(response)


class ArmAdapter(Node):
    def __init__(self):
        super().__init__('roboweave_arm_adapter')
        self.lock=threading.Lock(); self.busy=False; self.unknown=False; self.state={}
        self.publisher=self.create_publisher(String,'/roboweave/arm/state',10)
        self.group=ReentrantCallbackGroup()
        self.action=ActionServer(self,Manipulate,'/roboweave/arm/manipulate',self.execute,
            callback_group=self.group,goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT)
        threading.Thread(target=self.poll_state,daemon=True).start()

    def poll_state(self):
        while rclpy.ok():
            try:
                state=request('/state')
                self.state=state
                self.publisher.publish(String(data=json.dumps(state)))
            except Exception:
                pass  # Consumers independently reject stale or missing observations.
            time.sleep(0.15)

    def command(self, goal):
        return {name:getattr(goal,name) for name in ('skill','object_id','frame_id','timeout_sec','seed','inject_fault')}

    def accept(self,goal):
        try:validate(self.command(goal))
        except ValueError:return GoalResponse.REJECT
        with self.lock:
            if self.busy or self.unknown or time.time()-self.state.get('timestamp',0)>2:
                return GoalResponse.REJECT
            self.busy=True
        return GoalResponse.ACCEPT

    async def execute(self,goal):
        future=Future()
        def work():future.set_result(self.perform(goal))
        threading.Thread(target=work,daemon=True).start()
        return await future

    def perform(self,goal):
        result=Manipulate.Result(); jid=None; finished=False; started=time.monotonic()
        try:
            jid=request('/jobs',self.command(goal.request))['id']
            cancelled=False
            while True:
                elapsed=time.monotonic()-started
                if (goal.is_cancel_requested or elapsed>goal.request.timeout_sec+2) and not cancelled:
                    request('/cancel',{'id':jid}); cancelled=True
                job=request('/jobs/'+jid)
                if job['status']=='done':
                    finished=True; r=job['result']; obs=r['observation']
                    result.success=r['success'] and not cancelled
                    result.reason='cancelled' if goal.is_cancel_requested else ('execution_timeout' if cancelled else r['reason'])
                    result.observation_json=json.dumps(obs);result.trajectory_id=obs.get('trajectory_id') or ''
                    if goal.is_cancel_requested:goal.canceled()
                    elif result.success:goal.succeed()
                    else:goal.abort()
                    break
                state=self.state
                if state.get('tcp_position') and state.get('phase')==goal.request.skill and state.get('timestamp',0)>=job['started_at']:
                    goal.publish_feedback(Manipulate.Feedback(phase=state['phase'],tcp_position=state['tcp_position'],
                        object_position=state['cube_position'],distance_to_goal=float(state['distance_to_goal']),
                        is_grasped=state['is_grasped'],steps=state['steps'],elapsed_sec=elapsed))
                if elapsed>goal.request.timeout_sec+10:
                    raise RuntimeError('unknown_execution_state')
                time.sleep(0.15)
        except Exception as error:
            result.reason=type(error).__name__+':'+str(error)[:100]
            if not finished:
                if jid:
                    try:request('/cancel',{'id':jid})
                    except Exception:pass
                # A lost POST acknowledgement may hide an accepted job even
                # before a job id reaches us. Do not accept more goals blindly.
                self.unknown=True
                result.reason='unknown_execution_state'
            if goal.is_active:goal.abort()
        finally:
            with self.lock:self.busy=False
        return result


def main():
    rclpy.init();node=ArmAdapter();executor=SingleThreadedExecutor();executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
