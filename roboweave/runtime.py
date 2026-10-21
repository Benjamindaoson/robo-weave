"""RAI task runtime: actual ROS 2 transport, bounded recovery, sensor verifier."""
import copy
import io
import json
import math
import re
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from langchain_core.messages import HumanMessage
from langchain_core.tools import tool
from rai import get_llm_model
from rai.agents.langchain.core import create_react_runnable
from rai.communication.ros2 import ROS2Connector, ROS2Message

ROOT=Path('artifacts/roboweave')
ROOT.mkdir(parents=True,exist_ok=True)


class Runtime:
    def __init__(self):
        self.lock=threading.RLock()
        self.skill_lock=threading.Lock()
        self.failed_skill=None
        self.connector=ROS2Connector(node_name='roboweave_rai_runtime',executor_type='single_threaded')
        self.pose=None;self.pose_time=0.0;self.trail=[];self.events=[];self.task=None
        self.busy=False;self.cancel=threading.Event();self.action_handle=None;self.run_dir=None
        self.connector.register_callback('/turtle1/pose',self.on_pose,msg_type='turtlesim/msg/Pose')

    def on_pose(self,message):
        p={k:float(getattr(message.payload,k)) for k in ('x','y','theta','linear_velocity','angular_velocity')}
        with self.lock:
            self.pose={k:float(p[k]) for k in ('x','y','theta','linear_velocity','angular_velocity')}
            self.pose_time=time.monotonic()
            if self.busy and (not self.trail or math.hypot(p['x']-self.trail[-1]['x'],p['y']-self.trail[-1]['y'])>0.03):
                self.trail.append({'x':p['x'],'y':p['y']})
                self.trail=self.trail[-500:]

    def emit(self,actor,kind,**data):
        with self.lock:
            event=dict(timestamp=time.time(),actor=actor,kind=kind,**data)
            self.events.append(event)
            if self.run_dir:
                with (self.run_dir/'events.jsonl').open('a') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')

    def observe(self):
        with self.lock:
            return {'pose':copy.deepcopy(self.pose),'age_sec':round(time.monotonic()-self.pose_time,3),'source':'/turtle1/pose','frame':'turtlesim_world'}

    def status(self):
        with self.lock:
            return dict(environment='ROS 2 Jazzy / turtlesim',runtime='RAI ReAct + ROS2Connector',busy=self.busy,task=copy.deepcopy(self.task),observation=self.observe(),trail=list(self.trail),events=list(self.events[-180:]),history=self.history(),scope='二维 ROS 2 运动闭环；非机械臂抓取，非真机部署')

    def history(self):
        records=[]
        for p in sorted(ROOT.glob('runs/*/result.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:12]:
            try:records.append(json.loads(p.read_text()))
            except (OSError,json.JSONDecodeError):pass
        return records

    def start(self,text,fault=False):
        if not isinstance(text,str) or not text.strip() or len(text)>1200:raise ValueError('任务应为 1–1200 字符')
        with self.lock:
            if self.busy:raise RuntimeError('已有任务运行中')
            if not self.pose or time.monotonic()-self.pose_time>1:raise RuntimeError('ROS 位姿不可用或已过期')
            pairs=re.findall(r'\(\s*(-?\d+(?:\.\d+)?)\s*[,，]\s*(-?\d+(?:\.\d+)?)\s*\)',text.replace('（','(').replace('）',')'))
            expected=[{'x':float(x),'y':float(y)} for x,y in pairs] or [{'x':8.0,'y':7.0},{'x':3.0,'y':8.0}]
            if len(expected)>5 or any(not (1<=g['x']<=10 and 1<=g['y']<=10) for g in expected):
                raise ValueError('最多 5 个目标，坐标必须在 [1,10]')
            self.busy=True;self.cancel.clear();self.events=[];self.trail=[];self.failed_skill=None
            self.run_dir=ROOT/'runs'/uuid.uuid4().hex;self.run_dir.mkdir(parents=True)
            self.task=dict(id=self.run_dir.name,text=text,status='running',started_at=time.time(),recoveries=0,verified_goals=[],fault_injection=fault,steps=[],expected_goals=expected)
        threading.Thread(target=self.run,args=(text,fault),daemon=True).start()
        return self.task['id']

    def stop(self):
        self.cancel.set()
        self.emit('User','cancel_requested')

    def move(self,x,y,timeout_sec=20):
        if not self.skill_lock.acquire(blocking=False):
            self.failed_skill='concurrent_skill_rejected'
            return {'success':False,'reason':self.failed_skill}
        try:
            if self.failed_skill:return {'success':False,'reason':'execution_halted','cause':self.failed_skill}
            index=len(self.task['steps'])
            if index>=len(self.task['expected_goals']) or {'x':x,'y':y}!=self.task['expected_goals'][index]:
                result={'success':False,'reason':'goal_not_in_ordered_plan'}
                self.emit('Executor','parameters_rejected',**result)
            else:
                result=self._move(x,y,timeout_sec)
            if not result.get('success'):self.failed_skill=result.get('reason','skill_failed')
            return result
        finally:self.skill_lock.release()

    def _move(self,x,y,timeout_sec=20):
        if self.cancel.is_set():return {'success':False,'reason':'cancelled'}
        if not all(math.isfinite(v) for v in (x,y,timeout_sec)) or not (1<=x<=10 and 1<=y<=10 and 1<=timeout_sec<=30):
            self.emit('Executor','parameters_rejected',x=x,y=y)
            return {'success':False,'reason':'invalid_goal'}
        if len(self.task['steps'])>=5:return {'success':False,'reason':'skill_budget_exhausted'}
        self.task['steps'].append({'x':x,'y':y,'status':'running'})
        for attempt in range(2):
            if self.cancel.is_set():return {'success':False,'reason':'cancelled'}
            observation=self.observe()
            if observation['age_sec']>1:return {'success':False,'reason':'stale_observation'}
            done=threading.Event();box={};feedback_count=[0]
            def feedback(msg):
                f=msg.feedback;feedback_count[0]+=1
                if feedback_count[0]%4==1:
                    self.emit('ROS2','action_feedback',x=f.x,y=f.y,theta=f.theta,distance=f.distance_remaining,elapsed=f.elapsed_sec,attempt=attempt)
            def finished(future):
                try:
                    response=future.result();r=response.result
                    box.update(ros_status=response.status,success=r.success,reason=r.reason,distance=r.distance,final_x=r.final_x,final_y=r.final_y)
                except Exception as error:box.update(success=False,reason=type(error).__name__)
                finally:done.set()
            self.emit('Executor','action_goal',action='/roboweave/move_to_pose',x=x,y=y,attempt=attempt)
            try:
                self.action_handle=self.connector.start_action(ROS2Message(payload={'x':float(x),'y':float(y),'timeout_sec':float(timeout_sec)}),'/roboweave/move_to_pose',on_feedback=feedback,on_done=finished,msg_type='roboweave_interfaces/action/MoveToPose',timeout_sec=3.0)
            except Exception as error:
                self.emit('Executor','action_error',reason=type(error).__name__)
                return {'success':False,'reason':type(error).__name__}
            deadline=time.monotonic()+timeout_sec+5;cancel_sent=False
            while not done.wait(0.1):
                if self.cancel.is_set() or time.monotonic()>deadline:
                    if not cancel_sent:
                        cancel_sent=True
                        try:self.connector.terminate_action(self.action_handle)
                        except Exception:pass
                        self.emit('Executor','action_cancel_sent')
                    if time.monotonic()>deadline+5:
                        box.update(success=False,reason='unknown_execution_state');break
            self.action_handle=None
            self.emit('Executor','action_result',feedback_samples=feedback_count[0],**box)
            if self.cancel.is_set():box.update(success=False,reason='cancelled')
            if box.get('success') and box.get('ros_status')==4:
                verdict=self.verify(x,y)
                self.task['steps'][-1]['status']='succeeded' if verdict['success'] else 'failed'
                return {**box,'verification':verdict,'success':verdict['success']}
            if box.get('ros_status')==6 and box.get('reason')=='injected_transient_failure' and attempt==0 and not self.cancel.is_set():
                self.task['recoveries']+=1
                self.emit('Recovery','reobserve_and_retry',retry=1,max_retries=1,observation=self.observe(),reason=box['reason'])
                continue
            self.task['steps'][-1]['status']='failed'
            return box
        return {'success':False,'reason':'retry_limit'}

    def verify(self,x,y):
        obs=self.observe();p=obs['pose']
        distance=math.hypot(p['x']-x,p['y']-y) if p else float('inf')
        success=bool(p and obs['age_sec']<1 and distance<=0.12)
        result=dict(success=success,distance=distance,x=x,y=y,source='/turtle1/pose',age_sec=obs['age_sec'])
        self.emit('Verifier','sensor_postcondition',**result)
        if success and self.task:
            goal={'x':x,'y':y}
            if goal not in self.task['verified_goals']:self.task['verified_goals'].append(goal)
        return result

    def run(self,text,fault):
        self.emit('User','task_received',text=text)
        try:
            if fault:
                receipt=self.connector.service_call(ROS2Message(payload={}),'/roboweave/inject_failure',msg_type='std_srvs/srv/Trigger',timeout_sec=3)
                self.emit('Recovery','fault_armed',receipt=str(receipt))
                if not receipt.payload.success:raise RuntimeError('fault_not_armed')
            @tool
            def observe_robot()->dict:
                """Read fresh robot pose and observation age from ROS 2; world range is x,y=1..10."""
                result=self.observe();self.emit('Planner','observation',**result);return result
            @tool
            def navigate_to_pose(x:float,y:float)->dict:
                """Move the simulated planar robot to x,y in its world frame via a ROS 2 action. Returns actual feedback/result and sensor verification. One bounded recovery is performed only for a known injected failure. Never call after cancellation or an unknown execution state."""
                return self.move(x,y)
            @tool
            def verify_goal(x:float,y:float)->dict:
                """Verify position from current ROS 2 pose. Does not move robot. Never invent success."""
                return self.verify(x,y)
            self.emit('Planner','planning_started',framework='RAI create_react_runnable',model='deepseek-chat')
            agent=create_react_runnable(llm=get_llm_model('complex_model',streaming=False,timeout=40,max_retries=1),tools=[observe_robot,navigate_to_pose,verify_goal],system_prompt='You control a ROS2 turtlesim planar robot, not an arm. Observe first, then execute requested ordered coordinate goals using navigate_to_pose sequentially. Use coordinates in [1,10]. If no coordinates are specified use (8,7) then (3,8). Do not claim grasping. Stop on any unsuccessful navigation; do not retry failures yourself. Do not invoke multiple navigation tools in one turn. Final answer must be based on actual sensor verification. Cancellation or unknown execution state means stop. At most 5 navigation calls.')
            result=agent.invoke({'messages':[HumanMessage(content=text)]},config={'recursion_limit':20})
            messages=[]
            for m in result['messages']:
                record={'type':m.type,'content':m.content}
                if getattr(m,'tool_calls',None):record['tool_calls']=m.tool_calls
                messages.append(record)
            (self.run_dir/'agent_messages.json').write_text(json.dumps(messages,ensure_ascii=False,indent=2))
            self.emit('Planner','agent_finished',answer=str(result['messages'][-1].content))
            steps=self.task['steps']
            executed=[{'x':s['x'],'y':s['y']} for s in steps if s['status']=='succeeded']
            complete=bool(steps and not self.failed_skill and all(s['status']=='succeeded' for s in steps) and executed==self.task['expected_goals'])
            self.task['status']='stopped' if self.cancel.is_set() else ('succeeded' if complete else 'failed')
            if self.task['status']=='failed':self.task['reason']=self.failed_skill or 'no_successful_execution_or_failed_skill'
        except Exception as error:
            self.task['status']='stopped' if self.cancel.is_set() else 'failed'
            self.task['reason']=type(error).__name__
            self.emit('Runtime','error',reason=type(error).__name__,detail=str(error)[:300])
        finally:
            self.task['duration_sec']=round(time.time()-self.task['started_at'],2)
            self.task['finished_at']=time.time();self.task['final_observation']=self.observe()
            self.emit('Runtime','task_finished',status=self.task['status'])
            with self.lock:
                (self.run_dir/'result.json').write_text(json.dumps(self.task,ensure_ascii=False,indent=2))
                (self.run_dir/'trajectory.json').write_text(json.dumps(self.trail))
                self.busy=False


def main():
    rclpy.init();runtime=Runtime()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def reply(self,data,status=200,kind='application/json'):
            body=json.dumps(data,ensure_ascii=False).encode() if kind=='application/json' else data
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':self.reply(Path('roboweave/index.html').read_bytes(),kind='text/html; charset=utf-8')
            elif self.path=='/api/roboweave/status':self.reply(runtime.status())
            elif self.path.split('?')[0]=='/frame.png':
                from PIL import ImageGrab
                image=ImageGrab.grab(xdisplay=':91');buffer=io.BytesIO();image.save(buffer,format='PNG');self.reply(buffer.getvalue(),kind='image/png')
            elif self.path=='/favicon.ico':self.reply(b'',204,'image/x-icon')
            else:self.reply({'error':'not_found'},404)
        def do_POST(self):
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':self.reply({'error':'json_required'},415);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=8192:raise ValueError('invalid_body')
                data=json.loads(self.rfile.read(length))
                if self.path=='/api/roboweave/start':self.reply({'task_id':runtime.start(data.get('task',''),bool(data.get('fault',False)))},202)
                elif self.path=='/api/roboweave/stop':runtime.stop();self.reply({'ok':True})
                else:self.reply({'error':'not_found'},404)
            except ValueError as e:self.reply({'error':str(e)},400)
            except RuntimeError as e:self.reply({'error':str(e)},409)
    ThreadingHTTPServer(('127.0.0.1',8768),Handler).serve_forever()

if __name__=='__main__':main()
