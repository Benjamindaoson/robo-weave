"""RAI ReAct planner with native ROS 2 manipulation and bounded recovery."""
import copy
import json
import threading
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from langchain_core.messages import HumanMessage
from langchain_core.tools import tool
from rai import get_llm_model
from rai.agents.langchain.core import create_react_runnable
from rai.communication.ros2 import ROS2Connector, ROS2Message
from .contracts import verified

ROOT=Path('artifacts/arm')
PLAN=['approach','grasp','transport','verify']


class Runtime:
    def __init__(self):
        self.lock=threading.RLock();self.skill_lock=threading.Lock()
        self.connector=ROS2Connector(node_name='roboweave_arm_agent',executor_type='single_threaded')
        self.connector.register_callback('/roboweave/arm/state',self.on_state,msg_type='std_msgs/msg/String')
        self.state={};self.events=[];self.task=None;self.busy=False
        self.cancel=threading.Event();self.failed=None;self.run_dir=None

    def on_state(self,msg):
        with self.lock:self.state=json.loads(msg.payload.data)

    def observe(self):
        with self.lock:
            return dict(copy.deepcopy(self.state),age_sec=round(time.time()-self.state.get('timestamp',0),3),source='/roboweave/arm/state')

    def emit(self,actor,kind,**data):
        with self.lock:
            event=dict(timestamp=time.time(),actor=actor,kind=kind,**data);self.events.append(event)
            if self.run_dir:
                with (self.run_dir/'events.jsonl').open('a') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')

    def status(self):
        with self.lock:
            task=copy.deepcopy(self.task)
            if task and self.busy:task['elapsed_sec']=max(0,time.time()-task['started_at'])
            history=[]
            for p in sorted((ROOT/'runs').glob('*/result.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:12]:
                try:history.append(json.loads(p.read_text()))
                except (OSError,json.JSONDecodeError):pass
            return dict(environment='ManiSkill · Panda · PickCube-v1',runtime='RAI ReAct / ROS 2 Jazzy',
                observation=self.observe(),task=task,events=copy.deepcopy(self.events[-160:]),busy=self.busy,history=history,
                scope='真实 SAPIEN 物理仿真；特权物体状态 + 官方运动规划器；尚未验证真机')

    def start(self,text,fault=False,seed=0):
        if not isinstance(text,str) or not 1<=len(text.strip())<=1200:raise ValueError('invalid_task')
        if type(fault) is not bool or type(seed) is not int or not 0<=seed<=9999:raise ValueError('invalid_parameters')
        with self.lock:
            if self.busy:raise RuntimeError('task_already_running')
            if self.observe()['age_sec']>2:raise RuntimeError('arm_state_unavailable')
            self.run_dir=ROOT/'runs'/uuid.uuid4().hex;self.run_dir.mkdir(parents=True)
            self.task=dict(id=self.run_dir.name,text=text,seed=seed,fault_injection=fault,status='running',started_at=time.time(),
                           recoveries=0,steps=[],plan=list(PLAN),trajectory_id=None)
            self.events=[];self.cancel.clear();self.failed=None;self.busy=True
        threading.Thread(target=self.run,args=(text,fault,seed),daemon=True).start()
        return self.task['id']

    def stop(self):
        self.cancel.set();self.emit('User','cancel_requested')

    def action(self,skill,seed=0,fault=False,timeout=60):
        if self.cancel.is_set():return dict(success=False,reason='cancelled')
        if self.observe()['age_sec']>2:return dict(success=False,reason='stale_observation')
        done=threading.Event();box={};samples=[0]
        def feedback(msg):
            f=msg.feedback;samples[0]+=1
            if samples[0]%3==1:self.emit('ROS2','action_feedback',skill=skill,phase=f.phase,tcp=list(f.tcp_position),
                cube=list(f.object_position),distance=f.distance_to_goal,is_grasped=f.is_grasped,steps=f.steps)
        def finish(future):
            try:
                response=future.result();r=response.result
                box.update(success=r.success and response.status==4,reason=r.reason,ros_status=response.status,
                    observation=json.loads(r.observation_json) if r.observation_json else {},trajectory_id=r.trajectory_id)
            except Exception as e:box.update(success=False,reason=type(e).__name__)
            finally:done.set()
        self.emit('Executor','action_goal',skill=skill,action='/roboweave/arm/manipulate')
        try:
            handle=self.connector.start_action(ROS2Message(payload=dict(skill=skill,object_id='cube',frame_id='world',
                timeout_sec=float(timeout),seed=seed,inject_fault=fault)), '/roboweave/arm/manipulate',
                on_feedback=feedback,on_done=finish,msg_type='roboweave_interfaces/action/Manipulate',timeout_sec=3)
        except Exception as e:return dict(success=False,reason=type(e).__name__)
        deadline=time.monotonic()+timeout+12;cancelled=False
        while not done.wait(.1):
            if (self.cancel.is_set() or time.monotonic()>deadline) and not cancelled:
                cancelled=True
                try:self.connector.terminate_action(handle)
                except Exception:pass
            if time.monotonic()>deadline+5:
                box.update(success=False,reason='unknown_execution_state');break
        if self.cancel.is_set():box.update(success=False,reason='cancelled')
        box['feedback_samples']=samples[0]
        if box.get('trajectory_id'):self.task['trajectory_id']=box['trajectory_id']
        self.emit('Executor','action_result',skill=skill,**box)
        return box

    def skill(self,name):
        if not self.skill_lock.acquire(False):
            self.failed='concurrent_skill_rejected';return dict(success=False,reason=self.failed)
        try:
            if self.failed or self.cancel.is_set():return dict(success=False,reason=self.failed or 'cancelled')
            completed=len(self.task['steps'])
            if completed>=len(PLAN) or name!=PLAN[completed]:
                self.failed='skill_not_in_ordered_plan';return dict(success=False,reason=self.failed)
            result=self.action(name)
            if name=='grasp' and result.get('reason')=='grasp_contact_missing' and result.get('ros_status')==6 and not self.cancel.is_set():
                self.task['recoveries']+=1
                self.emit('Recovery','reobserve_reapproach_retry',max_retries=1,observation=self.observe())
                recovery=self.action('recover')
                result=self.action('grasp') if recovery['success'] else recovery
                result.update(recovered=result['success'],recovery_count=1,initial_failure='grasp_contact_missing')
                self.emit('Verifier','recovery_verified',success=result['success'],observation=result.get('observation'))
            record=dict(skill=name,status='succeeded' if result['success'] else 'failed',**result)
            self.task['steps'].append(record)
            if not result['success']:self.failed=result['reason']
            return result
        finally:self.skill_lock.release()

    def run(self,text,fault,seed):
        self.emit('User','task_received',text=text)
        try:
            reset=self.action('reset',seed=seed,fault=fault)
            if not reset['success']:raise RuntimeError(reset['reason'])
            @tool
            def observe_arm()->dict:
                """Read real ROS2 arm state, cube, goal, finger contact and observation freshness."""
                obs=self.observe();self.emit('Planner','observation',observation=obs);return obs
            @tool
            def execute_skill(skill:str)->dict:
                """Execute exactly one ROS2 arm skill in sequence: approach, grasp, transport, verify.
                Grasp includes one bounded recovery if contact verification fails. Never retry yourself.
                Stop after any unsuccessful result. Only the red cube and green goal are supported.
                """
                return self.skill(skill)
            self.emit('Planner','planning_started',framework='RAI create_react_runnable',model='deepseek-chat',plan=PLAN)
            agent=create_react_runnable(llm=get_llm_model('complex_model',streaming=False,timeout=40,max_retries=1),
                tools=[observe_arm,execute_skill],system_prompt='You control a Panda robot in real ManiSkill physics via ROS2. This demo supports only picking the red cube and moving it to the green goal while still holding it. There is no release/place skill. Reject unrelated tasks. Observe first. Call execute_skill with approach, grasp, transport, verify in that exact order, one call per turn. Read each result before next call. Grasp recovery is built in and limited to one; report recovered, recovery_count and initial_failure from the tool result faithfully. Do not manually retry. Stop on any failure/cancellation. Never invent success. State privileged simulation state and motion planning, not vision or real hardware. Give a concise Chinese final answer in at most 4 sentences, no tables.')
            output=agent.invoke({'messages':[HumanMessage(content=text)]},config={'recursion_limit':20})
            messages=[]
            for m in output['messages']:
                row=dict(type=m.type,content=m.content)
                if getattr(m,'tool_calls',None):row['tool_calls']=m.tool_calls
                messages.append(row)
            (self.run_dir/'agent_messages.json').write_text(json.dumps(messages,ensure_ascii=False,indent=2))
            self.emit('Planner','agent_finished',answer=str(output['messages'][-1].content))
            obs=self.observe()
            success=not self.failed and len(self.task['steps'])==4 and all(s['success'] for s in self.task['steps']) and obs['age_sec']<2 and verified(obs)
            self.task['status']='stopped' if self.cancel.is_set() else ('succeeded' if success else 'failed')
            if not success:self.task['reason']=self.failed or 'native_verification_or_plan_incomplete'
            self.emit('Verifier','final_postcondition',success=success,observation=obs)
        except Exception as e:
            self.task['status']='stopped' if self.cancel.is_set() else 'failed';self.task['reason']=str(e)[:160]
            self.emit('Runtime','error',reason=type(e).__name__)
        finally:
            self.task['duration_sec']=round(time.time()-self.task['started_at'],2)
            self.task['final_observation']=self.observe()
            self.emit('Runtime','task_finished',status=self.task['status'])
            with self.lock:
                (self.run_dir/'result.json').write_text(json.dumps(self.task,ensure_ascii=False,indent=2));self.busy=False


def main():
    rclpy.init();runtime=Runtime()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,data,status=200,kind='application/json'):
            body=json.dumps(data,ensure_ascii=False).encode() if kind=='application/json' else data
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':self.reply(Path('roboweave/arm/index.html').read_bytes(),kind='text/html; charset=utf-8')
            elif self.path=='/api/roboweave/status':self.reply(runtime.status())
            elif self.path.split('?')[0]=='/frame.png':
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8772/frame.png',timeout=3) as r:self.reply(r.read(),kind='image/png')
                except Exception:self.reply({'error':'camera_unavailable'},503)
            elif self.path=='/favicon.ico':self.reply(b'',204,'image/x-icon')
            else:self.reply({'error':'not_found'},404)
        def do_POST(self):
            try:
                if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('json_required')
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=8192:raise ValueError('invalid_body')
                data=json.loads(self.rfile.read(n))
                if not isinstance(data,dict):raise ValueError('invalid_body')
                if self.path=='/api/roboweave/start':self.reply({'task_id':runtime.start(data.get('task',''),data.get('fault',False),data.get('seed',0))},202)
                elif self.path=='/api/roboweave/stop':runtime.stop();self.reply({'ok':True})
                else:self.reply({'error':'not_found'},404)
            except ValueError as e:self.reply({'error':str(e)},400)
            except RuntimeError as e:self.reply({'error':str(e)},409)
    ThreadingHTTPServer(('127.0.0.1',8770),Handler).serve_forever()

if __name__=='__main__':main()
