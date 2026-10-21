"""Single-threaded real SAPIEN physics; loopback-only job API, no LLM secrets.

Motion generation reuses ManiSkill's official Panda solver and grasp geometry.
Every commanded joint action and native state is persisted in trajectories.
HTTP threads only access snapshots or queues, never the simulator itself.
"""
import io
import json
import os
import queue
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import gymnasium as gym
import mani_skill.envs  # noqa: F401 registers environments
import numpy as np
import sapien
from PIL import Image
from mani_skill.examples.motionplanning.panda.motionplanner import PandaArmMotionPlanningSolver
from mani_skill.examples.motionplanning.base_motionplanner.utils import compute_grasp_info_by_obb, get_actor_obb

from .contracts import validate, verified

ROOT = Path(os.environ.get('ROBOWEAVE_ARM_ARTIFACTS', 'artifacts/arm'))


class ArmSimulation:
    def __init__(self):
        self.lock = threading.RLock()
        self.queue = queue.Queue(maxsize=1)
        self.jobs = {}
        self.active = None
        self.state = {}
        self.frame = b''
        self.cancel = threading.Event()
        self.steps = 0
        self.ever_grasped = False
        self.phase = 'booting'
        self.planner = None
        self.run_dir = None
        self.fault = False
        self.env = None

    def snapshot(self):
        with self.lock:
            return dict(self.state, age_sec=round(time.time()-self.state.get('timestamp', 0), 3), active_job=self.active)

    def submit(self, command):
        validate(command)
        with self.lock:
            if self.active is not None:
                raise RuntimeError('simulation_busy')
            jid = uuid.uuid4().hex
            self.active = jid
            self.cancel.clear()
            self.jobs[jid] = dict(id=jid, status='queued', command=command, started_at=time.time())
            # Bound long-lived public-service memory.
            while len(self.jobs) > 100:
                del self.jobs[next(iter(self.jobs))]
            self.queue.put_nowait(jid)
            return jid

    def refresh(self, render=False):
        b = self.env.unwrapped
        ev = {k:bool(v.item()) for k,v in b.evaluate().items()}
        cube = b.cube.pose.p[0].cpu().numpy()
        target = b.goal_site.pose.p[0].cpu().numpy()
        self.ever_grasped |= ev['is_grasped']
        state = dict(timestamp=time.time(), environment='ManiSkill PickCube-v1', robot='Panda', frame_id='world',
                     phase=self.phase, steps=self.steps, cube_position=cube.tolist(), goal_position=target.tolist(),
                     tcp_position=b.agent.tcp.pose.p[0].tolist(), joint_positions=b.agent.robot.get_qpos()[0].tolist(),
                     distance_to_goal=float(np.linalg.norm(cube-target)), is_grasped=ev['is_grasped'],
                     ever_grasped=self.ever_grasped, is_robot_static=ev['is_robot_static'],
                     native_success=ev['success'], lift_height=float(cube[2]-self.initial_z),
                     trajectory_id=self.run_dir.name if self.run_dir else None)
        frame = None
        if render:
            pixels = self.env.render()[0].cpu().numpy()
            buf = io.BytesIO(); Image.fromarray(pixels).save(buf, format='PNG'); frame = buf.getvalue()
        with self.lock:
            self.state = state
            if frame is not None:
                self.frame = frame
        if frame is not None and self.run_dir and self.steps % 6 == 0:
            (self.run_dir/'frames'/f'{self.steps:05d}.png').write_bytes(frame)
        return state

    def step(self, action):
        if self.cancel.is_set():
            raise RuntimeError('cancelled')
        if time.monotonic() > self.deadline:
            raise RuntimeError('execution_timeout')
        if self.steps >= 1000:
            raise RuntimeError('step_budget_exhausted')
        result = self.real_step(action)
        self.steps += 1
        state = self.refresh(render=self.steps % 3 == 0)
        with (self.run_dir/'trajectory.jsonl').open('a') as f:
            f.write(json.dumps(dict(state=state, action=np.asarray(action).tolist()))+'\n')
        # Slow playback enough to inspect; control still uses simulator timestep.
        time.sleep(0.025)
        return result

    def reset(self, seed, fault):
        if self.planner:
            self.planner.close()
        self.env.reset(seed=seed)
        self.steps = 0; self.ever_grasped = False; self.fault = fault
        self.initial_z = float(self.env.unwrapped.cube.pose.p[0,2])
        self.run_dir = ROOT/'trajectories'/uuid.uuid4().hex
        (self.run_dir/'frames').mkdir(parents=True)
        self.planner = PandaArmMotionPlanningSolver(self.env, vis=False, base_pose=self.env.unwrapped.agent.robot.pose,
            visualize_target_grasp_pose=False, print_env_info=False)
        # PhysX contact buffers can retain the previous episode until a physics step.
        # Settle the open gripper before publishing this episode's contact evidence.
        qpos=self.env.unwrapped.agent.robot.get_qpos()[0,:7].cpu().numpy()
        for _ in range(2):self.step(np.hstack([qpos,1.0]))
        (self.run_dir/'manifest.json').write_text(json.dumps(dict(seed=seed, inject_fault=fault, environment='PickCube-v1',
            planner='ManiSkill 3.0.1 PandaArmMotionPlanningSolver', control_mode='pd_joint_pos', sim_backend='cpu')))

    def grasp_pose(self):
        b = self.env.unwrapped
        approaching = np.array([0,0,-1])
        closing = b.agent.tcp.pose.to_transformation_matrix()[0,:3,1].cpu().numpy()
        info = compute_grasp_info_by_obb(get_actor_obb(b.cube), approaching=approaching, target_closing=closing, depth=0.025)
        return b.agent.build_grasp_pose(approaching, info['closing'], b.cube.pose.sp.p)

    def move(self, pose):
        result = self.planner.move_to_pose_with_screw(pose)
        if isinstance(result, int) and result == -1:
            raise RuntimeError('motion_plan_failed')

    def execute(self, command):
        self.phase = command['skill']; self.deadline = time.monotonic()+command.get('timeout_sec', 60)
        if self.phase == 'reset':
            self.reset(command.get('seed',0), command.get('inject_fault',False))
        elif self.phase == 'approach':
            self.planner.open_gripper()
            pose = self.grasp_pose()
            self.move(pose * sapien.Pose([0,0,-0.05]))
            self.move(pose)
            if np.linalg.norm(self.env.unwrapped.agent.tcp.pose.sp.p-pose.p)>0.015:
                raise RuntimeError('approach_postcondition_failed')
        elif self.phase == 'grasp':
            if self.fault:
                # A physical actuation fault: execute OPEN instead of CLOSE.
                self.fault = False
                self.planner.open_gripper(t=12)
            else:
                self.planner.close_gripper(t=12)
            if not self.env.unwrapped.agent.is_grasping(self.env.unwrapped.cube).item():
                raise RuntimeError('grasp_contact_missing')
        elif self.phase == 'recover':
            self.planner.open_gripper()
            pose = self.grasp_pose()
            self.move(pose * sapien.Pose([0,0,-0.06]))
            self.move(self.grasp_pose())
        elif self.phase == 'transport':
            if not self.env.unwrapped.agent.is_grasping(self.env.unwrapped.cube).item():
                raise RuntimeError('grasp_required')
            b = self.env.unwrapped
            self.move(sapien.Pose(b.goal_site.pose.sp.p, b.agent.tcp.pose.sp.q))
            self.planner.close_gripper(t=12)
        state = self.refresh(render=True)
        if self.phase in ('transport','verify') and not verified(state):
            raise RuntimeError('native_postcondition_failed')
        return state

    def run(self):
        ROOT.mkdir(parents=True, exist_ok=True)
        self.env = gym.make('PickCube-v1', obs_mode='state', control_mode='pd_joint_pos', render_mode='rgb_array', sim_backend='cpu', max_episode_steps=1000)
        self.real_step = self.env.step
        self.env.step = self.step
        self.deadline=time.monotonic()+60
        self.reset(0,False); self.phase='ready'; self.refresh(render=True)
        while True:
            try:
                jid = self.queue.get(timeout=0.25)
            except queue.Empty:
                # Physics is paused between skills; republish fresh measured state.
                self.refresh(); continue
            job = self.jobs[jid]; job['status']='running'
            try:
                state = self.execute(job['command'])
                result = dict(success=True, reason='postcondition_passed', observation=state)
            except Exception as error:
                self.refresh(render=True)
                result = dict(success=False, reason=str(error)[:160], observation=self.snapshot())
            with self.lock:
                job.update(status='done', result=result, finished_at=time.time())
                self.active = None
            (self.run_dir/f'job-{jid}.json').write_text(json.dumps(job,indent=2))


def main():
    sim = ArmSimulation()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def reply(self,data,status=200,kind='application/json'):
            body=json.dumps(data).encode() if kind=='application/json' else data
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            path=self.path.split('?')[0]
            if path=='/state': self.reply(sim.snapshot())
            elif path=='/frame.png': self.reply(sim.frame,kind='image/png')
            elif path.startswith('/jobs/'):
                with sim.lock: job=sim.jobs.get(path.split('/')[-1])
                self.reply(job or {'error':'not_found'},200 if job else 404)
            else:self.reply({'error':'not_found'},404)
        def do_POST(self):
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4096: raise ValueError('invalid_body')
                body=json.loads(self.rfile.read(length))
                if self.path=='/jobs':self.reply({'id':sim.submit(body)},202)
                elif self.path=='/cancel':
                    with sim.lock:
                        if body.get('id') != sim.active: raise ValueError('job_not_active')
                        sim.cancel.set()
                    self.reply({'ok':True})
                else:self.reply({'error':'not_found'},404)
            except ValueError as e:self.reply({'error':str(e)},400)
            except RuntimeError as e:self.reply({'error':str(e)},409)
    server=ThreadingHTTPServer(('127.0.0.1',8772),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    sim.run()

if __name__=='__main__':main()
