"""Exercise the real RAI HTTP entry and verify native ROS action receipts."""
import argparse
import json
import time
import urllib.request
from pathlib import Path

BASE='http://127.0.0.1:8770'


def call(path,data=None):
    request=urllib.request.Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=10) as response:return json.load(response)


def run(seed,fault):
    before=call('/api/roboweave/status')
    assert not before['busy'], 'another task is active'
    started=call('/api/roboweave/start',{'task':'Pick up the red cube, move it to the green goal, and verify the result.', 'seed':seed,'fault':fault})
    print('STARTED',started['task_id'],'seed',seed,'fault',fault,flush=True)
    deadline=time.monotonic()+240;previous=None
    while time.monotonic()<deadline:
        state=call('/api/roboweave/status');task=state['task']
        assert task['id']==started['task_id'], 'task replaced'
        phase=(task['status'],state['observation'].get('phase'),len(task['steps']))
        if phase!=previous:print('PROGRESS',phase,flush=True);previous=phase
        if not state['busy']:break
        time.sleep(1)
    else:raise AssertionError('task deadline exceeded')
    report={'task':task,'events':state['events']}
    directory=Path('artifacts/arm/verification');directory.mkdir(parents=True,exist_ok=True)
    (directory/f'rai-seed-{seed}-fault-{int(fault)}.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print('RESULT',json.dumps({k:task[k] for k in ('id','status','recoveries','duration_sec','trajectory_id')},ensure_ascii=True),flush=True)
    assert task['status']=='succeeded',task.get('reason')
    assert [s['skill'] for s in task['steps']]==['approach','grasp','transport','verify']
    assert all(s['success'] and s['ros_status']==4 for s in task['steps'])
    obs=task['final_observation']
    assert obs['is_grasped'] and obs['native_success'] and obs['lift_height']>.02
    assert obs['distance_to_goal']<=.025
    assert any(e['kind']=='action_feedback' for e in state['events'])
    if fault:
        assert task['recoveries']==1
        assert any(e['kind']=='action_result' and e.get('reason')=='grasp_contact_missing' and e.get('ros_status')==6 for e in state['events'])
        assert any(e['kind']=='recovery_verified' and e['success'] for e in state['events'])
    else:assert task['recoveries']==0
    print('PASS',started['task_id'],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,default=0);parser.add_argument('--fault',action='store_true');args=parser.parse_args()
    run(args.seed,args.fault)
