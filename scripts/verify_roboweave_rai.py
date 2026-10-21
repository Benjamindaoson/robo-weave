"""Real HTTP -> RAI -> ROS2 Action -> turtlesim acceptance checks."""
import json,time,urllib.request,urllib.error
from pathlib import Path

BASE='http://127.0.0.1:8768'
def request(path,data=None):
    req=urllib.request.Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=10) as response:return json.load(response)
def wait_task(timeout=130):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        state=request('/api/roboweave/status')
        if not state['busy']:return state
        time.sleep(1)
    raise TimeoutError('task_timeout')

receipts=[]
for fault,task in [(False,'依次移动到 (8,7) 和 (3,8)，严格按顺序执行。'),(True,'依次移动到 (7,3) 和 (4,6)，严格按顺序执行。')]:
    request('/api/roboweave/start',{'task':task,'fault':fault})
    state=wait_task();t=state['task'];receipts.append(t)
    assert t['status']=='succeeded',t
    assert len(t['verified_goals'])==2,t
    assert t['recoveries']==int(fault),t
    events=state['events']
    assert any(e['kind']=='action_feedback' for e in events)
    if fault:
        assert any(e['kind']=='action_result' and e.get('ros_status')==6 and e.get('reason')=='injected_transient_failure' for e in events)
        assert any(e['kind']=='reobserve_and_retry' for e in events)
    print(json.dumps({'case':'recovery' if fault else 'normal','task':t['id'],'status':t['status'],'recoveries':t['recoveries'],'duration':t['duration_sec']}),flush=True)

request('/api/roboweave/start',{'task':'移动到 (10,1)，到达后验证位置。'})
deadline=time.monotonic()+45
while time.monotonic()<deadline:
    s=request('/api/roboweave/status')
    if any(e['kind']=='action_feedback' for e in s['events']):break
    time.sleep(.2)
else:raise AssertionError('No running action to cancel')
start=time.monotonic();request('/api/roboweave/stop',{})
state=wait_task();t=state['task'];receipts.append(t)
assert t['status']=='stopped',t
assert any(e['kind']=='action_result' and e.get('ros_status')==5 for e in state['events']),state['events']
time.sleep(1)
pose=request('/api/roboweave/status')['observation']['pose']
assert abs(pose['linear_velocity'])<.001 and abs(pose['angular_velocity'])<.001,pose
print(json.dumps({'case':'cancel','task':t['id'],'status':t['status'],'stop_elapsed_sec':round(time.monotonic()-start,2),'pose':pose}),flush=True)
Path('artifacts/roboweave/acceptance.json').write_text(json.dumps(receipts,ensure_ascii=False,indent=2))
