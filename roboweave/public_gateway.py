"""Authenticated, allowlisted public presentation gateway for RoboWeave."""
import asyncio
import hmac
import os
import re
import time
from urllib.parse import parse_qs, urlsplit

import httpx
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import HTMLResponse, RedirectResponse, Response, JSONResponse
from starlette.routing import Route

PASSWORD = os.environ['ROBOWEAVE_SHARE_PASSWORD']
TOKEN = os.environ['ROBOWEAVE_SHARE_TOKEN']
UPSTREAM = 'http://127.0.0.1:8768'
attempts = {}
LOGIN = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>RoboWeave · 访问验证</title><style>body{margin:0;min-height:100vh;display:grid;place-items:center;background:#f3f7fc;color:#172b4d;font:16px system-ui}main{width:min(360px,80vw);padding:40px;background:white;border:1px solid #dce5ef;border-radius:22px;box-shadow:0 16px 60px #234b7510}small{color:#2675ed;font-weight:700;letter-spacing:2px}h1{font-size:30px;margin-bottom:8px}p{color:#6b7e95;line-height:1.7}input,button{box-sizing:border-box;width:100%;padding:14px;border-radius:10px;font:inherit;margin-top:12px}input{border:1px solid #cbd7e5}button{background:#2469eb;color:white;border:0;cursor:pointer}</style><main><small>ROBOTICS · AGENT OS</small><h1>RoboWeave</h1><p>真实机器人仿真与 Agent 执行工作台。<br>请输入分享者提供的访问码。</p><form method="post" action="/login"><input name="password" type="password" autocomplete="current-password" placeholder="访问码" aria-label="访问码" required><button>进入演示</button></form></main></html>'''
POLL_SHIM = '''<script>
class RoboWeavePollSource {
 constructor(url){this.url=url;this.handlers={};this.closed=false;setTimeout(()=>this.poll(),0)}
 addEventListener(type,fn){(this.handlers[type]??=[]).push(fn)}
 emit(type,data){(this.handlers[type]||[]).forEach(fn=>fn({data}))}
 async poll(){if(this.closed)return;try{const r=await fetch(this.url,{cache:'no-store'});if(!r.ok)throw Error(r.status);const d=await r.json();if(!this.closed)this.emit('message',JSON.stringify(d))}catch(e){if(!this.closed)this.emit('error','')}if(!this.closed)setTimeout(()=>this.poll(),1200)}
 close(){this.closed=true}
}
</script>'''

def secured(response):
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['Referrer-Policy'] = 'same-origin'
    response.headers['X-Frame-Options'] = 'SAMEORIGIN'
    response.headers['Content-Security-Policy'] = "frame-ancestors 'self'"
    return response

async def gateway(request: Request):
    path = request.url.path
    if path == '/login':
        if request.method == 'GET':
            return secured(HTMLResponse(LOGIN))
        if request.method != 'POST':
            return Response(status_code=405)
        origin = request.headers.get('origin')
        if origin and urlsplit(origin).netloc != request.headers.get('host'):
            return Response(status_code=403)
        ip = request.headers.get('cf-connecting-ip', request.client.host)
        now = time.monotonic()
        # Bound memory and failed attempts without storing credentials.
        if len(attempts) > 1000:
            attempts.clear()
        recent = [t for t in attempts.get(ip, []) if now-t < 300]
        if len(recent) >= 20:
            return Response('Too many attempts. Retry in five minutes.', status_code=429)
        body = await request.body()
        if len(body) > 1024:
            return Response(status_code=413)
        password = parse_qs(body.decode('utf-8', errors='replace')).get('password', [''])[0]
        if not hmac.compare_digest(password.encode(), PASSWORD.encode()):
            attempts[ip] = recent + [now]
            return secured(HTMLResponse(LOGIN.replace('请输入分享者提供的访问码。','访问码不正确，请重试。'), status_code=401))
        response = RedirectResponse('/', status_code=303)
        response.set_cookie('rw_access', TOKEN, max_age=43200, secure=True, httponly=True, samesite='lax')
        return secured(response)
    if not hmac.compare_digest(request.cookies.get('rw_access', ''), TOKEN):
        if path.startswith('/api/') or request.method != 'GET':
            return JSONResponse({'error':'authentication_required'}, status_code=401)
        return RedirectResponse('/login', status_code=303)
    allowed = (request.method == 'GET' and path in ['/', '/favicon.ico', '/frame.png', '/api/roboweave/status']) or (request.method == 'POST' and path in ['/api/roboweave/start','/api/roboweave/stop'])
    if not allowed:
        return Response('Not found', status_code=404)
    if request.method == 'POST':
        origin = request.headers.get('origin')
        if not origin or urlsplit(origin).netloc != request.headers.get('host'):
            return Response('Origin rejected', status_code=403)
    if int(request.headers.get('content-length', '0')) > 16384:
        return Response(status_code=413)
    headers = {'host':request.headers['host']}
    for name in ('content-type','origin'):
        if name in request.headers:
            headers[name] = request.headers[name]
    try:
        async with httpx.AsyncClient(timeout=40, trust_env=False) as client:
            if path.endswith('/stream'):
                # Finite HTTP snapshots preserve the real upstream camera payload.
                # Quick Tunnel does not support long-lived SSE responses.
                async with client.stream('GET', UPSTREAM+path) as upstream:
                    async for line in upstream.aiter_lines():
                        if line.startswith('data: ') and '"envs"' in line:
                            return secured(Response(line[6:], media_type='application/json'))
                return JSONResponse({'envs':[]})
            upstream = await client.request(request.method, UPSTREAM+path, headers=headers, content=await request.body())
            content = upstream.content
            content_type = upstream.headers.get('content-type','application/octet-stream')
            if 'text/html' in content_type:
                html = content.decode('utf-8')
                if 'new EventSource(' in html:
                    html = html.replace('<head>', '<head>'+POLL_SHIM).replace('new EventSource(', 'new RoboWeavePollSource(')
                html = html.replace('SSH 隧道', 'HTTPS 访问保护')
                content = html.encode('utf-8')
            return secured(Response(content, status_code=upstream.status_code, headers={'content-type':content_type}))
    except (httpx.HTTPError, asyncio.TimeoutError):
        return JSONResponse({'error':'upstream_unavailable'}, status_code=502)

app = Starlette(routes=[Route('/{path:path}', gateway, methods=['GET','POST','PUT','DELETE','PATCH','OPTIONS'])])
