#!/usr/bin/env python3
"""Start the deployed Bracketbot runner over bb SSH; credentials travel on stdin.

Run through with-home-credentials.py. The robot's BBOS camera/depth/SLAM daemons
must already be active. This starts only the isolated Domain Harness runner.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess

REMOTE = r'''
import json, os, signal, subprocess, sys, time
from pathlib import Path
request=json.load(sys.stdin)
root=Path(request['root']); source=root/'source'; state=root/'state'
state.mkdir(parents=True,exist_ok=True,mode=0o700)
os.chmod(state,0o700)
binary=root/'bin/bracketbot-runner'
if not binary.is_file(): raise SystemExit('Build bracketbot-runner before launching')
# Validate before stopping a healthy runner. Never silently launch without Home.
home=request.get('home')
if home is None:
    profile=state/'home-connection.json'
    if not profile.is_file(): raise SystemExit('Missing Home connection; pass --home-connection with a robot-reachable profile')
    home=json.loads(profile.read_text())
if (home.get('schema') != 'spatial-harness.home-connection/v1'
        or not home.get('peer_id') or not home.get('frame') or not home.get('product_id')
        or not home.get('routes')
        or all(route.startswith(('/ip4/127.', '/ip4/0.0.0.0/', '/ip6/::1/', '/ip6/::/')) for route in home['routes'])):
    raise SystemExit('Home connection must contain its identity and a robot-reachable route, not only loopback/wildcard addresses')
pidfile=state/'runner.pid'
if pidfile.exists():
    pid=int(pidfile.read_text())
    try:
        cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
        owned=b'supervise-bracketbot.py' in cmd and str(state).encode() in cmd
    except FileNotFoundError: owned=False
    if owned:
        if not request['restart']: raise SystemExit('Runner already active; use --restart to update this deployment')
        os.kill(pid,signal.SIGTERM)
        until=time.monotonic()+100
        while Path(f'/proc/{pid}').exists() and time.monotonic()<until: time.sleep(.25)
        if Path(f'/proc/{pid}').exists(): raise SystemExit('Previous supervisor is still stopping')
config=request['peer']; config['identity_file']=str(state/'peer.identity')
(state/'peer.json').write_text(json.dumps(config,indent=2)+'\n')
(state/'home-connection.json').write_text(json.dumps(home,indent=2)+'\n')
(state/'connection.json').unlink(missing_ok=True)
env=os.environ.copy(); env.update(request['credentials'])
env['SPATIAL_ROBOT_NAME']=request.get('robot_name','Bracketbot')
env['SPATIAL_HOME_CONNECTION']=str(state/'home-connection.json')
env.pop('SPATIAL_DRIVE_PEER',None)
drive=request.get('drive_peer')
if drive: env['SPATIAL_DRIVE_PEER']=drive  # the runner caches only this phone's stick
with (state/'supervisor.log').open('ab',buffering=0) as log:
    child=subprocess.Popen(['/usr/bin/python3',str(source/'scripts/supervise-bracketbot.py'),
        '--state',str(state),'--binary',str(binary)] +
        (['--execute-portal-search'] if request.get('execute_portal_search') else []) +
        (['--execute-phone-reach'] if request.get('execute_phone_reach') else []) +
        (['--execute-phone-drive','--drive-peer',drive] if drive else []) +
        (['--drive-fast'] if drive and request.get('drive_fast') else []),cwd=source,env=env,
        stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
print(json.dumps({'supervisor_pid':child.pid,'state':str(state)}),flush=True)
until=time.monotonic()+45
while time.monotonic()<until:
    if child.poll() is not None: raise SystemExit('Supervisor exited; inspect supervisor.log')
    profile=state/'connection.json'
    if profile.exists():
        print(profile.read_text(),flush=True)
        break
    time.sleep(.25)
else:
    print('Runner is starting; inspect state/runner.log and connection.json for readiness.',flush=True)
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', default='0184', help='bb robot ID')
    ap.add_argument('--root', default='/home/bracketbot/deployments/domain-harness/runner')
    ap.add_argument('--robot-name', help='Display name published to Home')
    ap.add_argument('--peer-config', type=Path, default=Path('config/bracketbot-0184.peer.json'))
    ap.add_argument('--home-connection', type=Path)
    ap.add_argument('--restart', action='store_true')
    ap.add_argument('--execute-portal-search', action='store_true',
                    help='Enable wrist search after confirming empty hands and a clear workspace; phone pointing stays gated')
    ap.add_argument('--execute-phone-reach', action='store_true', help='Enable persistent full-arm reaching to the selected note or nearest phone, 20 cm short')
    ap.add_argument('--execute-phone-drive', action='store_true', help='Let one phone drive the base with its joystick. Off on every restart unless given again.')
    ap.add_argument('--drive-peer', help='Peer ID of the only phone allowed to drive')
    ap.add_argument('--drive-fast', action='store_true', help='Full teleop speeds instead of half')
    args = ap.parse_args()
    if not Path(args.root).is_absolute(): ap.error('--root must be absolute')
    if args.execute_phone_drive and not args.drive_peer: ap.error('--execute-phone-drive needs --drive-peer')
    peer = json.loads(args.peer_config.read_text())
    names = ('AUKI_ACCESS_KEY', 'AUKI_APP_SECRET') if peer['login'] == 'app' else ('AUKI_EMAIL', 'AUKI_PASSWORD')
    if not all(os.environ.get(name) for name in names): ap.error('Supply authentication through the environment or with-home-credentials.py')
    payload = dict(root=args.root, peer=peer, restart=args.restart,
                   robot_name=args.robot_name or f'Bracketbot {args.robot}',
                   execute_portal_search=args.execute_portal_search,
                   execute_phone_reach=args.execute_phone_reach,
                   drive_peer=args.drive_peer if args.execute_phone_drive else None,
                   drive_fast=args.drive_fast,
                   credentials={name:os.environ[name] for name in names},
                   home=json.loads(args.home_connection.read_text()) if args.home_connection else None)
    command = shlex.join(['/usr/bin/python3', '-c', REMOTE])
    subprocess.run(['bb', 'ssh', args.robot, command], input=json.dumps(payload), text=True, check=True)


if __name__ == '__main__': main()
