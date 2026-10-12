#!/usr/bin/env python3
"""Activate the reconciled Home build after explicit restart approval.

Preserves the production journal/peer identity and LAN settings. Does not touch
the robot. Refuses to terminate a PID unless it owns the expected listener.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path('/Users/nilspihl/Documents/ChatGPT/harness')
binary = root / 'target/release/spatial-home'
old_binary = str(binary)
pid = 65640
assert binary.is_file(), 'Replacement binary missing'
command = subprocess.check_output(['ps', '-p', str(pid), '-o', 'command='], text=True).strip()
assert command == old_binary, 'Home process changed; inspect before restarting'
listeners = subprocess.check_output(['lsof','-nP','-t','-iTCP:4329','-sTCP:LISTEN'],text=True).split()
assert listeners == [str(pid)], 'Unexpected or duplicate Home listeners'
os.kill(pid,signal.SIGTERM)
deadline = time.monotonic()+25
while time.monotonic()<deadline:
    if subprocess.run(['kill','-0',str(pid)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
        break
    time.sleep(.25)
else:
    raise SystemExit('Home did not exit cleanly; no replacement started')
assert subprocess.run(['lsof','-nP','-t','-iTCP:4329','-sTCP:LISTEN'],stdout=subprocess.DEVNULL).returncode != 0
env = os.environ.copy()
env.update(SPATIAL_BIND='0.0.0.0',SPATIAL_UI_HOSTS='172.20.10.3:4329,192.168.1.26:4329',
           SPATIAL_PORT='4329',SPATIAL_DATA='data/bracketbot-verification/home.json',
           SPATIAL_PEER_CONFIG='data/bracketbot-verification/home-peer.json',
           SPATIAL_CREDENTIAL_FILE=str(root/'data/homeserver.rtf'))
with (root/'data/bracketbot-verification/home-volume-editor.log').open('ab') as log:
    process=subprocess.Popen(['python3',str(root/'scripts/with-home-credentials.py'),str(binary)],
        cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
print({'home_pid':process.pid,'binary':str(binary),'lan':True,'robot_untouched':True},flush=True)
