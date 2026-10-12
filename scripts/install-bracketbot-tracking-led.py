#!/usr/bin/env python3
"""Deploy a LED-only adapter and add supervisor lifecycle hooks incrementally."""
import shlex
import subprocess
import tarfile
import tempfile
from pathlib import Path

REMOTE=r'''
from pathlib import Path
import os,subprocess
root=Path('/home/bracketbot/deployments/domain-harness/runner')
path=root/'source/scripts/supervise-bracketbot.py'
text=path.read_text()
def patch(old,new):
    global text
    if text.count(old)!=1:raise RuntimeError('Unexpected supervisor anchor')
    text=text.replace(old,new,1)
if 'tracking_led = None' not in text:
    patch('        drive = None','        drive = None\n        tracking_led = None')
    patch('            if drive and drive.poll() is None: drive.terminate()',
          '            if drive and drive.poll() is None: drive.terminate()\n            if tracking_led and tracking_led.poll() is None: tracking_led.terminate()')
    patch('            heartbeat(False)\n            behavior_args',
          "            heartbeat(False)\n            tracking_led = subprocess.Popen([sys.executable,str(Path(__file__).with_name('bracketbot-python.py')),str(Path(__file__).with_name('bracketbot_tracking_led.py')),'--state',str(args.state)],stdin=subprocess.DEVNULL)\n            behavior_args")
    patch('        finally:\n            heartbeat(False)',
          '        finally:\n            heartbeat(False)\n            if tracking_led and tracking_led.poll() is None:\n                tracking_led.terminate()\n                try: tracking_led.wait(timeout=4)\n                except subprocess.TimeoutExpired:\n                    tracking_led.kill(); tracking_led.wait()')
    temporary=path.with_suffix('.led.tmp');temporary.write_text(text);temporary.replace(path)
# The existing supervisor need not restart: start its LED-only companion now.
state=root/'state'
with (state/'tracking-led.log').open('ab',buffering=0) as log:
    child=subprocess.Popen(['/usr/bin/python3',str(root/'source/scripts/bracketbot-python.py'),
        str(root/'source/scripts/bracketbot_tracking_led.py'),'--state',str(state)],
        stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
print({'tracking_led_pid':child.pid,'runner_restarted':False,'bbos_unchanged':True},flush=True)
'''

if __name__=='__main__':
    repo=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        archive=Path(directory)/'tracking-led.tar'
        with tarfile.open(archive,'w') as tar:
            tar.add(repo/'scripts/bracketbot_tracking_led.py',arcname='bracketbot_tracking_led.py')
        with archive.open('rb') as payload:
            subprocess.run(['bb','ssh','0184','tar -xf - -C /home/bracketbot/deployments/domain-harness/runner/source/scripts'],stdin=payload,check=True,timeout=30)
        subprocess.run(['bb','ssh','0184',shlex.join(['/usr/bin/python3','-c',REMOTE])],check=True,timeout=30)
