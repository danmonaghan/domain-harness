#!/usr/bin/env python3
"""Restart the isolated runner with a fresh map session after source resets."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--state', type=Path, required=True)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--execute-portal-search', action='store_true',
                    help='Enable supervised wrist search; requires a clear workspace and empty hands')
    ap.add_argument('--execute-phone-reach', action='store_true')
    ap.add_argument('--execute-phone-drive', action='store_true')
    ap.add_argument('--drive-peer')
    ap.add_argument('--drive-fast', action='store_true')
    ap.add_argument('--disable-volume-speech', action='store_true')
    args = ap.parse_args()
    args.state.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (args.state/'supervisor.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        pidfile = args.state/'runner.pid'
        pidfile.write_text(str(os.getpid()))
        stopped = False
        child = None
        behavior = None
        drive = None
        speech = None
        tracking_led = None
        def heartbeat(running):
            tmp = args.state/'supervisor-live.tmp'
            tmp.write_text(json.dumps(dict(pid=os.getpid(), updated_ns=time.time_ns(), running=running)))
            tmp.replace(args.state/'supervisor-live.json')
        def stop(*_):
            nonlocal stopped
            stopped = True
            if child and child.poll() is None: child.terminate()
            if behavior and behavior.poll() is None: behavior.terminate()
            if drive and drive.poll() is None: drive.terminate()
            if speech and speech.poll() is None: speech.terminate()
            if tracking_led and tracking_led.poll() is None: tracking_led.terminate()
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        failures = 0
        try:
            heartbeat(False)
            tracking_led = subprocess.Popen([sys.executable,str(Path(__file__).with_name('bracketbot-python.py')),
                str(Path(__file__).with_name('bracketbot_tracking_led.py')),'--state',str(args.state)],stdin=subprocess.DEVNULL)
            behavior_args = [sys.executable, str(Path(__file__).with_name('control-bracketbot-behavior.py')),
                             '--state', str(args.state), '--supervisor-pid', str(os.getpid())]
            if args.execute_portal_search: behavior_args.append('--execute-search')
            if args.execute_phone_reach: behavior_args.append('--execute-phone-reach')
            # Never automatically restart a failed arm controller.
            behavior = subprocess.Popen(behavior_args, stdin=subprocess.DEVNULL)
            if not args.disable_volume_speech:
                speech = subprocess.Popen([sys.executable,
                    str(Path(__file__).with_name('speak-bracketbot-volumes.py')),
                    '--state', str(args.state)], stdin=subprocess.DEVNULL)
            if args.execute_phone_drive and args.drive_peer:
                # Like the arm controller, a failed drive controller is never restarted.
                drive = subprocess.Popen([sys.executable, str(Path(__file__).with_name('drive-bracketbot.py')),
                    '--state', str(args.state), '--driver-peer', args.drive_peer, '--execute'] +
                    (['--fast'] if args.drive_fast else []),
                    stdin=subprocess.DEVNULL)
            while not stopped:
                log = args.state/'runner.log'
                if log.exists() and log.stat().st_size > 8*1024*1024:
                    log.replace(args.state/'runner.previous.log')
                started = time.monotonic()
                with log.open('ab', buffering=0) as output:
                    child = subprocess.Popen([str(args.binary), str(args.state/'peer.json'),
                        str(args.state/'local-scene.json'), str(args.state/'connection.json')],
                        stdin=subprocess.DEVNULL, stdout=output, stderr=output)
                    while child.poll() is None:
                        heartbeat(not stopped)
                        if stopped:
                            try: child.wait(timeout=25)
                            except subprocess.TimeoutExpired:
                                child.kill(); child.wait()
                            break
                        time.sleep(.25)
                heartbeat(False)
                (args.state/'supervisor-status.json').write_text(json.dumps({
                    'pid': os.getpid(), 'last_exit': child.returncode,
                    'stopped': stopped, 'last_runtime_seconds': round(time.monotonic()-started, 2)}))
                failures = 0 if time.monotonic()-started > 30 else failures+1
                delay = min(30, 2**min(failures, 5))
                until = time.monotonic()+delay
                while not stopped and time.monotonic() < until: time.sleep(.25)
        finally:
            heartbeat(False)
            if tracking_led and tracking_led.poll() is None:
                tracking_led.terminate()
                try: tracking_led.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    tracking_led.kill(); tracking_led.wait()
            if speech and speech.poll() is None:
                speech.terminate()
                try: speech.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    speech.kill(); speech.wait()
            # The base stops first: the drive controller zeroes its topic on exit.
            if drive and drive.poll() is None:
                drive.terminate()
                try: drive.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    drive.kill(); drive.wait()
            if behavior and behavior.poll() is None:
                behavior.terminate()
                try: behavior.wait(timeout=68)
                except subprocess.TimeoutExpired:
                    behavior.kill(); behavior.wait()
            pidfile.unlink(missing_ok=True)


if __name__ == '__main__': main()
