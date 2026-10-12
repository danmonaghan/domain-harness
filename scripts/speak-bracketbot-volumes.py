#!/usr/bin/env python3
"""Announce Home volume entries independently of capture, mapping and motors.

Offline backend: espeak-ng (or espeak), installed separately on the robot.
Playback uses BBOS's speaker daemon, not exclusive ALSA device access.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from bracketbot_volume_speech import Entries, inputs


def read(path):
    return json.loads(path.read_text())


def speak(text, backend, stopped):
    # Text is stdin, never a shell command or command-line option.
    with tempfile.TemporaryDirectory(prefix='bracketbot-speech-') as directory:
        wave = str(Path(directory)/'speech.wav')
        playback = [sys.executable, str(Path(__file__).with_name('bracketbot-python.py')),
                    '--daemon', 'speaker', str(Path(__file__).with_name('play-bracketbot-speech.py')), wave]
        for command, stdin in (([backend, '--stdin', '-w', wave], text.encode()), (playback, None)):
            process = subprocess.Popen(command, stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                if stdin:
                    process.stdin.write(stdin); process.stdin.close()
                deadline = time.monotonic()+180
                while process.poll() is None:
                    if stopped.wait(.05) or time.monotonic() > deadline:
                        raise RuntimeError('Speech cancelled or timed out')
                if process.returncode: raise RuntimeError('Speech synthesis/playback failed')
            finally:
                if process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=1)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--state', required=True, type=Path)
    args = ap.parse_args()
    state = args.state
    supervisor=(state/'runner.pid').read_text().strip()
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_: stopped.set())
    backend = shutil.which('espeak-ng') or shutil.which('espeak')
    entries = Entries()
    pending = {}
    mutex = threading.Lock()
    condition = threading.Condition(mutex)
    active = set()
    report = {'backend': backend, 'status':'waiting_for_localization' if backend else 'speech_backend_unavailable'}

    def worker():
        while not stopped.is_set():
            with condition:
                condition.wait_for(lambda: bool(pending) or stopped.is_set(), timeout=.2)
                if stopped.is_set(): break
                if not pending: continue
                identity, text = next(iter(pending.items()))
                del pending[identity]
                if identity not in active: continue
            try:
                speak(text, backend, stopped)
                with mutex: report.update(last_spoken_volume=identity, error=None)
            except (OSError, RuntimeError) as error:
                with mutex: report.update(error=str(error))

    with (state/'volume-speech.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        thread = threading.Thread(target=worker, daemon=True)
        if backend: thread.start()
        else: backend = None
        try:
            while not stopped.is_set():
                if not (Path('/proc')/supervisor).exists() or not (state/'runner.pid').exists() or (state/'runner.pid').read_text().strip()!=supervisor:
                    break
                valid = None
                scope = entries.scope
                try:
                    profile = read(state/'home-connection.json')
                    scope = (profile['peer_id'], profile['frame'], profile['product_id'])
                    control = state/'robot-control.json'
                    valid = inputs(read(control), read(state/'semantic-volumes.json'), profile,
                                   now_ns=time.time_ns(), capture_now_ns=time.time_ns(),
                                   scene_age_ms=(time.time_ns()-control.stat().st_mtime_ns)/1e6)
                    announcements = entries.update(*(valid or (None, None)), time.monotonic(), scope)
                except (OSError, KeyError, ValueError, TypeError):
                    announcements = entries.update(None, None, time.monotonic(), scope)
                    valid = None
                with condition:
                    active = set(entries.inside) if valid else set()
                    pending = {key:text for key,text in pending.items() if key in active}
                    if backend: pending.update(announcements)
                    report.update(status=('tracking' if valid else 'waiting_for_localization')
                                  if backend else 'speech_backend_unavailable',
                                  inside=sorted(active), pending=sorted(pending), updated_ns=time.time_ns())
                    condition.notify()
                    status = dict(report)
                tmp = state/'volume-speech-status.tmp'
                tmp.write_text(json.dumps(status)); tmp.replace(state/'volume-speech-status.json')
                stopped.wait(.1)
        finally:
            stopped.set()
            with condition: condition.notify_all()
            if thread.is_alive(): thread.join(timeout=3)


if __name__ == '__main__': main()
