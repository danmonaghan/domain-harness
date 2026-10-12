#!/usr/bin/env python3
"""Red LED on BBOS tracking loss; release the LED on recovery or shutdown."""
import argparse
import fcntl
import signal
import time
from pathlib import Path


def tracking_lost(health, pose_ns, health_ns, now_ns):
    if health is None or pose_ns is None or health_ns is None:
        return True
    if any(not -100_000_000 <= now_ns-t <= 800_000_000 for t in (pose_ns,health_ns)):
        return True
    return bool(health['vo_lost']) or bool(health['stalled']) or not bool(health['localized'])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--state',type=Path,required=True)
    args=parser.parse_args()
    from bbos import Reader,Writer,Type
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    with (args.state/'tracking-led.lock').open('a') as lock:
        deadline=time.monotonic()+3
        while True:
            try:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);break
            except BlockingIOError:
                if time.monotonic()>=deadline:raise
                time.sleep(.1)
        supervisor=(args.state/'runner.pid').read_text().strip()
        health=pose_ns=health_ns=None
        writer=None;last_sent=0
        readers=[]
        try:
            for topic in ('slam.health','slam.pose'):
                reader=Reader(topic,keeptime=False);reader.__enter__();readers.append(reader)
            while not stopped and (Path('/proc')/supervisor).exists() and (args.state/'runner.pid').read_text().strip()==supervisor:
                if readers[0].ready():
                    health=readers[0].data.copy()
                    health_ns=int(health['timestamp'].astype('int64'))
                if readers[1].ready():
                    pose_ns=int(readers[1].data['timestamp'].astype('int64'))
                lost=tracking_lost(health,pose_ns,health_ns,time.time_ns())
                if lost and writer is None:
                    # Writer enforces exclusive topic ownership; do not bypass it.
                    writer=Writer('led.ctrl',Type('led_ctrl'));writer.__enter__()
                    with writer.buf() as b:
                        b['rgb']=[255,0,0];b['brightness']=-1;b['period_ms']=0
                        # Older BBOS exposes only rgb/brightness/period_ms.
                        for name,value in [('animation',b''),('pixels',0),('charging_level',0)]:
                            if name in b.dtype.names:b[name]=value
                    print('Tracking LED: red',flush=True)
                    last_sent=time.monotonic()
                elif lost and writer is not None and time.monotonic()-last_sent>=1:
                    # Old BBOS expires control intents after three seconds.
                    with writer.buf() as b:
                        b['rgb']=[255,0,0];b['brightness']=-1;b['period_ms']=0
                    last_sent=time.monotonic()
                elif not lost and writer is not None:
                    writer.__exit__(None,None,None);writer=None
                    print('Tracking LED: recovered; BBOS default restored',flush=True)
                time.sleep(.1)
        finally:
            if writer is not None:writer.__exit__(None,None,None)
            for reader in readers:reader.__exit__(None,None,None)


if __name__=='__main__':main()
