#!/usr/bin/env python3
"""First install: build current sources on laptop and ship ARM64 binary + source.

Uses already cached vendored dependencies. Refuses to overwrite a deployment.
Never copies a robot's state, identity, credential files or calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import tarfile
import tempfile

REMOTE=r'''
import sys,tarfile,subprocess,json,hashlib
from pathlib import Path
root=Path('/home/bracketbot/deployments/domain-harness/runner')
if root.exists():raise SystemExit('Deployment already exists; inspect before replacing it')
root.mkdir(parents=True)
with tarfile.open(fileobj=sys.stdin.buffer,mode='r|gz') as archive:
    for entry in archive:
        if entry.issym() or entry.islnk() or not (root/entry.name).resolve().is_relative_to(root.resolve()):raise SystemExit('Unsafe archive')
        archive.extract(entry,root)
binary=root/'bin/bracketbot-runner';binary.chmod(0o755)
manifest=json.loads((root/'build-manifest.json').read_text())
assert hashlib.sha256(binary.read_bytes()).hexdigest()==manifest['binary_sha256']
check=subprocess.run(['ldd',str(binary)],capture_output=True,text=True)
assert check.returncode==0 and 'not found' not in check.stdout,check.stdout+check.stderr
print(json.dumps(dict(installed=str(binary),sha256=manifest['binary_sha256'],compiled_on_laptop=True)),flush=True)
'''

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--robot',required=True);args=parser.parse_args()
    repo=Path(__file__).resolve().parents[1]
    cache=Path('/Users/nilspihl/Documents/ChatGPT/harness/data/bracketbot-linux-build')
    paths=['Cargo.toml','Cargo.lock','src','scripts','web','config','vendor',
           'ios/PortalBridge/Cargo.toml','ios/PortalBridge/src']
    def include(entry):
        if any(p in ('__pycache__','.git','.DS_Store','target','.deps') for p in Path(entry.name).parts):return None
        if entry.issym() or entry.islnk():raise ValueError('Unexpected source symlink')
        return entry
    with tempfile.TemporaryDirectory(prefix='bracketbot-install-') as directory:
        root=Path(directory);source=root/'source';source.mkdir()
        source_archive=root/'source.tar.gz'
        with tarfile.open(source_archive,'w:gz') as tar:
            for path in paths:tar.add(repo/path,arcname=path,filter=include)
        with tarfile.open(source_archive,'r:gz') as tar:tar.extractall(source,filter='data')
        (source/'.cargo').mkdir()
        shutil.copyfile(cache/'source/.cargo/config.toml',source/'.cargo/config.toml')
        subprocess.run(['docker','run','--rm','--platform','linux/arm64','-v',str(source)+':/work',
            '-v',str(cache/'source/.deps')+':/work/.deps:ro','-v',str(cache/'target')+':/build-target',
            '-w','/work','-e','CARGO_TARGET_DIR=/build-target','-e','CARGO_PROFILE_DEV_DEBUG=0',
            'rust:1.91-bullseye','cargo','build','--locked','--offline','--jobs','4','--bin','bracketbot-runner'],check=True)
        binary=cache/'target/debug/bracketbot-runner'
        manifest=dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),robot=args.robot,compiled_on_laptop=True)
        manifest_path=root/'build-manifest.json';manifest_path.write_text(json.dumps(manifest,indent=2))
        deployment=root/'deployment.tar.gz'
        with tarfile.open(deployment,'w:gz') as tar:
            for path in paths:tar.add(source/path,arcname='source/'+path,filter=include)
            tar.add(binary,arcname='bin/bracketbot-runner');tar.add(manifest_path,arcname='build-manifest.json')
        with deployment.open('rb') as payload:
            subprocess.run(['bb','--llm','ssh',args.robot,shlex.join(['/usr/bin/python3','-c',REMOTE])],stdin=payload,check=True,timeout=120)

if __name__=='__main__':main()
