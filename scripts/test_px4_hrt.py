#!/usr/bin/env python3
"""Check interval-change scheduling against the actual pinned POSIX HRT source."""
from pathlib import Path
import argparse
import json
import shlex
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('px4', type=Path)
parser.add_argument('--output', type=Path, default=Path('/tmp/test-px4-hrt'))
args = parser.parse_args()
px4 = args.px4.resolve()
build = px4 / 'build/px4_sitl_default'
output = args.output
output.parent.mkdir(parents=True, exist_ok=True)
commands = build / 'compile_commands.json'
if not commands.is_file():
    raise SystemExit('Build the patched PX4 target first: make px4_sitl_default')
row = next(item for item in json.loads(commands.read_text())
           if item['file'].endswith('/posix/src/px4/common/drv_hrt.cpp'))
# Reuse the board's includes/defines. The test drives timer callbacks directly,
# so it needs neither the simulator lockstep clock nor the HRT worker thread.
flags = [flag for flag in shlex.split(row['command'])
         if flag.startswith('-I') or (flag.startswith('-D') and flag not in
                                      ['-DENABLE_LOCKSTEP_SCHEDULER', '-DNDEBUG'])]
cmd = ['/usr/bin/c++', *flags, '-std=gnu++17', '-O0', '-g',
       '-ffunction-sections', '-fdata-sections', '-include', 'visibility.h']
for name, relative in [
    ('PX4_HRT_SOURCE', 'platforms/posix/src/px4/common/drv_hrt.cpp'),
    ('PX4_QUEUE_SOURCE', 'platforms/common/work_queue/queue.c'),
    ('PX4_ADD_AFTER_SOURCE', 'platforms/common/work_queue/sq_addafter.c'),
    ('PX4_ADD_LAST_SOURCE', 'platforms/common/work_queue/sq_addlast.c'),
]:
    cmd.append(f'-D{name}="{px4 / relative}"')
cmd += [str(root / 'tests/cpp/test_px4_hrt_reschedule.cpp'),
        '-Wl,--gc-sections', '-o', str(output)]
subprocess.run(cmd, check=True)
subprocess.run([str(output)], check=True)
