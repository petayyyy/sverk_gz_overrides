#!/usr/bin/env python3
"""Register the local 4902 airframe in a PX4 source checkout; build afterwards."""
import argparse
from pathlib import Path
import re
import shutil

def install(px4):
    source = Path(__file__).resolve().parents[1]/'px4/porter_D2/4902_gz_porter_D2'
    folder = px4/'ROMFS/px4fmu_common/init.d-posix/airframes'
    cmake = folder/'CMakeLists.txt'
    if not cmake.is_file():
        raise ValueError('PX4 POSIX airframe CMakeLists.txt not found')
    name = source.name
    conflicts = [p for p in folder.glob('4902_*') if p.name != name]
    if conflicts:
        raise ValueError('Local airframe ID 4902 is already used: '+str(conflicts))
    target = folder/name
    if target.exists() and target.read_bytes() != source.read_bytes():
        # Preserve an existing customization for review before replacing.
        backup = target.with_suffix('.before_porter_D2')
        if backup.exists():
            raise ValueError('Existing customized profile and backup found: '+str(backup))
        shutil.copyfile(target, backup)
    content = cmake.read_text(encoding='utf-8')
    if not re.search(r'^\s*'+re.escape(name)+r'\s*$', content, re.M):
        match = re.search(r'px4_add_romfs_files\s*\(', content)
        if not match:
            raise ValueError('Unsupported PX4 airframe CMake format')
        content = content[:match.end()]+'\n\t'+name+content[match.end():]
        cmake.write_text(content, encoding='utf-8')
    shutil.copyfile(source, target)
    target.chmod(0o755)
    print('Registered '+str(target)+'; rebuild with make px4_sitl_default')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('px4_dir', type=Path)
    install(parser.parse_args().px4_dir.resolve())
