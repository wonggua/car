"""Verify project references, single entry point, HEX checksums and memory bounds."""
from pathlib import Path
import hashlib
import struct
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
project = root/'car.uvprojx'
xml = ET.parse(project)
assert xml.findtext('.//TargetName') == 'car'
assert xml.findtext('.//Device') == 'STM32F103C8'
paths = [(project.parent / n.text).resolve() for n in xml.findall('.//FilePath')]
assert all(p.is_file() and p.is_relative_to(root) for p in paths)
assert sum(p.name == 'main.c' for p in paths) == 1
assert all('Reference' not in p.parts and 'Tests' not in p.parts for p in paths)
assert not any(p.name in ['exti.c','hc_sr04.c'] for p in paths), 'legacy interrupt conflict'
assert list(root.rglob('*.uvprojx')) == [project]
data = (root/'car.hex').read_bytes()
assert data == (root/'Objects/car.hex').read_bytes(), 'root HEX must match latest build'
memory = {}
upper = 0
eof = False
for line in data.decode('ascii').splitlines():
    assert line.startswith(':') and not eof
    row = bytes.fromhex(line[1:])
    assert len(row) == row[0] + 5 and sum(row) % 256 == 0
    addr = int.from_bytes(row[1:3],'big')
    kind, payload = row[3], row[4:-1]
    if kind == 4: upper = int.from_bytes(payload,'big') << 16
    elif kind == 0:
        for i,value in enumerate(payload):
            address = upper + addr + i
            assert 0x08000000 <= address < 0x08010000
            assert address not in memory
            memory[address] = value
    elif kind == 1: eof = True
assert eof
sp,pc = struct.unpack('<II',bytes(memory[0x08000000+i] for i in range(8)))
assert 0x20000000 < sp <= 0x20005000 and sp % 8 == 0
assert pc & 1 and (pc & ~1) in memory
log = (root/'Objects/keil_build.log').read_text(encoding='utf-8',errors='replace')
assert '0 Error(s), 0 Warning(s).' in log
report = (f'PASS: {len(paths)} project source paths; one main; no conflicting legacy IRQ files.\n'
          f'PASS: HEX checksum, flash bounds, stack and Thumb reset vector.\n'
          f'HEX payload={len(memory)} bytes; SP=0x{sp:08X}; RESET=0x{pc:08X}\n'
          f'SHA256(car.hex)={hashlib.sha256(data).hexdigest()}\n'
          'Keil: 0 errors, 0 warnings. Hardware behavior not tested.\n')
(root/'Tests/project_verification.txt').write_text(report,encoding='utf-8')
print(report)
