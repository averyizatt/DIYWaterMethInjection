"""Pull the signals of interest out of an LTspice transient .raw file with stepped runs.

usage: ltraw.py <file.raw> <out folder>
Writes one file per step, step_NN.bin: little-endian float32 triples (time s, V(adc), V(vout), V(sensor)).
The .raw layout (LTspice, "real forward stepped"): a UTF-16LE text header ending in
"Binary:\\n", then for every point the time as a float64 and each other variable as a float32.
"""
import struct
import sys
from pathlib import Path

source, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
marker = 'Binary:\n'.encode('utf-16-le')
with open(source, 'rb') as f:
    head = f.read(65536)
    start = head.find(marker) + len(marker)
    text = head[:start].decode('utf-16-le')
    names = []
    listing = False
    count = points = 0
    for line in text.splitlines():
        if line.startswith('No. Variables:'):
            count = int(line.split(':')[1])
        elif line.startswith('No. Points:'):
            points = int(line.split(':')[1])
        elif line.startswith('Variables:'):
            listing = True
        elif line.startswith('Binary:'):
            listing = False
        elif listing and line.strip():
            names.append(line.split()[1].lower())
    assert len(names) == count and names[0] == 'time', (count, names[:3])
    record = 8 + 4 * (count - 1)
    wanted = [names.index(name) for name in ('v(adc)', 'v(vout)', 'v(sensor)')]
    offsets = [8 + 4 * (index - 1) for index in wanted]
    f.seek(start)
    step, last, written = 0, -1.0, 0
    target = None
    pack = struct.Struct('<4f').pack
    time_at = struct.Struct('<d').unpack_from
    value_at = struct.Struct('<f').unpack_from
    chunk_records = 32768
    remaining = points
    while remaining > 0:
        take = min(chunk_records, remaining)
        data = f.read(take * record)
        assert len(data) == take * record, 'file shorter than its header says'
        remaining -= take
        buffer = bytearray()
        for n in range(take):
            base = n * record
            t = abs(time_at(data, base)[0])   # LTspice marks some points with a negative time
            if t < last or target is None:    # Time starts again: the next stepped run
                if target is not None:
                    target.write(buffer)
                    target.close()
                    buffer = bytearray()
                step += 1
                target = open(out / f'step_{step:02d}.bin', 'wb')
            last = t
            buffer += pack(t, *(value_at(data, base + offset)[0] for offset in offsets))
            written += 1
        target.write(buffer)
    target.close()
print(f'{written} points in {step} steps, {count} variables, record {record} bytes')
