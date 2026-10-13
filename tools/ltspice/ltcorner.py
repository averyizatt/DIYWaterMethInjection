"""From the LTspice run itself: how much of the sensor's own (unloaded) signal reaches the
amplifier input at each of the simulation's three tones. usage: ltcorner.py <file.raw>"""
import cmath
import math
import struct
import sys

path = sys.argv[1]
marker = 'Binary:\n'.encode('utf-16-le')
with open(path, 'rb') as f:
    head = f.read(65536)
    start = head.find(marker) + len(marker)
    names = []
    listing = False
    for line in head[:start].decode('utf-16-le').splitlines():
        if line.startswith('Variables:'):
            listing = True
        elif line.startswith('Binary:'):
            listing = False
        elif listing and line.strip():
            names.append(line.split()[1].lower())
    record = 8 + 4 * (len(names) - 1)
    cols = [8 + 4 * (names.index(n) - 1) for n in ('v(vin)', 'v(vref)', 'i(bbase)', 'i(bcomb)', 'i(bknock)')]
    f.seek(start)
    step, last, rows = 0, -1.0, []
    while step <= 10:
        data = f.read(record * 16384)
        if not data:
            break
        for n in range(len(data) // record):
            base = n * record
            t = abs(struct.unpack_from('<d', data, base)[0])
            if t < last:
                step += 1
            last = t
            if step == 10 and 0.020 <= t <= 0.050:      # sel=1, AMP=1; the knock-free 30 ms
                rows.append((t, *[struct.unpack_from('<f', data, base + c)[0] for c in cols]))

C_SOURCE = 1.15e-9 + 200e-12
print(f'{len(rows)} points; sensor + cable {C_SOURCE * 1e9:.2f} nF, coupling 1.5 nF, 47 k to the bias point')
print(' tone       reaches the amplifier input    a 4.77 kHz corner predicts    a 1.9 kHz corner would give')
for hz in (2300.0, 6000.0, 9100.0):
    w = 2 * math.pi * hz
    vin = cur = 0j
    for (t0, v0, r0, a0, b0, c0), (t1, v1, r1, a1, b1, c1) in zip(rows, rows[1:]):
        dt = t1 - t0
        e0, e1 = cmath.exp(-1j * w * t0), cmath.exp(-1j * w * t1)
        vin += 0.5 * dt * ((v0 - r0) * e0 + (v1 - r1) * e1)
        cur += 0.5 * dt * ((a0 + b0 + c0) * e0 + (a1 + b1 + c1) * e1)
    unloaded = cur / (1j * w * C_SOURCE)       # What the sensor would give into nothing
    ratio = abs(vin / unloaded)

    def model(corner):
        return (hz / corner) / math.sqrt(1 + (hz / corner) ** 2)

    print(f' {hz / 1000:4.1f} kHz    {ratio * 100:5.1f} %  ({20 * math.log10(ratio):+5.1f} dB)           '
          f'{model(4770) * 100:5.1f} %                      {model(1900) * 100:5.1f} %')
