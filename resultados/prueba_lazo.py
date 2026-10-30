import serial, time, sys, csv

PORT, OUT = sys.argv[1], sys.argv[2]
s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.02
s.dtr = s.rts = False
s.open()
buf = b""
rec = []
last = None


def pump(d):
    global buf, last
    e = time.time() + d
    while time.time() < e:
        buf += s.read(2048)
        while b"\n" in buf:
            l, buf = buf.split(b"\n", 1)
            p = l.decode(errors="replace").strip().split(",")
            if p[0] == "D" and len(p) == 10:
                try:
                    r = [float(v) for v in p[1:]]
                except ValueError:
                    continue
                last = r
                rec.append(r)
            elif p[0] in ("ERR", "CFG"):
                print("   <", ",".join(p))


def w(c):
    s.write((c + "\n").encode())


def metr(seg, r0, rf):
    A = rf - r0
    t0 = seg[0][0]
    ys = [(q[0] - t0) / 1000, q[2]] if False else None
    sg = 1 if A > 0 else -1
    mp = max(0, max(sg * (q[2] - rf) for q in seg)) / abs(A) * 100
    out = [i for i, q in enumerate(seg) if abs(q[2] - rf) > 0.02 * abs(A)]
    ts = None if out and out[-1] == len(seg) - 1 else ((seg[out[-1] + 1][0] - t0) / 1000 if out else 0)
    ess = rf - sum(q[2] for q in seg[-30:]) / 30
    ise = sum(q[3] ** 2 for q in seg) * 0.01
    umax = max(abs(q[4]) for q in seg)
    return ts, mp, ess, ise, umax


pump(0.8)
w("CFG")
w("LIM 25")
w("S")
pump(1.0)
allrows = []
for mode, name in ((1, "continuo-discreto"), (2, "discreto-discreto")):
    w("R 0")
    w("C %d" % mode)
    pump(1.5)
    print(f"== {name}: y en r=0 -> {last[2]:+.3f} mm")
    prev = 0.0
    for ref in (5, -5, 0):
        rec.clear()
        w("R %g" % ref)
        pump(2.5)
        seg = list(rec)
        ts, mp, ess, ise, umax = metr(seg, prev, ref)
        print(f"   r {prev:+.0f} -> {ref:+.0f} mm: ts={'no' if ts is None else '%.2f s' % ts}  Mp={mp:5.1f}%  ess={ess:+.3f} mm  ISE={ise:.3f}  |u_ef|max={umax:.1f}")
        for q in seg:
            allrows.append([mode] + q)
        prev = ref
w("S")
pump(0.3)
s.close()
with open(OUT, "w", newline="") as f:
    cw = csv.writer(f)
    cw.writerow(["modo_prueba", "t_ms", "r_mm", "y_mm", "e_mm", "u_ef", "u_pwm", "v_mm_s", "y_mod_mm", "modo"])
    cw.writerows(allrows)
print("guardado", len(allrows), "muestras")
