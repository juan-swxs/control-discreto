"""Pruebas de escalon para identificacion. Uso: escalones.py COM3 salida.csv"""
import serial, time, sys, csv

PORT, OUT = sys.argv[1], sys.argv[2]
s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.02
s.dtr = s.rts = False
s.open()
buf = b""
last = None
rec = None  # lista donde se graba


def pump(d):
    global buf, last
    e = time.time() + d
    while time.time() < e:
        buf += s.read(1024)
        while b"\n" in buf:
            l, buf = buf.split(b"\n", 1)
            p = l.decode(errors="replace").strip().split(",")
            if p[0] == "D" and len(p) >= 6:
                try:
                    r = (int(p[1]), int(p[2]), float(p[3]), float(p[4]), float(p[5]))
                except ValueError:
                    continue
                last = r
                if rec is not None:
                    rec.append(r)
            elif p[0] == "ERR":
                print("   <", ",".join(p))


def w(c):
    s.write((c + "\n").encode())


def recentrar():
    w("S")
    pump(1.2)
    for _ in range(25):
        if abs(last[2]) <= 0.15:
            break
        w("P %d 30" % (60 if last[2] > 0 else -60))
        pump(0.5)


pump(1.0)
w("LIM 30")
recentrar()
w("ZERO")
pump(0.3)

rows = []
amps = [60, 70, 80, 90, 100]
tid = 0
for a in amps:
    for sg in (1, -1):
        recentrar()
        tid += 1
        rec = []
        pump(0.3)
        w("U %d" % (sg * a))
        pump(1.5)
        w("S")
        pump(1.5)
        d, rec = rec, None
        x0 = d[0][2]
        xss = sum(r[2] for r in d if r[4] != 0 and r[0] - d[0][0] > 1300) / max(1, len([r for r in d if r[4] != 0 and r[0] - d[0][0] > 1300]))
        print(f"test {tid:2d}  U {sg*a:+4d}%  x0 {x0:+.3f}  x_ss {xss:+8.3f} mm  ({len(d)} muestras)")
        for r in d:
            rows.append((tid, sg * a) + r)

recentrar()
s.close()
with open(OUT, "w", newline="") as f:
    cw = csv.writer(f)
    cw.writerow(["test", "u_step", "t_ms", "counts", "x_mm", "v_mm_s", "u_pct"])
    cw.writerows(rows)
print("guardado", len(rows), "muestras en", OUT)
