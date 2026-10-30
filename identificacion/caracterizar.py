import serial, time, sys, csv

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
OUT = sys.argv[2] if len(sys.argv) > 2 else "caracterizacion.csv"

s = serial.Serial()
s.port = PORT
s.baudrate = 115200
s.timeout = 0.05
s.dtr = False
s.rts = False
s.open()

rows = []
buf = b""
t0 = time.time()


def pump(dur):
    """Lee telemetria durante dur segundos; devuelve lista de (t,counts,x,v,u)."""
    global buf
    out = []
    end = time.time() + dur
    while time.time() < end:
        buf += s.read(512)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.decode(errors="replace").strip()
            if line.startswith("D,"):
                p = line.split(",")
                try:
                    r = (int(p[1]), int(p[2]), float(p[3]), float(p[4]), float(p[5]))
                except (ValueError, IndexError):
                    continue
                out.append(r)
                rows.append(r)
            elif line.startswith(("ACK", "ERR")) or "lab5" in line:
                print("   <", line)
    return out


def cmd(c):
    s.write((c + "\n").encode())


pump(1.0)
s.reset_input_buffer()
buf = b""

# ---- 1. Calibracion del cero ----
print("== Calibracion del cero ==")
cmd("S")
cmd("LIM 30")
cmd("ZERO")
d = pump(2.0)
xs = [r[2] for r in d]
cs = [r[1] for r in d]
print(f"   reposo 2 s: x min={min(xs):.3f} max={max(xs):.3f} mm | cuentas min={min(cs)} max={max(cs)} | {len(d)} muestras")

# ---- 2. Pulsos ----
tests = [(100, 100), (-100, 100), (100, 200), (-100, 200), (100, 300), (-100, 300),
         (70, 300), (-70, 300)]
res = []
for u, ms in tests:
    cmd("ZERO")
    pump(0.3)
    cmd(f"P {u} {ms}")
    d = pump(ms / 1000 + 1.2)
    xs = [r[2] for r in d]
    vs = [r[3] for r in d]
    peak = max(xs, key=abs) if xs else float("nan")
    vpk = max(vs, key=abs) if vs else float("nan")
    final = xs[-1] if xs else float("nan")
    res.append((u, ms, peak, final, vpk))
    print(f"   P {u:+4d}% {ms:3d} ms -> pico {peak:+8.3f} mm | final {final:+8.3f} mm | v_max {vpk:+8.1f} mm/s")
    cmd("S")
    pump(0.5)

cmd("S")
pump(0.3)
s.close()

with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_ms", "counts", "x_mm", "v_mm_s", "u_pct"])
    w.writerows(rows)
print(f"\nGuardado {len(rows)} muestras en {OUT}")
