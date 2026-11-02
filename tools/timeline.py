import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
q0 = [float(rows[5][k]) for k in 'abcd']
def conj(q): return [q[0], -q[1], -q[2], -q[3]]
def mul(a, b): return [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                       a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]]
last = -1
for x in rows:
    t = float(x['t'])
    if t - last < 0.5: continue
    last = t
    q = [float(x[k]) for k in 'abcd']
    r = mul(conj(q0), q)
    if r[0] < 0: r = [-v for v in r]
    ang = 2*math.degrees(math.acos(min(1, r[0]))); s = math.sqrt(max(1e-12, 1-r[0]**2))
    print(f"{t:5.1f} {x['phase']:6s} raw=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})  rel={ang:5.1f}deg axis=({r[1]/s:+.2f},{r[2]/s:+.2f},{r[3]/s:+.2f})")
