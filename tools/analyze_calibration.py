import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
cues = [(float(x['t']), x['phase'][4:]) for x in rows if x['phase'].startswith('cue_')]
data = [(float(x['t']), [float(x[k]) for k in 'abcd']) for x in rows if not x['phase'].startswith('cue_')]
print('cues:', cues); print('samples:', len(data), 'span %.1f-%.1f' % (data[0][0], data[-1][0]))
def conj(q): return [q[0], -q[1], -q[2], -q[3]]
def mul(a, b): return [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                       a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]]
def rel(a, b):
    r = mul(conj(a), b)
    if r[0] < 0: r = [-v for v in r]
    ang = 2*math.degrees(math.acos(min(1, r[0]))); s = math.sqrt(max(1e-12, 1-r[0]**2))
    return ang, [r[1]/s, r[2]/s, r[3]/s]
def at(t): return min(data, key=lambda d: abs(d[0]-t))[1]
ref = at(4.0)
print('\ntimeline (relative to t=4s):')
for t in [x/2 for x in range(0, 80)]:
    if t > data[-1][0]: break
    a, ax = rel(ref, at(t)); mark = ''.join(f' <== {p}' for ct, p in cues if t <= ct < t+0.5)
    print(f'{t:5.1f} {a:6.1f} deg ({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f}){mark}')
print('\nper movement (peak vs the 1s before its cue):')
for mv in ('yaw', 'pitch', 'roll'):
    ct = dict((p, t) for t, p in cues)[mv]
    base = at(ct - 0.5)
    win = [d for d in data if ct <= d[0] <= ct + 7]
    peak = max(win, key=lambda d: rel(base, d[1])[0])
    a, ax = rel(base, peak[1])
    onset = next((d[0] for d in win if rel(base, d[1])[0] > 5), None)
    print(f'{mv:5s} peak {a:5.1f} deg at t={peak[0]:.1f} (cue {ct:.1f}, onset {onset}) axis=({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f})')
