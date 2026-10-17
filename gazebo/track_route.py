"""Ordered centreline of a TrackBotGP circuit drawing (media/track_TRACK.png), and the route of a recorded race
on it: the robot's recorded distance and timing, lap by lap (referee.log), placed on the drawn line.

The drawing is thinned to a 1-pixel skeleton (Zhang-Suen), and walked from the finish line up the main straight;
at every junction the walk keeps the straightest continuation, which follows the main loop past the pit-lane
fork, the finish-marker stubs and any crossing. The pit lane is the longest piece of skeleton left over.

A race route is: lap 1 on the main loop; lap 2 on the main loop up to the pit fork, then into the pit lane up to
the stop; lap 3 from the stop out of the pit lane and once more round to the finish. Inside each stretch the
recorded wheel distance is scaled linearly onto the drawn length, so speed changes, line searches and the pit
stop keep their real timing while the robot stays on the line (the real axle cuts inside the bends).
"""
# References (full entries: docs/references.md at the repository root):
#   [R1] ISEP, TrackBotGP competition rules (2026 Robotics@ISEP Open): track figures
#   Zhang & Suen (1984), A fast parallel algorithm for thinning digital patterns
import csv
import math
import os
import re

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MM_PX = 2.0
PIT_SIDE = {"portimao": 1, "monza": 1, "suzuka": 1}   # pit lane right of the travel direction (track configs)
NB = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]


def thin(img):
    """Zhang-Suen thinning of a 0/1 image."""
    img = img.copy().astype(np.uint8)
    while True:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            n = [p[:-2, 1:-1], p[:-2, 2:], p[1:-1, 2:], p[2:, 2:], p[2:, 1:-1], p[2:, :-2], p[1:-1, :-2], p[:-2, :-2]]
            b = sum(n)
            a = sum(((n[i] == 0) & (n[(i + 1) % 8] == 1)) for i in range(8))
            if step == 0:
                c = (n[0] * n[2] * n[4] == 0) & (n[2] * n[4] * n[6] == 0)
            else:
                c = (n[0] * n[2] * n[6] == 0) & (n[0] * n[4] * n[6] == 0)
            rm = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & c
            if rm.any():
                img[rm] = 0
                changed = True
        if not changed:
            return img


def skeleton(track):
    g = cv2.imread(os.path.join(HERE, "media", f"track_{track}.png"), cv2.IMREAD_GRAYSCALE)
    m = (g < 128).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    m = (lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)   # the track only
    return thin(m), g.shape


def walk(sk, start, heading, stop_near=None, max_steps=200000):
    """Follow the skeleton from start (row, col) along heading (drow, dcol), keeping the straightest branch."""
    path, visited = [start], {start}
    d = np.array(heading, float)
    r, c = start
    for _ in range(max_steps):
        cands = [(r + dr, c + dc) for dr, dc in NB
                 if 0 <= r + dr < sk.shape[0] and 0 <= c + dc < sk.shape[1] and sk[r + dr, c + dc]
                 and (r + dr, c + dc) not in visited]
        if not cands:
            break
        # direction over the last few pixels, compared with a look-ahead along each candidate
        def score(p):
            v = np.array(p, float) - np.array(path[max(0, len(path) - 6)], float)
            return float(v @ d) / (np.linalg.norm(v) + 1e-9)
        nxt = max(cands, key=score)
        path.append(nxt)
        visited.add(nxt)
        if len(path) > 8:
            v = np.array(path[-1], float) - np.array(path[-9], float)
            d = 0.7 * d + 0.3 * v / (np.linalg.norm(v) + 1e-9)
            d /= np.linalg.norm(d)
        r, c = nxt
        if stop_near is not None and len(path) > 200 and abs(r - stop_near[0]) <= 2 and abs(c - stop_near[1]) <= 2:
            break
    return path


def smooth_resample(pts, step_m=0.005, win=9, closed=False):
    a = np.array(pts, float)
    if len(a) > win:
        k = np.ones(win) / win
        if closed:
            pad = np.vstack([a[-win:], a, a[:win]])
            a = np.column_stack([np.convolve(pad[:, i], k, "same")[win:-win] for i in (0, 1)])
        else:
            a = np.column_stack([np.convolve(np.pad(a[:, i], win // 2, mode="edge"), k, "valid") for i in (0, 1)])
    seg = np.hypot(*np.diff(a, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = int(s[-1] / step_m) + 1
    q = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(q, s, a[:, 0]), np.interp(q, s, a[:, 1])])


class Route:
    """A polyline in world metres with arc length; position and heading at any distance along it."""

    def __init__(self, xy):
        self.xy = np.asarray(xy, float)
        self.s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(self.xy, axis=0).T))])
        self.length = float(self.s[-1])

    def at(self, s):
        s = min(max(s, 0.0), self.length)
        x, y = np.interp(s, self.s, self.xy[:, 0]), np.interp(s, self.s, self.xy[:, 1])
        a = np.array([np.interp(max(s - 0.03, 0), self.s, self.xy[:, i]) for i in (0, 1)])
        b = np.array([np.interp(min(s + 0.03, self.length), self.s, self.xy[:, i]) for i in (0, 1)])
        return float(x), float(y), math.atan2(b[1] - a[1], b[0] - a[0])


def line_mask(track):
    g = cv2.imread(os.path.join(HERE, "media", f"track_{track}.png"), cv2.IMREAD_GRAYSCALE)
    m = (g < 128).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    return (lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)   # the track only


def follow(mask, p, d, stop=None, step=3.0, max_steps=20000, prefer=0.0, prefer_steps=0, until=None):
    """Drive along the line in the mask like a line follower: from p (col, row) in direction d, each step take the
    direction closest to straight ahead (or to prefer, in rad, at a fork) whose look-ahead stays on the line, then
    re-centre across the line. Stops near stop, or off the line."""
    h, w = mask.shape

    def on(q):
        c, r = int(round(q[0])), int(round(q[1]))
        return 0 <= r < h and 0 <= c < w and mask[r, c] > 0

    p, d = np.array(p, float), np.array(d, float) / np.linalg.norm(d)
    path = [p.copy()]
    for k in range(max_steps):
        best = None
        pref = prefer(k, p) if callable(prefer) else (prefer if k < prefer_steps else 0.0)
        for a in sorted(np.radians(np.arange(-75, 76, 5)), key=lambda a: abs(a - pref)):
            u = np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a)])
            if all(on(p + u * L) for L in (step, 2 * step, 4 * step)):
                best = u
                break
        if best is None:
            break
        q = p + best * step
        n = np.array([-best[1], best[0]])                       # re-centre across the line
        ts = [t for t in np.arange(-12, 12.5, 0.5) if on(q + n * t)]
        if ts:
            # the run of line pixels containing (or nearest to) q
            runs, cur = [], [ts[0]]
            for t in ts[1:]:
                if t - cur[-1] <= 0.6:
                    cur.append(t)
                else:
                    runs.append(cur)
                    cur = [t]
            runs.append(cur)
            run = min(runs, key=lambda r: min(abs(t) for t in r))
            if max(run) - min(run) < 14:                         # skip at junctions (the run is wider there)
                q = q + n * (min(run) + max(run)) / 2
        d = 0.6 * d + 0.4 * (q - p) / (np.linalg.norm(q - p) + 1e-9)
        d /= np.linalg.norm(d)
        p = q
        path.append(p.copy())
        if stop is not None and k > 30 and np.linalg.norm(p - np.array(stop, float)) < 2.5 * step:
            break
        if until is not None and until(k, p):
            break
    return path


def circuit(track, finish_px):
    """Main loop (from the finish line round to it) and pit lane (from the fork to the rejoin), world metres."""
    mask = line_mask(track)
    h_px = mask.shape[0]
    fx, fy = finish_px
    loop = follow(mask, (fx, fy - 6), (0, -1), stop=(fx, fy - 6))   # up the drawing first, keep straight
    loop_a = np.array(loop)
    # pit lane: drive the main straight from below the fork and take the branch on the pit side (TrackBotGP:
    # the side the pit lane leaves on; +1 = right of the direction of travel) until it meets the loop again
    side = PIT_SIDE.get(track, 1)
    # start on the main straight, at its lowest point within 0.2 m of the line's column (below the fork)
    straight = [k for k in range(len(loop) // 2, len(loop)) if abs(loop_a[k, 0] - fx) < 4]
    k0 = max(straight, key=lambda k: loop_a[k, 1])
    start_d = loop_a[min(k0 + 3, len(loop) - 1)] - loop_a[k0]
    left_loop = [False]

    def dist_loop(p):
        return float(np.min(np.hypot(loop_a[:, 0] - p[0], loop_a[:, 1] - p[1])))

    def back_on_loop(k, p):
        dmin = dist_loop(p)
        if dmin > 10:
            left_loop[0] = True
        return left_loop[0] and k > 40 and dmin < 3

    def prefer_branch(k, p):                                    # turn to the pit side until off the main line
        return 0.0 if left_loop[0] else side * 0.6

    lane = follow(mask, loop_a[k0], start_d, prefer=prefer_branch, until=back_on_loop)
    lane_a = np.array(lane)
    off = [k for k in range(len(lane_a)) if np.min(np.hypot(loop_a[:, 0] - lane_a[k, 0], loop_a[:, 1] - lane_a[k, 1])) > 4]
    a_k = max(off[0] - 1, 0)
    j_entry = int(np.argmin(np.hypot(loop_a[:, 0] - lane_a[a_k, 0], loop_a[:, 1] - lane_a[a_k, 1])))
    j_exit = int(np.argmin(np.hypot(loop_a[:len(loop) // 2, 0] - lane_a[-1, 0], loop_a[:len(loop) // 2, 1] - lane_a[-1, 1])))
    pit = [loop[j_entry]] + lane[a_k:] + [loop[j_exit]]

    def world(pts):
        return [(c * MM_PX / 1000, (h_px - r) * MM_PX / 1000) for c, r in pts]

    main = Route(smooth_resample(world(loop), closed=True))
    pit_r = Route(smooth_resample(world(pit)))
    frac = lambda j: float(np.interp(j, range(len(loop)), np.linspace(0, 1, len(loop))))   # noqa: E731
    return main, pit_r, frac(j_entry) * main.length, frac(j_exit) * main.length


def referee_events(run_dir):
    """Times (epoch s) of the race start, end of lap 1, pit stop, release and finish from referee.log."""
    ev = {}
    for line in open(os.path.join(run_dir, "referee.log"), encoding="utf-8", errors="replace"):
        m = re.search(r"\[(\d{10}\.\d+)\]", line)
        if not m:
            continue
        t = float(m.group(1))
        if "READY -> LAP1" in line:
            ev["start"] = t
        elif "LAP1 -> LAP2" in line:
            ev["lap1"] = t
        elif "LAP2 -> PIT" in line:
            ev["pit"] = t
        elif "PIT -> LAP3" in line:
            ev["release"] = t
        elif "LAP3 -> FINISH" in line:
            ev["finish"] = t
    return ev


def race_route(run_dir, track, finish_px, behind_m=0.15):
    """[(t from the start, x, y, yaw, lap)] at every recorded frame, on the drawn line."""
    main, pit, s_entry, s_exit = circuit(track, finish_px)
    ev = referee_events(run_dir)
    rec = os.path.join(run_dir, "rec.csv")
    rows = [r for r in csv.DictReader(open(rec)) if r.get("odom_v")]
    t = np.array([float(r["t"]) for r in rows])
    v = np.array([abs(float(r["odom_v"])) for r in rows])
    d = np.concatenate([[0], np.cumsum(v[1:] * np.diff(t))])      # wheel distance
    D = lambda tt: float(np.interp(tt, t, d))                     # noqa: E731
    stop_into_pit = 0.50                                          # "pit stop 0.50 m into the pit lane"
    L = main.length
    # drawn distance at each event, along the race route
    k0 = behind_m
    marks = [(ev["start"], 0.0), (ev["lap1"], k0 + L), (ev["pit"], k0 + L + s_entry + stop_into_pit),
             (ev["release"], k0 + L + s_entry + stop_into_pit),
             (ev["finish"], k0 + L + s_entry + pit.length + (L - s_exit))]

    def place(sd):
        """Race-route distance -> (x, y, yaw, lap)."""
        if sd < k0:                                               # behind the line, on the main straight
            x, y, a = main.at(0.0)
            return x - math.cos(a) * (k0 - sd), y - math.sin(a) * (k0 - sd), a, 1
        s = sd - k0
        if s <= L:
            return (*main.at(s), 1)
        s -= L
        if s <= s_entry:
            return (*main.at(s), 2)
        s -= s_entry
        if s <= pit.length:
            return (*pit.at(s), 2 if sd <= marks[2][1] else 3)
        s -= pit.length
        return (*main.at(s_exit + s), 3)

    out = []
    for ti in t:
        if ti < ev["start"]:
            sd = 0.0
        elif ti >= ev["finish"]:
            sd = marks[-1][1]
        else:
            for (ta, sa), (tb, sb) in zip(marks, marks[1:]):
                if ta <= ti <= tb:
                    da, db = D(ta), D(tb)
                    sd = sa if db - da < 1e-6 else sa + (sb - sa) * (D(ti) - da) / (db - da)
                    break
        out.append((ti - ev["start"], *place(sd)))
    return [o for o in out if -1.0 <= o[0]], main, pit
