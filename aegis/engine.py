"""AEGIS simulator, nominal autonomy stack and predictive safety supervisor.

One ``Simulation`` instance owns the whole world state (robot, scripted movers,
static obstacles) and every controller decision. Both modes share:

* the same seeded ``Scenario`` (layout, unmapped pallets, mover scripts),
* the same acceleration-limited robot dynamics (|dv| <= A_MAX * DT per step),
* the same nominal stack: A* on the *prior map* (shelves only) + pure-pursuit tracking.

Only AEGIS mode adds the runtime layer: occlusion-aware perception with
optional Gaussian noise, least-squares velocity tracking, event-triggered A*
replanning around detected obstacles, and a predictive safety filter that
evaluates candidate commands by rolling out the real dynamics ("commit for
COMMIT seconds, then brake at A_MAX") against inflated obstacle forecasts.

This is an educational 2D kinematic experiment, not certified safety software.
"""
from __future__ import annotations

import random
from collections import deque
from math import hypot, sqrt

from .constants import (A_MAX, CLEARANCE, COMMIT, DT, EPISODE_LIMIT, FIELD_H, FIELD_W, GOAL_TOL,
                        GROWTH_CAP, GROWTH_RATE, HORIZON, NOISE_K, PLANNER_INFLATION, PLANNER_RES,
                        REACT_INFLUENCE, REACT_K, REACT_MOVING, REACT_RAY, REACT_RAY_STEP, REACT_REP_MAX,
                        REACT_STOP_BUFFER, REACT_T_REACT, ROBOT_R, SENSOR_RANGE, STATIC_CLEARANCE,
                        TRACK_COAST, UNKNOWN_VEL_GROWTH, V_MAX)
from .geometry import bounds_gap, circle_rect_gap, point_rect_distance, segment_intersects_rect
from .planner import GridPlanner, path_length
from .scenarios import SCENARIO_TYPES, make_scenario

H_STEPS = int(round(HORIZON / DT))
COMMIT_STEPS = int(round(COMMIT / DT))
LOOKAHEAD = 0.7
NEAR_MISS = 0.15
CANDIDATE_ORDER = ('PROCEED', 'SLOW_60', 'SLOW_30', 'STOP', 'EVADE_LEFT', 'EVADE_RIGHT', 'RETREAT')
FALLBACK_SET = ('STOP', 'EVADE_LEFT', 'EVADE_RIGHT', 'RETREAT')
# After holding for RECOVERY_TICKS, prefer safe moves that create space over waiting forever.
RECOVERY_ORDER = ('PROCEED', 'SLOW_60', 'SLOW_30', 'RETREAT', 'EVADE_LEFT', 'EVADE_RIGHT', 'STOP')
RECOVERY_TICKS = 30
VEL_TRUST = 0.5                       # max velocity standard error [m/s] before a velocity is used
MAX_SPEED = {'pedestrian': 2.5, 'forklift': 3.5}   # plausibility cap on estimated speed [m/s]


def accel_limit(vx: float, vy: float, cx: float, cy: float):
    """Move velocity toward the command by at most A_MAX*DT, then cap speed at V_MAX."""
    dvx, dvy = cx - vx, cy - vy
    d = hypot(dvx, dvy)
    lim = A_MAX * DT
    if d > lim:
        dvx, dvy = dvx * lim / d, dvy * lim / d
    nvx, nvy = vx + dvx, vy + dvy
    s = hypot(nvx, nvy)
    if s > V_MAX:
        nvx, nvy = nvx * V_MAX / s, nvy * V_MAX / s
    return nvx, nvy


class PathTracker:
    """Pure-pursuit follower on a polyline with goal-approach speed limiting."""

    def __init__(self, path):
        self.set_path(path)

    def set_path(self, path):
        self.path = path
        self.seg = 0

    def command(self, x, y, goal):
        p = self.path
        if not p or len(p) < 2:
            return 0.0, 0.0, None
        best, best_i, proj = float('inf'), self.seg, p[self.seg]
        for i in range(self.seg, len(p) - 1):
            ax, ay = p[i]
            bx, by = p[i + 1]
            sx, sy = bx - ax, by - ay
            ll = sx * sx + sy * sy
            u = 0.0 if ll < 1e-12 else max(0.0, min(1.0, ((x - ax) * sx + (y - ay) * sy) / ll))
            qx, qy = ax + sx * u, ay + sy * u
            d = hypot(x - qx, y - qy)
            if d < best - 1e-9:
                best, best_i, proj = d, i, (qx, qy)
        self.seg = best_i
        rem, (px, py), i = LOOKAHEAD, proj, best_i
        target = p[-1]
        while i < len(p) - 1:
            bx, by = p[i + 1]
            d = hypot(bx - px, by - py)
            if d >= rem:
                target = (px + (bx - px) * rem / max(d, 1e-9), py + (by - py) * rem / max(d, 1e-9))
                break
            rem -= d
            px, py = bx, by
            i += 1
        dx, dy = target[0] - x, target[1] - y
        dn = hypot(dx, dy)
        if dn < 1e-9:
            return 0.0, 0.0, None
        ux, uy = dx / dn, dy / dn
        speed = min(V_MAX, sqrt(2 * A_MAX * 0.7 * hypot(goal[0] - x, goal[1] - y)))
        return speed * ux, speed * uy, (ux, uy)


class Track:
    """Least-squares constant-velocity track built from (possibly noisy) observations."""

    def __init__(self, mid, kind, radius, window):
        self.id, self.kind, self.radius = mid, kind, radius
        self.obs = deque(maxlen=window)
        self.last_seen = -1.0
        self.first_seen = None

    def estimate(self, t, sigma):
        n = len(self.obs)
        if n == 1:
            _, x, y = self.obs[0]
            return x, y, 0.0, 0.0, sigma, 0.0, True
        tb = sum(o[0] for o in self.obs) / n
        xb = sum(o[1] for o in self.obs) / n
        yb = sum(o[2] for o in self.obs) / n
        s = sum((o[0] - tb) ** 2 for o in self.obs)
        vx = sum((o[0] - tb) * (o[1] - xb) for o in self.obs) / s
        vy = sum((o[0] - tb) * (o[2] - yb) for o in self.obs) / s
        sp = sigma * sqrt(1.0 / n + (t - tb) ** 2 / s)
        sv = sigma / sqrt(s)
        if sv > VEL_TRUST:
            # Too few noisy samples for a usable velocity: forecast the latest averaged
            # position with the (larger) unknown-velocity reachable-set growth instead.
            return xb, yb, 0.0, 0.0, sigma / sqrt(n), 0.0, True
        sp_cap = MAX_SPEED.get(self.kind, 3.5)
        spd = hypot(vx, vy)
        if spd > sp_cap:
            vx, vy = vx * sp_cap / spd, vy * sp_cap / spd
        return xb + vx * (t - tb), yb + vy * (t - tb), vx, vy, sp, sv, False


CONTROLLERS = ('basic', 'reactive', 'aegis')


def resolve_controller(aegis=True, controller=None):
    """Backwards-compatible controller selection: ``controller`` wins, else ``aegis`` bool."""
    if controller is None:
        return 'aegis' if aegis else 'basic'
    if controller not in CONTROLLERS:
        raise ValueError(f'unknown controller {controller!r}; expected one of {", ".join(CONTROLLERS)}')
    return controller


class Simulation:
    def __init__(self, scenario='crossing', aegis=True, seed=7, record=True, controller=None):
        self.controller = resolve_controller(aegis, controller)
        self.sc = make_scenario(scenario, seed)   # raises ValueError for unknown types
        self.scenario, self.seed, self.record = scenario, int(seed), record
        self.aegis = self.controller == 'aegis'
        # Reactive and AEGIS share the identical perception / tracking / replanning layer.
        self.perceives = self.controller in ('reactive', 'aegis')
        sc = self.sc
        self.x, self.y = sc.start
        self.vx = self.vy = 0.0
        self.goal = sc.goal
        self.t = 0.0
        self.steps = 0
        self.planner = GridPlanner(FIELD_W, FIELD_H, PLANNER_RES, PLANNER_INFLATION)
        # Both modes start from the identical prior-map plan (shelves only).
        self.plan = self.planner.plan(sc.start, sc.goal, sc.shelves, [])
        self.tracker = PathTracker(self.plan)
        self.trail = [(round(self.x, 3), round(self.y, 3))]
        self.events = []
        self.collision = None
        self.completed = False
        self.no_path = self.plan is None
        self.interventions = 0
        self.fallback_ticks = 0
        self.replans = 0
        self.near_misses = 0
        self.min_dyn = float('inf')
        self.min_static = float('inf')
        self.distance = 0.0
        self.max_dv = 0.0
        self.action = 'PROCEED'
        self.decision = None
        self.last_fallback_t = None
        # AEGIS-only runtime state.
        self.noise_rng = random.Random(f'noise:{scenario}:{seed}')
        self.window = 4 if sc.sensor_sigma == 0 else 10
        self.tracks = {}
        self.known_statics = set()
        self.temp_blocks = {}
        self.hold_ticks = 0
        # Stop metrics (all controllers): a "stop" is moving (>=0.05 m/s) -> stationary after start.
        self.stop_events = 0
        self.stopped_ticks = 0
        self._was_moving = False
        mode = {'aegis': 'AEGIS', 'basic': 'baseline (no supervisor)',
                'reactive': 'reactive baseline (same sensing; braking + local avoidance, no forecasting)'}
        self._log('START', f'{sc.description}. Mode: {mode[self.controller]}.')
        if self.no_path:
            self._log('NO_PATH', 'Prior map has no route to the goal.')

    # ------------------------------------------------------------------ helpers
    def _log(self, kind, msg):
        self.events.append({'t': round(self.t, 2), 'type': kind, 'message': msg})

    def _visible(self, ox, oy):
        if hypot(ox - self.x, oy - self.y) > SENSOR_RANGE:
            return False
        return not any(segment_intersects_rect(self.x, self.y, ox, oy, r) for r in self.sc.shelves)

    def _known_circles(self):
        circles = [self.sc.statics[i] for i in sorted(self.known_statics)]
        return circles + list(self.temp_blocks.values())

    def _replan(self, reason):
        new = self.planner.plan((self.x, self.y), self.goal, self.sc.shelves, self._known_circles())
        self.replans += 1
        if new is None:
            if not self.no_path:
                self._log('NO_PATH', f'{reason}; no collision-free route exists. Holding position.')
            self.no_path = True
            self.plan = None
        else:
            self.no_path = False
            self.plan = new
            self._log('REPLAN', f'{reason}; A* route {path_length(new):.1f} m via {len(new) - 2} turn point(s).')
        self.tracker.set_path(self.plan)

    # ------------------------------------------------------------------ perception (AEGIS only)
    def _perceive(self):
        sigma = self.sc.sensor_sigma
        for m in self.sc.movers:
            if not m.active(self.t):
                continue
            mx, my = m.pos(self.t)
            if not self._visible(mx, my):
                continue
            tr = self.tracks.get(m.id)
            if tr is None:
                tr = self.tracks[m.id] = Track(m.id, m.kind, m.radius, self.window)
                tr.first_seen = self.t
                gap = hypot(mx - self.x, my - self.y) - ROBOT_R - m.radius
                self._log('DETECT', f'{m.kind.title()} #{m.id} first observed {gap:.2f} m away.')
            if sigma:
                mx += self.noise_rng.gauss(0, sigma)
                my += self.noise_rng.gauss(0, sigma)
            tr.obs.append((self.t, mx, my))
            tr.last_seen = self.t
        for mid in [k for k, tr in self.tracks.items() if self.t - tr.last_seen > TRACK_COAST]:
            del self.tracks[mid]
            if mid in self.temp_blocks:
                del self.temp_blocks[mid]
                self._replan(f'Track #{mid} lost; removing its temporary blockage')
        new = []
        for i, (sx, sy, sr) in enumerate(self.sc.statics):
            if i not in self.known_statics and self._visible(sx, sy):
                self.known_statics.add(i)
                new.append(i)
        if new:
            self._log('DETECT', f'Unmapped obstacle(s) {new} detected.')
            self._replan('New static obstacle')

    def _forecasts(self, at_last_seen=False):
        """Track estimates. AEGIS evaluates them now (coasting extrapolation); the reactive
        controller passes ``at_last_seen=True`` so it only uses the fit at the latest
        observation time - no extrapolation into the future."""
        sigma = self.sc.sensor_sigma
        out = []
        for tr in self.tracks.values():
            x, y, vx, vy, sp, sv, unknown = tr.estimate(tr.last_seen if at_last_seen else self.t, sigma)
            out.append({'id': tr.id, 'kind': tr.kind, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'r': tr.radius,
                        'sp': sp, 'sv': sv, 'unknown': unknown, 'age': self.t - tr.last_seen})
        return out

    @staticmethod
    def _growth(f, t):
        te = t + f['age']
        g = min(GROWTH_RATE * te, GROWTH_CAP) + NOISE_K * (f['sp'] + f['sv'] * t)
        if f['unknown']:
            g += UNKNOWN_VEL_GROWTH * te
        return g

    # ------------------------------------------------------------------ supervisor
    def _rollout(self, cx, cy, forecasts, statics, shelves, keep=False):
        x, y, vx, vy = self.x, self.y, self.vx, self.vy
        worst, worst_t, culprit = float('inf'), 0.0, None
        pts = []
        for k in range(1, H_STEPS + 1):
            t = k * DT
            if k <= COMMIT_STEPS:
                vx, vy = accel_limit(vx, vy, cx, cy)
            else:
                vx, vy = accel_limit(vx, vy, 0.0, 0.0)
            x += vx * DT
            y += vy * DT
            if keep:
                pts.append((x, y))
            for f in forecasts:
                s = hypot(x - f['x'] - f['vx'] * t, y - f['y'] - f['vy'] * t) - ROBOT_R - f['r'] \
                    - CLEARANCE - self._growth(f, t)
                if s < worst:
                    worst, worst_t, culprit = s, t, f"{f['kind']} #{f['id']}"
            for (sx, sy, sr) in statics:
                s = hypot(x - sx, y - sy) - ROBOT_R - sr - STATIC_CLEARANCE
                if s < worst:
                    worst, worst_t, culprit = s, t, 'static obstacle'
            for rect in shelves:
                s = circle_rect_gap(x, y, ROBOT_R, rect) - STATIC_CLEARANCE
                if s < worst:
                    worst, worst_t, culprit = s, t, 'shelf'
            s = bounds_gap(x, y, ROBOT_R, FIELD_W, FIELD_H) - STATIC_CLEARANCE
            if s < worst:
                worst, worst_t, culprit = s, t, 'wall'
        return worst, worst_t, culprit, pts

    def _candidates(self, nx, ny, direction):
        if direction is None:
            sp = hypot(self.vx, self.vy)
            direction = (self.vx / sp, self.vy / sp) if sp > 1e-6 else (1.0, 0.0)
        ux, uy = direction
        return {
            'PROCEED': (nx, ny), 'SLOW_60': (0.6 * nx, 0.6 * ny), 'SLOW_30': (0.3 * nx, 0.3 * ny),
            'STOP': (0.0, 0.0), 'EVADE_LEFT': (-uy * V_MAX, ux * V_MAX),
            'EVADE_RIGHT': (uy * V_MAX, -ux * V_MAX), 'RETREAT': (-ux * V_MAX, -uy * V_MAX),
        }

    def _supervise(self, nx, ny, direction):
        forecasts = self._forecasts()
        statics = self._known_circles()[:len(self.known_statics)]  # temp blocks are tracked dynamically
        shelves = [r for r in self.sc.shelves if point_rect_distance(self.x, self.y, r) < 2.5]
        cands = self._candidates(nx, ny, direction)
        evaluated, chosen = [], None
        order = RECOVERY_ORDER if self.hold_ticks >= RECOVERY_TICKS and not self.no_path else CANDIDATE_ORDER
        for name in order:
            slack, t_min, culprit, _ = self._rollout(*cands[name], forecasts, statics, shelves)
            evaluated.append({'name': name, 'slack': round(slack, 3), 't': round(t_min, 1),
                              'culprit': culprit, 'safe': slack >= 0})
            if slack >= 0:
                chosen = name
                break   # candidates are in preference order; stop at the first safe one
        fallback = chosen is None
        if fallback:
            # Conservative best effort: never keep driving at (reduced) nominal speed when no
            # command is verified safe. Choose among braking/evasive manoeuvres the one with the
            # largest worst-case forecast clearance; ties favour braking.
            best = max((e for e in evaluated if e['name'] in FALLBACK_SET),
                       key=lambda e: (e['slack'], e['name'] == 'STOP'))
            chosen = best['name']
        pick = next(e for e in evaluated if e['name'] == chosen)
        nom = evaluated[0]
        if fallback:
            why = (f'No candidate meets the margin. Best effort {chosen}: short by {-pick["slack"]:.2f} m vs '
                   f'{pick["culprit"]} at +{pick["t"]:.1f} s. Collision may be unavoidable.')
        elif chosen == 'PROCEED':
            why = (f'Nominal command keeps {nom["slack"]:.2f} m spare clearance over the {HORIZON:.1f} s forecast '
                   f'(hold {COMMIT:.1f} s, then max braking).')
        else:
            why = (f'PROCEED short by {-nom["slack"]:.2f} m vs {nom["culprit"]} at +{nom["t"]:.1f} s; '
                   f'{chosen} keeps {pick["slack"]:.2f} m spare.')
        if self.no_path:
            why = ('No collision-free route to the goal is known: holding position, mission cannot be '
                   'completed safely. ' + why)
        cx, cy = cands[chosen]
        pts = []
        if self.record:
            _, _, _, pts = self._rollout(cx, cy, forecasts, statics, shelves, keep=True)
        self.decision = {'action': chosen, 'fallback': fallback, 'rationale': why, 'candidates': evaluated,
                         'nominal_slack': nom['slack'],
                         'forecast_robot': [[round(px, 2), round(py, 2)] for px, py in pts[1::2]],
                         'forecast_obstacles': [
                             {'id': f['id'], 'kind': f['kind'], 'unknown': f['unknown'],
                              'est': [round(f['x'], 2), round(f['y'], 2)],
                              'disks': [[round(f['x'] + f['vx'] * t, 2), round(f['y'] + f['vy'] * t, 2),
                                         round(f['r'] + CLEARANCE + self._growth(f, t), 2)]
                                        for t in (0.0, 0.5, 1.0, 1.5, 2.0)]}
                             for f in forecasts]}
        return chosen, fallback, cx, cy, forecasts

    # ------------------------------------------------------------------ reactive baseline
    def _react(self, nx, ny, direction):
        """Competent reactive controller using only CURRENT observations.

        Same perception, tracks, noise, static discovery and replanning as AEGIS, but:
        * obstacle positions are the track fit at the latest observation (no extrapolation);
        * no multi-step obstacle forecast and no candidate rollouts.
        Behaviour: (1) repulsive local avoidance with a lateral bias for obstacles ahead;
        (2) braking-distance speed limit from a ray-cast along the commanded heading,
        v <= -A*T + sqrt((A*T)^2 + 2*A*d_free) (one control period reaction time);
        (3) speed-and-separation cap near people; (4) emergency brake when a person is
        inside the margin and ahead of the robot.
        """
        sigma = self.sc.sensor_sigma
        obs = self._forecasts(at_last_seen=True)
        margin = CLEARANCE + NOISE_K * sigma
        statics = [self.sc.statics[i] for i in sorted(self.known_statics)]
        shelves = [r for r in self.sc.shelves if point_rect_distance(self.x, self.y, r) < REACT_RAY + 1.0]
        speed_nom = hypot(nx, ny)
        if direction is None:
            sp = hypot(self.vx, self.vy)
            direction = (self.vx / sp, self.vy / sp) if sp > 1e-6 else (1.0, 0.0)
        ux, uy = direction
        if self.no_path or speed_nom < 1e-9:
            why = 'No collision-free route known: holding position.' if self.no_path else 'Nominal command is zero.'
            self.decision = {'action': 'STOP' if self.no_path else 'PROCEED', 'rationale': why}
            return self.decision['action'], 0.0, 0.0, obs

        # (1) local avoidance from current positions of tracked people / vehicles
        rx = ry = 0.0
        gaps = []
        for f in obs:
            dx, dy = self.x - f['x'], self.y - f['y']
            dist = max(hypot(dx, dy), 1e-6)
            gap = dist - ROBOT_R - f['r']
            gaps.append((gap, f, dx, dy, dist))
            if gap >= REACT_INFLUENCE:
                continue
            w = REACT_K * (1.0 / max(gap, 0.05) - 1.0 / REACT_INFLUENCE)
            rx += w * dx / dist
            ry += w * dy / dist
            ovx, ovy = (0.0, 0.0) if f['unknown'] else (f['vx'], f['vy'])
            osp = hypot(ovx, ovy)
            if osp > REACT_MOVING:
                # Moving person: step sideways off their CURRENT line of motion (no forecast).
                qx, qy = ovx / osp, ovy / osp
                side = 1.0 if (qx * dy - qy * dx) > 0 else -1.0
                rx += w * -qy * side
                ry += w * qx * side
            elif -(dx * ux + dy * uy) > 0:                  # static/slow obstacle ahead: sidestep
                left_of_path = (ux * (-dy) - uy * (-dx)) > 0
                px, py = (uy, -ux) if left_of_path else (-uy, ux)
                rx += w * px
                ry += w * py
        rm = hypot(rx, ry)
        if rm > REACT_REP_MAX:
            rx, ry = rx * REACT_REP_MAX / rm, ry * REACT_REP_MAX / rm
        hx, hy = ux + rx, uy + ry
        hm = hypot(hx, hy)
        dx_, dy_ = (hx / hm, hy / hm) if hm > 1e-6 else (ux, uy)

        # (2) braking-distance limit from a ray-cast along the chosen heading
        def gap_at(px, py):
            out = []
            for f in obs:
                out.append(hypot(px - f['x'], py - f['y']) - ROBOT_R - f['r'] - margin)
            for sx, sy, sr in statics:
                out.append(hypot(px - sx, py - sy) - ROBOT_R - sr - STATIC_CLEARANCE)
            for rect in shelves:
                out.append(circle_rect_gap(px, py, ROBOT_R, rect) - STATIC_CLEARANCE)
            out.append(bounds_gap(px, py, ROBOT_R, FIELD_W, FIELD_H) - STATIC_CLEARANCE)
            return out

        g0 = gap_at(self.x, self.y)
        n = int(round(REACT_RAY / REACT_RAY_STEP))

        def ray(hx_, hy_):
            for k in range(1, n + 1):
                s = k * REACT_RAY_STEP
                gk = gap_at(self.x + hx_ * s, self.y + hy_ * s)
                # Blocked only where clearance is violated AND moving makes it worse.
                hit = next((i for i, (a, b) in enumerate(zip(gk, g0)) if a < 0 and a < b - 1e-9), None)
                if hit is not None:
                    return s - REACT_RAY_STEP, hit
            return REACT_RAY, None

        aT = A_MAX * DT

        def brake_speed(d):
            return -aT + sqrt(aT * aT + 2 * A_MAX * max(0.0, d - REACT_STOP_BUFFER))

        d_free, blocker = ray(dx_, dy_)
        v_brake = brake_speed(d_free)
        # The robot cannot turn instantly (|dv| <= A_MAX*DT), so it must also be able to stop
        # along its CURRENT direction of motion; otherwise brake hard.
        cur = hypot(self.vx, self.vy)
        overspeed = False
        if cur > 0.05:
            d_cur, _ = ray(self.vx / cur, self.vy / cur)
            overspeed = cur > brake_speed(d_cur) + 0.05

        # (3) speed-and-separation monitoring (ISO/TS 15066-style, current state only):
        #     S >= v_h*(T + T_s) + c*v*T + c*v^2/(2a), with T_s = v/a the robot stopping time,
        #     c = cos(heading, person) and v_h the person's CURRENT closing speed.
        #     (4) emergency brake when a person is inside the margin ahead.
        v_ssm, estop, near = float('inf'), False, None
        for gap, f, dx, dy, dist in gaps:
            clear = gap - margin
            ahead = -(dx * dx_ + dy * dy_) / dist          # cos(angle) between heading and person
            if clear < 0 and ahead > 0:
                estop, near = True, f
            if ahead <= 1e-3:
                continue
            ovx, ovy = (0.0, 0.0) if f['unknown'] else (f['vx'], f['vy'])
            v_h = max(0.0, (ovx * dx + ovy * dy) / dist)    # person's current speed toward robot
            room = clear - v_h * REACT_T_REACT
            if room <= 0:
                lim = 0.0
            else:
                # ISO/TS 15066 SSM with stopping time T_s = v/a, robot speed component c*v toward
                # the person:  room >= v*(v_h/a + c*T) + c*v^2/(2a)  ->  solve for v.
                qa, qb = ahead / (2 * A_MAX), v_h / A_MAX + ahead * REACT_T_REACT
                lim = (-qb + sqrt(qb * qb + 4 * qa * room)) / (2 * qa)
            if lim < v_ssm:
                v_ssm, near = lim, f
        cap = min(1.0, v_ssm / V_MAX)
        v = min(speed_nom, v_brake, cap * V_MAX)
        turned = dx_ * ux + dy_ * uy < 0.985          # > ~10 degrees off the nominal heading
        if estop or overspeed:
            action, v = 'EMERGENCY_STOP', 0.0
            why = (f'{near["kind"]} #{near["id"]} inside the {margin:.2f} m margin ahead: emergency brake.'
                   if estop else 'Cannot stop within the free distance along current motion: emergency brake.')
        elif v < speed_nom - 0.05 or turned:
            action = 'AVOID' if turned else 'SLOW'
            lim = 'braking distance' if v_brake <= cap * V_MAX else 'separation cap'
            what = 'obstacle' if blocker is None else ('person/vehicle' if blocker < len(obs) else 'static obstacle')
            why = (f'{action}: free distance {d_free:.2f} m along heading (limit {what}); '
                   f'speed {v:.2f} m/s limited by {lim}.')
        else:
            action = 'PROCEED'
            why = f'Path clear for {d_free:.2f} m; nominal speed.'
        self.decision = {'action': action, 'rationale': why}
        return action, v * dx_, v * dy_, obs

    def _check_loiter(self, forecasts):
        """If held for 1.5 s by a near-stationary person, plan around them as a temporary obstacle."""
        if self.hold_ticks < 15:
            return
        for f in forecasts:
            if f['id'] in self.temp_blocks or f['unknown']:
                continue
            if hypot(f['vx'], f['vy']) < 0.15 and hypot(f['x'] - self.x, f['y'] - self.y) < 3.0:
                # Pad so the planned pass keeps more clearance than the supervisor's own
                # worst-case forecast margin for a stationary person (CLEARANCE + GROWTH_CAP).
                pad = CLEARANCE + GROWTH_CAP + NOISE_K * f['sp'] - (PLANNER_INFLATION - ROBOT_R) + 0.1
                self.temp_blocks[f['id']] = (f['x'], f['y'], f['r'] + max(pad, 0.25))
                self.hold_ticks = 0
                self._replan(f'{f["kind"].title()} #{f["id"]} stationary in lane')
                return
        if self.hold_ticks == 15 and (self.temp_blocks or self.known_statics):
            # Still held although every blocker is already on the map: the current plan was
            # made from a worse position (e.g. inside an inflated zone). Replan from here.
            self._replan('Held 1.5 s by mapped blockers; replanning from current position')
        for mid in list(self.temp_blocks):
            f = next((g for g in forecasts if g['id'] == mid), None)
            if f is not None and hypot(f['vx'], f['vy']) > 0.4:
                del self.temp_blocks[mid]
                self._replan(f'Track #{mid} moving again; removing temporary blockage')

    # ------------------------------------------------------------------ main loop
    def finished(self):
        return self.completed or self.collision is not None or self.t >= EPISODE_LIMIT - 1e-9

    def tick(self):
        if self.finished():
            return self.state()
        self.steps += 1
        if self.perceives:
            self._perceive()
        nx, ny, direction = self.tracker.command(self.x, self.y, self.goal)
        fallback = False
        if self.aegis:
            action, fallback, cx, cy, forecasts = self._supervise(nx, ny, direction)
        elif self.controller == 'reactive':
            action, cx, cy, forecasts = self._react(nx, ny, direction)
        else:
            action, cx, cy = 'PROCEED', nx, ny
        if action != 'PROCEED':
            self.interventions += 1
        if fallback:
            self.fallback_ticks += 1
            self.last_fallback_t = self.t
        if action != self.action or (fallback and self.events[-1]['type'] != 'FALLBACK'):
            if self.perceives:
                self._log('FALLBACK' if fallback else action, self.decision['rationale'])
        self.action = action

        ovx, ovy = self.vx, self.vy
        self.vx, self.vy = accel_limit(self.vx, self.vy, cx, cy)
        self.max_dv = max(self.max_dv, hypot(self.vx - ovx, self.vy - ovy))
        x0, y0 = self.x, self.y
        self.x += self.vx * DT
        self.y += self.vy * DT
        self.distance += hypot(self.x - x0, self.y - y0)
        self._physics(x0, y0)
        self.t = round(self.t + DT, 4)
        spd = hypot(self.vx, self.vy)
        if spd >= 0.05:
            self._was_moving = True
        elif self.collision is None and not self.completed:
            self.stopped_ticks += 1 if self._was_moving else 0
            if self._was_moving and hypot(ovx, ovy) >= 0.05:
                self.stop_events += 1
        if self.perceives:
            moving = hypot(self.vx, self.vy) > 0.2
            self.hold_ticks = 0 if (action == 'PROCEED' or moving) else self.hold_ticks + 1
            self._check_loiter(forecasts)
        self.trail.append((round(self.x, 3), round(self.y, 3)))
        if self.collision is None and hypot(self.x - self.goal[0], self.y - self.goal[1]) < GOAL_TOL:
            self.completed = True
            self._log('GOAL', f'Goal reached in {self.t:.1f} s.')
        if self.collision is None and not self.completed and self.t >= EPISODE_LIMIT - 1e-9:
            self._log('TIMEOUT', f'{EPISODE_LIMIT:.0f} s episode limit reached without completing the mission.')
        return self.state()

    def _physics(self, x0, y0):
        """Ground-truth contact check with 4 sub-steps to avoid tunnelling."""
        sc = self.sc
        for k in range(1, 5):
            a = k / 4
            px, py = x0 + (self.x - x0) * a, y0 + (self.y - y0) * a
            ts = self.t + DT * a
            for m in sc.movers:
                if not m.active(ts):
                    continue
                mx, my = m.pos(ts)
                gap = hypot(px - mx, py - my) - ROBOT_R - m.radius
                if gap < self.min_dyn:
                    self.min_dyn = gap
                if gap < 0:
                    mvx, mvy = m.vel(ts)
                    return self._collide(f'{m.kind} #{m.id}', hypot(self.vx - mvx, self.vy - mvy))
            for i, (sx, sy, sr) in enumerate(sc.statics):
                gap = hypot(px - sx, py - sy) - ROBOT_R - sr
                if gap < self.min_static:
                    self.min_static = gap
                if gap < 0:
                    return self._collide(f'unmapped pallet #{i}', hypot(self.vx, self.vy))
            for rect in sc.shelves:
                if circle_rect_gap(px, py, ROBOT_R, rect) < 0:
                    return self._collide('shelf', hypot(self.vx, self.vy))
            if bounds_gap(px, py, ROBOT_R, FIELD_W, FIELD_H) < 0:
                return self._collide('wall', hypot(self.vx, self.vy))
        # Near-miss counted once per tick using end-of-step separation.
        for m in sc.movers:
            if m.active(self.t + DT):
                mx, my = m.pos(self.t + DT)
                if 0 <= hypot(self.x - mx, self.y - my) - ROBOT_R - m.radius < NEAR_MISS:
                    self.near_misses += 1
                    break

    def _collide(self, what, rel_speed):
        speed = hypot(self.vx, self.vy)
        t = round(self.t + DT, 2)
        warned = self.aegis and self.last_fallback_t is not None and t - self.last_fallback_t <= 1.5
        self.collision = {'t': t, 'with': what, 'robot_speed': round(speed, 2),
                          'relative_speed': round(rel_speed, 2), 'robot_stationary': speed < 0.05,
                          'supervisor_declared_no_safe_action': bool(warned)}
        note = 'robot was stationary' if speed < 0.05 else f'robot moving at {speed:.2f} m/s'
        if self.aegis:
            note += '; supervisor had declared no safe action' if warned else '; NOT anticipated by supervisor'
        self._log('COLLISION', f'Contact with {what} ({note}, relative speed {rel_speed:.2f} m/s).')

    # ------------------------------------------------------------------ reporting
    def outcome(self):
        if self.collision:
            return 'collision'
        if self.completed:
            return 'completed'
        if self.finished():
            return 'blocked_safe_stop' if self.no_path else 'timeout'
        return 'running'

    def state(self):
        s = {
            't': round(self.t, 2), 'robot': [round(self.x, 3), round(self.y, 3)],
            'velocity': [round(self.vx, 3), round(self.vy, 3)], 'speed': round(hypot(self.vx, self.vy), 3),
            'movers': [{'id': m.id, 'kind': m.kind, 'r': m.radius, 'pos': [round(v, 3) for v in m.pos(self.t)]}
                       for m in self.sc.movers if m.active(self.t)],
            'action': self.action, 'interventions': self.interventions, 'outcome': self.outcome(),
            'collision': self.collision, 'completed': self.completed, 'finished': self.finished(),
            'event_count': len(self.events),
            'min_clearance_dynamic': None if self.min_dyn == float('inf') else round(self.min_dyn, 3),
        }
        if self.record:
            s['plan'] = [list(p) for p in self.plan] if self.plan else []
        if self.perceives:
            s['known_statics'] = sorted(self.known_statics)
            s['temp_blocks'] = [[round(v, 2) for v in c] for c in self.temp_blocks.values()]
            if self.decision and self.record:
                # Reactive decisions only carry 'action' and 'rationale' (no forecasts).
                s['decision'] = self.decision
        return s

    def summary(self):
        return {
            'scenario': self.scenario, 'seed': self.seed, 'aegis': self.aegis, 'outcome': self.outcome(),
            'collision': self.collision is not None, 'collision_info': self.collision,
            'completed': self.completed, 'elapsed': round(self.t, 2),
            'interventions': self.interventions, 'fallback_ticks': self.fallback_ticks,
            'replans': self.replans, 'near_misses': self.near_misses,
            'min_clearance_dynamic': None if self.min_dyn == float('inf') else round(self.min_dyn, 3),
            'min_clearance_static': None if self.min_static == float('inf') else round(self.min_static, 3),
            'distance': round(self.distance, 2), 'max_velocity_change': round(self.max_dv, 4),
        }

    def summary3(self):
        """Summary plus three-way-experiment fields (kept separate so v0.2 summaries are unchanged)."""
        s = self.summary()
        s.update(controller=self.controller, stop_events=self.stop_events,
                 stopped_time=round(self.stopped_ticks * DT, 2))
        return s


# ---------------------------------------------------------------------- runners
def run_episode(scenario='crossing', aegis=True, seed=7, controller=None):
    s = Simulation(scenario, aegis, seed, record=False, controller=controller)
    while not s.finished():
        s.tick()
    return s.summary() if controller is None else s.summary3()


def replay(scenario='crossing', aegis=True, seed=7, controller=None):
    s = Simulation(scenario, aegis, seed, record=True, controller=controller)
    states = [s.state()]
    while not s.finished():
        states.append(s.tick())
    return {'states': states, 'events': s.events, 'final': s.summary() if controller is None else s.summary3()}


def run_pair(scenario='crossing', seed=7):
    """Baseline and AEGIS on the identical seeded scenario."""
    base, aeg = replay(scenario, False, seed), replay(scenario, True, seed)
    return {'scenario': make_scenario(scenario, seed).to_json(), 'baseline': base, 'aegis': aeg,
            'constants': {'dt': DT, 'robot_r': ROBOT_R, 'v_max': V_MAX, 'a_max': A_MAX, 'horizon': HORIZON,
                          'commit': COMMIT, 'clearance': CLEARANCE, 'sensor_range': SENSOR_RANGE}}


def _mean(xs):
    return round(sum(xs) / len(xs), 2) if xs else None


def benchmark(seeds=range(1000, 1020), types=SCENARIO_TYPES):
    seeds = list(seeds)
    rows, totals = [], {'pairs': 0, 'baseline_collisions': 0, 'aegis_collisions': 0, 'baseline_completed': 0,
                        'aegis_completed': 0, 'aegis_safe_where_baseline_collided': 0, 'both_collided': 0,
                        'aegis_only_collided': 0}
    for st in types:
        pairs = [(run_episode(st, False, s), run_episode(st, True, s)) for s in seeds]
        row = {'scenario': st, 'category': make_scenario(st, seeds[0]).category, 'pairs': len(pairs)}
        for label, idx in (('baseline', 0), ('aegis', 1)):
            rs = [p[idx] for p in pairs]
            row[label] = {
                'collisions': sum(r['collision'] for r in rs), 'completed': sum(r['completed'] for r in rs),
                'timeouts': sum(r['outcome'] == 'timeout' for r in rs),
                'safe_stops_no_path': sum(r['outcome'] == 'blocked_safe_stop' for r in rs),
                'mean_time_completed': _mean([r['elapsed'] for r in rs if r['completed']]),
                'mean_interventions': _mean([r['interventions'] for r in rs]),
                'stationary_contacts': sum(bool(r['collision_info'] and r['collision_info']['robot_stationary'])
                                           for r in rs),
                'anticipated_contacts': sum(bool(r['collision_info'] and
                                                 r['collision_info']['supervisor_declared_no_safe_action'])
                                            for r in rs),
            }
        row['paired'] = {
            'aegis_safe_where_baseline_collided': sum(b['collision'] and not a['collision'] for b, a in pairs),
            'both_collided': sum(b['collision'] and a['collision'] for b, a in pairs),
            'aegis_only_collided': sum(a['collision'] and not b['collision'] for b, a in pairs),
        }
        rows.append(row)
        totals['pairs'] += len(pairs)
        for k in ('collisions', 'completed'):
            totals[f'baseline_{k}'] += row['baseline'][k]
            totals[f'aegis_{k}'] += row['aegis'][k]
        for k in ('aegis_safe_where_baseline_collided', 'both_collided', 'aegis_only_collided'):
            totals[k] += row['paired'][k]
    return {
        'method': (f'{len(seeds)} paired episodes per scenario type (seeds {seeds[0]}-{seeds[-1]}); each pair '
                   'shares the identical seeded layout, unmapped obstacles and scripted mover trajectories. '
                   'Movers do not react to the robot and many are timed adversarially at the nominal robot '
                   'schedule. Simulated 2D kinematics only; not real-robot validation.'),
        'results': rows, 'totals': totals,
    }


# ---------------------------------------------------------------------- three-way experiment
def _agg(rs):
    done = [r for r in rs if r['completed']]
    return {
        'collisions': sum(r['collision'] for r in rs), 'completed': len(done),
        'timeouts': sum(r['outcome'] == 'timeout' for r in rs),
        'safe_stops_no_path': sum(r['outcome'] == 'blocked_safe_stop' for r in rs),
        'mean_time_completed': _mean([r['elapsed'] for r in done]),
        'mean_interventions': _mean([r['interventions'] for r in rs]),
        'mean_stop_events': _mean([r['stop_events'] for r in rs]),
        'mean_stopped_time': _mean([r['stopped_time'] for r in rs]),
    }


def benchmark3(seeds=range(1000, 1030), types=SCENARIO_TYPES, controllers=CONTROLLERS):
    """Basic vs Reactive vs AEGIS on identical (type, seed) scenarios.

    'Unnecessary stop' proxy: in episodes where Basic completed without collision, the
    nominal plan was collision-free for that (open-loop) scenario, so every stop and every
    second of extra travel time by another controller was not needed for safety there.
    """
    seeds = list(seeds)
    rows, all_runs = [], {c: [] for c in controllers}
    for st in types:
        runs = {c: [run_episode(st, seed=s, controller=c) for s in seeds] for c in controllers}
        for c in controllers:
            all_runs[c] += runs[c]
        row = {'scenario': st, 'category': make_scenario(st, seeds[0]).category, 'episodes': len(seeds)}
        row.update({c: _agg(runs[c]) for c in controllers})
        if {'reactive', 'aegis'} <= set(controllers):
            R, A = runs['reactive'], runs['aegis']
            both = [(r, a) for r, a in zip(R, A) if r['completed'] and a['completed']]
            row['reactive_vs_aegis'] = {
                'aegis_only_collided': sum(a['collision'] and not r['collision'] for r, a in zip(R, A)),
                'reactive_only_collided': sum(r['collision'] and not a['collision'] for r, a in zip(R, A)),
                'both_collided': sum(r['collision'] and a['collision'] for r, a in zip(R, A)),
                'aegis_only_completed': sum(a['completed'] and not r['completed'] for r, a in zip(R, A)),
                'reactive_only_completed': sum(r['completed'] and not a['completed'] for r, a in zip(R, A)),
                'both_completed': len(both),
                'mean_time_diff_aegis_minus_reactive': _mean([a['elapsed'] - r['elapsed'] for r, a in both]),
                'aegis_faster': sum(a['elapsed'] < r['elapsed'] - 1e-9 for r, a in both),
                'reactive_faster': sum(r['elapsed'] < a['elapsed'] - 1e-9 for r, a in both),
            }
        rows.append(row)
    totals = {c: _agg(all_runs[c]) for c in controllers}
    totals['episodes_per_controller'] = len(seeds) * len(types)
    if 'basic' in controllers:
        ok = [i for i, b in enumerate(all_runs['basic']) if b['completed'] and not b['collision']]
        totals['basic_clean_episodes'] = len(ok)
        totals['on_basic_clean_episodes'] = {
            c: {'stop_events': sum(all_runs[c][i]['stop_events'] for i in ok),
                'episodes_with_a_stop': sum(all_runs[c][i]['stop_events'] > 0 for i in ok),
                'collisions': sum(all_runs[c][i]['collision'] for i in ok),
                'not_completed': sum(not all_runs[c][i]['completed'] for i in ok),
                'mean_extra_time_vs_basic': _mean([all_runs[c][i]['elapsed'] - all_runs['basic'][i]['elapsed']
                                                   for i in ok if all_runs[c][i]['completed']])}
            for c in controllers if c != 'basic'}
    if {'reactive', 'aegis'} <= set(controllers):
        keys = ('aegis_only_collided', 'reactive_only_collided', 'both_collided', 'aegis_only_completed',
                'reactive_only_completed', 'both_completed', 'aegis_faster', 'reactive_faster')
        totals['reactive_vs_aegis'] = {k: sum(r['reactive_vs_aegis'][k] for r in rows) for k in keys}
    return {
        'method': (f'{len(seeds)} episodes per scenario type per controller (seeds {seeds[0]}-{seeds[-1]}); '
                   'all controllers see the identical seeded scenario, dynamics and nominal A* tracker. '
                   'Reactive and AEGIS share perception (range, occlusion, noise), tracking, static '
                   'discovery and replanning; Reactive uses current observations only. Simulated 2D only.'),
        'controllers': list(controllers), 'results': rows, 'totals': totals,
    }
