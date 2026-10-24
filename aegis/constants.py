"""Physical and controller constants shared by every module (SI units)."""

DT = 0.10                 # simulation and control period [s]
EPISODE_LIMIT = 25.0      # episode timeout [s]

FIELD_W = 12.0            # warehouse floor [m]
FIELD_H = 10.0

ROBOT_R = 0.28            # robot footprint radius [m]
V_MAX = 1.5               # robot cruise / max speed [m/s]
A_MAX = 2.0               # max magnitude of velocity change per second [m/s^2]
GOAL_TOL = 0.25           # goal reached when centre is within this distance [m]

SENSOR_RANGE = 6.0        # perception range [m]; shelves also occlude line of sight
TRACK_COAST = 1.0         # keep forecasting an occluded track this long [s]

# Supervisor parameters (frozen before the held-out benchmark was run; see README).
HORIZON = 2.0             # forecast horizon [s]
COMMIT = 0.4              # each candidate is held this long, then max braking [s]
CLEARANCE = 0.15          # required clearance to moving obstacles [m]
STATIC_CLEARANCE = 0.05   # required clearance to shelves / known static obstacles [m]
GROWTH_RATE = 0.20        # forecast disk growth for behaviour changes [m per s of lookahead]
GROWTH_CAP = 0.40
UNKNOWN_VEL_GROWTH = 0.9  # growth while a track has <2 observations (velocity unknown)
NOISE_K = 2.0             # margin += NOISE_K * sensor sigma

PLANNER_RES = 0.2
PLANNER_INFLATION = ROBOT_R + 0.17
