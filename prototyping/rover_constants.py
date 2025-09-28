import numpy as np

"""Rover geometric constants (units: millimeters)


Coordinate Frame (body frame / base_link):
    +X forward, +Y left, +Z up.

Conventional Naming (standardized here):
    WHEEL_BASE            Longitudinal distance between front-most and rear-most wheel contact points (A↔C).
    TRACK_WIDTH_MIDDLE    Lateral distance between left & right MIDDLE wheels (old name: WHEEL_BASE).
    TRACK_WIDTH_STEERING  Lateral distance between left & right STEERING wheel pairs (front & rear) (old name: STEERING_WHEEL_BASE).

Legacy Inversion (before this refactor):
    TRACK_WIDTH held the longitudinal distance and WHEEL_BASE the lateral middle spacing.
    This inversion is now corrected; no compatibility aliases are provided to expose stale usages during migration.

Ratios:
    FRONT_TO_MIDDLE_RATIO = (front wheel x offset) / WHEEL_BASE
    REAR_TO_MIDDLE_RATIO  = (rear wheel x offset)  / WHEEL_BASE
    These were manually measured; their sum slightly exceeds 1 due to rocker geometry & measurement error (~1.01226). Acceptable for now.

Wheel Locations Mapping:
    WHEEL_LOCATIONS[name] = (x, y) in body frame.
     x: longitudinal offsets derived from WHEEL_BASE and ratios.
     y: half of the appropriate lateral spacing:
                - Middle wheels use TRACK_WIDTH_MIDDLE / 2
                - Front & rear steering wheels use TRACK_WIDTH_STEERING / 2

If future designs differentiate front vs rear steering lateral spacing, introduce
    TRACK_WIDTH_FRONT_STEERING and TRACK_WIDTH_REAR_STEERING and update WHEEL_LOCATIONS.

Migration Guidance:
    Replace imports of deprecated names (TRACK_WIDTH, STEERING_WHEEL_BASE) with the explicit new semantic constant.
    Re-derive any kinematic matrices (e.g., mecanum) using:
            L = WHEEL_BASE / 2 (half-length), W = TRACK_WIDTH_MIDDLE / 2 (half-width)
    rather than the legacy swapped usage.
"""

# ---------------- Core Dimensions ----------------
WHEEL_DIAMETER = 70  # Wheel diameter [mm]
WHEEL_RADIUS = WHEEL_DIAMETER / 2  # Wheel radius [mm]
# Longitudinal wheelbase (front↔rear A to C) [mm]  (was TRACK_WIDTH)
WHEEL_BASE = 250
# Lateral middle wheel spacing (was WHEEL_BASE) [mm]
TRACK_WIDTH_MIDDLE = 294.5
# Lateral steering wheel spacing (was STEERING_WHEEL_BASE) [mm]
TRACK_WIDTH_STEERING = 255.5
ROVER_BODY_WIDTH = 150         # Central body width [mm]
ROVER_BODY_LENGTH = 240        # Central body length [mm]

# ---------------- Longitudinal Ratios (manual measurements) ----------------
FRONT_TO_MIDDLE_RATIO = 0.533416
REAR_TO_MIDDLE_RATIO = 0.478844

# ---------------- Wheel Locations (Body Frame) ----------------
WHEEL_LOCATIONS = {
    # Front steering pair
    "front_left":  (FRONT_TO_MIDDLE_RATIO * WHEEL_BASE,  TRACK_WIDTH_STEERING / 2),
    "front_right": (FRONT_TO_MIDDLE_RATIO * WHEEL_BASE, -TRACK_WIDTH_STEERING / 2),
    # Middle fixed pair
    "middle_left": (0,  TRACK_WIDTH_MIDDLE / 2),
    "middle_right": (0, -TRACK_WIDTH_MIDDLE / 2),
    # Rear steering pair
    "rear_left":  (-REAR_TO_MIDDLE_RATIO * WHEEL_BASE,  TRACK_WIDTH_STEERING / 2),
    "rear_right": (-REAR_TO_MIDDLE_RATIO * WHEEL_BASE, -TRACK_WIDTH_STEERING / 2),
}
STEERABLE = {"front_left", "front_right", "rear_left", "rear_right"}
# ---------------------------------------------------------------------------
PLOT_SCALE = 1000  # Scale down for plotting
WHEEL_LOCATIONS_PLOT = {
    name: np.array(loc) / PLOT_SCALE for name, loc in WHEEL_LOCATIONS.items()
}
# Change to raw dictionary again for easier import without numpy dependency
WHEEL_LOCATIONS_SCALED = {name: (
    loc[0] / PLOT_SCALE, loc[1] / PLOT_SCALE) for name, loc in WHEEL_LOCATIONS.items()}
print(f"WHEEL_LOCATIONS: {WHEEL_LOCATIONS}")
print(f"WHEEL_LOCATIONS_PLOT: {WHEEL_LOCATIONS_PLOT}")
print(f"WHEEL_LOCATIONS_SCALED: {WHEEL_LOCATIONS_SCALED}")
