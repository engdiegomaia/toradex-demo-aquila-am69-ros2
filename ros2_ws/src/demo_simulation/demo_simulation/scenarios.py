"""
Demo scenarios: world, spawn pose, and camera framing.

WHY THIS TABLE EXISTS

Before it, the nine numbers that describe a scenario lived in three places:
the `world` in the launch file, the spawn `yaw` in another launch file, and
the framing of the two scene cameras in a third -- with a FOURTH copy in a
comment in `docker/compose.host.yml`, in the form of a six-line SIM_ARGS the
operator had to paste by hand.

That produces one specific silent failure, and it already happened: changing
the world without changing the framing. The maze has its usable area centred
on (-4.855, 4.855), and the warehouse is centred on the origin, so the maze
world with the warehouse camera shows empty ground next to the maze. Nothing
errors, nothing logs: the cockpit's blue panel simply points at the wrong
place, and whoever is watching concludes the robot is not moving.

The fix is structural, not a matter of discipline: the framing is now
DERIVED from the world. There is no longer a way to pick one without the
other, because there are no longer two places to pick from.

HOW A VALUE GETS OVERRIDDEN

Every `scene_*` and `yaw` argument in the launch files accepts empty (the
default), which means "ask this table". Any non-empty value wins. So probing
a corner of the maze is still just `scene_top_z:=20.0` on the command line,
with no file to edit, and without losing the other eight numbers.

WHERE EACH NUMBER COMES FROM

None were chosen: all were measured or computed, and the working is in
`launch/scene_cameras.launch.py`. Summary of the source:

- warehouse: iso at (-3, +3) because the (-x,-y) quadrant falls inside the
  shelving aisle; top at 6 m because at 12 m the camera ends up ABOVE the
  roof trusses and the whole image turns into an orange beam;
- maze11: centre and extent came from the STL bounding box read from the
  binary and multiplied by the 0.002 scale (see `tools/maze/maze_fit.py`),
  not from the file name.

maze11 is the quadruped's OFFICIAL scenario. The warehouse remains the
diff-drive's, which is the fallback and was validated there -- see
`robot_selection.py`.
"""

import os

# Generic framing: both cameras looking at the origin. Serves the small
# worlds centred on the origin (empty, corridor, objects, ramp, rough), where
# the usable area IS the origin. Does NOT serve maze11, which is exactly why
# it has its own entry.
GENERIC = {
    'spawn': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    # (x, y, z, pitch, yaw)
    'scene_iso': (-3.0, 3.0, 2.4, 0.5150, -0.7854),
    'scene_top': (0.0, 0.0, 6.0, 1.5708, 1.5708),
}

SCENARIOS = {
    # Quadruped's OFFICIAL scenario. 1.20 m corridor, 0.40 m wall, 11.60 x
    # 11.60 m footprint centred on (-4.855, 4.855).
    #
    # yaw 1.5708 spawns the robot looking DOWN THE CORRIDOR, not at the wall.
    # Without that the first thing Nav2 has to do is a 90-degree turn inside
    # a 1.20 m corridor, which eats up the first seconds of any run and
    # pollutes the comparison between conditions.
    #
    # The maze has no ceiling, so the top view can go higher: 13 m covers
    # 17.8 x 13.3 m, the 11.6 m with margin. The iso view comes from outside
    # and below (-13, -3, 9) so as not to look from inside a corridor.
    'quadruped_maze11.sdf': {
        'spawn': {'x': 0.0, 'y': 0.0, 'yaw': 1.5708},
        'scene_iso': (-13.0, -3.0, 9.0, 0.6717, 0.7676),
        'scene_top': (-4.855, 4.855, 13.0, 1.5708, 1.5708),
        # Mesh EXTERNAL to the repository -- see `external_models` below.
        'needs_models': ('maze11',),
    },
    # The diff-drive's scenario, which is the validated fallback. The
    # warehouse is about 28 x 45 m, and framing it completely would require
    # h ~ 44 m, a height at which the robot becomes a handful of pixels; the
    # generic framing is deliberate.
    'warehouse.sdf': GENERIC,
}

# Where compose mounts the external models inside the `sim` container.
# Mirrors the volume in docker/compose.host.yml; if one of the two changes,
# the `missing_models` check stops finding anything and goes silent again.
EXTERNAL_MODELS_DIR = '/maze/models'


def scenario(world: str) -> dict:
    """
    Return the scenario for a world, by FILE name.

    By file name and not by the `<world>` element's name: here the input is
    a launch path, and reading the SDF to discover the internal name would
    cost a file read at launch time to resolve what the path itself already
    says. (Where the distinction really matters -- the Gazebo service -- is
    in sim_control.launch.py, which documents that case.)

    A world with no entry falls back to the generic one. That is
    deliberate: a new world works without touching this table, and only
    needs an entry when the usable area is NOT on the origin.
    """
    return SCENARIOS.get(os.path.basename(world), GENERIC)


def camera_pose(world: str, camera: str) -> tuple:
    """Pose (x, y, z, pitch, yaw) of `scene_iso` or `scene_top`."""
    key = f'scene_{camera}'
    if key not in ('scene_iso', 'scene_top'):
        raise ValueError(f'unknown camera {camera!r}: use iso or top.')
    return scenario(world)[key]


def spawn_pose(world: str) -> dict:
    """Robot spawn pose: x, y, yaw."""
    return scenario(world)['spawn']


def external_models(world: str) -> tuple:
    """Models the world loads that are NOT in the repository."""
    return tuple(scenario(world).get('needs_models', ()))


def missing_models(world: str, root: str = EXTERNAL_MODELS_DIR) -> tuple:
    """
    Which external models the world requires and are not mounted.

    Exists because the failure is COMPLETELY silent. maze11's SDF references
    `model://maze11/meshes/maze11.stl`; without the mesh Gazebo loads the
    world, the model ends up with no visual and no collision, and the result
    is an empty plane. The lidar sees nothing, Nav2 plans a straight line and
    finishes with SUCCEEDED. In other words: the run PASSES, with better
    numbers than the real ones, and nothing in the output says the maze was
    not there.

    It is even worse on compose's default path: `MAZE_MODELS:-./models-extra`
    points at a directory that does not exist in the repository, and Docker
    creates an EMPTY directory instead of failing.
    """
    return tuple(
        name for name in external_models(world)
        if not os.path.isdir(os.path.join(root, name))
    )
