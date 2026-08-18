"""
scenario_loader

Parses CommonOcean-format scenario XML files (e.g. scenarios/*.xml -- Marine
Cadastre AIS data converted to CommonOcean by TU Munich, see
https://commonocean.cps.cit.tum.de) into plain dataclasses, and provides the
random ship-type assignment + coordinate re-anchoring used to turn a parsed
scenario into a spawnable/replayable traffic-vessel fleet.

Stdlib-only (xml.etree.ElementTree, random) so it's safe to import both from
launch/gz_sim.launch.py (evaluated by `ros2 launch` itself, at launch
*description* time, not inside a running node) and from
obstacle_vessel_controler.py at node runtime -- both call the exact same
parse -> ship_obstacles -> reanchor -> assign_vessel_types pipeline, so as
long as they're given the same scenario_file + scenario_seed, they
independently derive the identical vessel fleet without needing to pass
anything else between the two processes.
"""

import random
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional


@dataclass
class StateSample:
    """One (time, position, heading, speed) sample, from either a scenario
    obstacle's initialState or one entry of its trajectory/state list."""
    t: int       # raw scenario timestep -- not seconds, scale by
                 # ScenarioData.time_step_size to get elapsed seconds.
    x: float
    y: float
    yaw: float   # radians, planar heading -- not a compass bearing.
    v: float     # scalar speed, m/s.


@dataclass
class ScenarioObstacle:
    id: int
    source: str        # 'dynamic' or 'static'
    raw_type: str       # CommonOcean <type> text, e.g. 'motorvessel'
    initial: StateSample
    trajectory: list    # list[StateSample], chronological, `initial` is
                         # always trajectory[0] -- a staticObstacle (no
                         # <trajectory> of its own) ends up with a
                         # single-sample trajectory, which the replay
                         # controller naturally holds forever.


@dataclass
class ScenarioData:
    time_step_size: float
    obstacles: list                  # list[ScenarioObstacle]
    ego_initial_xy: Optional[tuple]  # (x, y) of the <planningProblem>'s
                                      # initialState, or None if absent --
                                      # the anchor reanchor() shifts against.
    ego_initial_yaw: Optional[float]  # radians, from the same initialState --
                                       # None iff ego_initial_xy is None.
    ego_goal_xy: Optional[tuple]    # (x, y) center of the <planningProblem>'s
                                     # goalState region, or None if absent.
                                     # Raw scenario-frame coordinates, same as
                                     # ego_initial_xy -- reanchor_xy() shifts
                                     # this the same way reanchor() shifts
                                     # obstacles, before sending it anywhere.
    ego_goal_radius: Optional[float]  # half the goal region's smaller extent
                                       # (rectangle/circle), or None for a
                                       # bare point goal or no goalState.


# CommonOcean <type> strings that denote an actual ship/vessel, as opposed
# to some other obstacle kind (buoy, land, ...) this repo has no model for.
# Only obstacles whose type is in this list are candidates for random
# vessel-model assignment -- extend as new scenario files introduce new
# type strings.
SHIP_TYPES = {
    'motorvessel', 'sailboat', 'sailingvessel', 'tanker', 'fishingvessel',
    'containership', 'cargoship', 'passengership', 'militaryvessel',
    'pilotvessel', 'tugboat',
}


def _parse_state(state_elem) -> StateSample:
    point = state_elem.find('position/point')
    time_elem = state_elem.find('time/exact')
    velocity_elem = state_elem.find('velocity/exact')
    return StateSample(
        t=int(time_elem.text) if time_elem is not None else 0,
        x=float(point.find('x').text),
        y=float(point.find('y').text),
        yaw=float(state_elem.find('orientation/exact').text),
        # staticObstacle initialStates have no <velocity> -- they don't move.
        v=float(velocity_elem.text) if velocity_elem is not None else 0.0,
    )


def _parse_goal_position(position_elem):
    """Return ((x, y), radius) from a planningProblem goalState's <position>
    element -- handles the region shapes CommonOcean scenario files use for
    a goal (rectangle, circle, or a bare point). radius is half the
    region's smaller extent for a rectangle/circle, or None for a point
    (no implied acceptance radius) or an unrecognized shape."""
    rect = position_elem.find('rectangle')
    if rect is not None:
        center = rect.find('center')
        radius = min(float(rect.find('length').text), float(rect.find('width').text)) / 2.0
        return (float(center.find('x').text), float(center.find('y').text)), radius
    circle = position_elem.find('circle')
    if circle is not None:
        center = circle.find('center')
        return (float(center.find('x').text), float(center.find('y').text)), float(circle.find('radius').text)
    point = position_elem.find('point')
    if point is not None:
        return (float(point.find('x').text), float(point.find('y').text)), None
    return None, None


def _parse_obstacle(elem, source: str) -> ScenarioObstacle:
    initial = _parse_state(elem.find('initialState'))
    trajectory = [initial]
    traj_elem = elem.find('trajectory')
    if traj_elem is not None:
        trajectory += [_parse_state(s) for s in traj_elem.findall('state')]
    trajectory.sort(key=lambda s: s.t)
    type_elem = elem.find('type')
    return ScenarioObstacle(
        id=int(elem.get('id')),
        source=source,
        raw_type=(type_elem.text or '').strip() if type_elem is not None else '',
        initial=initial,
        trajectory=trajectory,
    )


def parse_scenario(path: str) -> ScenarioData:
    """Parse a CommonOcean scenario XML file."""
    root = ET.parse(path).getroot()
    time_step_size = float(root.get('timeStepSize', '1.0'))

    obstacles = (
        [_parse_obstacle(e, 'dynamic') for e in root.findall('dynamicObstacle')] +
        [_parse_obstacle(e, 'static') for e in root.findall('staticObstacle')]
    )

    ego_initial_xy = None
    ego_initial_yaw = None
    pp_initial = root.find('planningProblem/initialState')
    if pp_initial is not None:
        # Same element shape as a dynamicObstacle/staticObstacle initialState
        # (position/point, orientation/exact, velocity/exact, time/exact), so
        # reuse _parse_state instead of hand-parsing position only.
        state = _parse_state(pp_initial)
        ego_initial_xy = (state.x, state.y)
        ego_initial_yaw = state.yaw

    ego_goal_xy = None
    ego_goal_radius = None
    goal_position = root.find('planningProblem/goalState/position')
    if goal_position is not None:
        ego_goal_xy, ego_goal_radius = _parse_goal_position(goal_position)

    return ScenarioData(time_step_size=time_step_size, obstacles=obstacles,
                         ego_initial_xy=ego_initial_xy,
                         ego_initial_yaw=ego_initial_yaw,
                         ego_goal_xy=ego_goal_xy,
                         ego_goal_radius=ego_goal_radius)


def ship_obstacles(scenario: ScenarioData):
    """Split scenario.obstacles into (ship-like, everything else) using
    SHIP_TYPES as a small explicit allow-list. Non-ship obstacles (buoys,
    land, ...) have no vessel model in this repo -- callers should skip/log
    the second list rather than assigning one of the 5 vessel models to it."""
    kept, skipped = [], []
    for obstacle in scenario.obstacles:
        (kept if obstacle.raw_type.lower() in SHIP_TYPES else skipped).append(obstacle)
    return kept, skipped


def reanchor_xy(xy, ego_xy, scenario_ego_xy):
    """Shift a single (x, y) point by the same (ego_xy - scenario_ego_xy)
    delta reanchor() applies to every obstacle sample -- used for the
    planningProblem's own goalState position (ego_goal_xy), which needs the
    identical translation to land consistently with the reanchored
    ego/obstacles rather than out at the scenario's own recorded
    coordinates. Returns xy unchanged if either is None (nothing recorded
    to anchor, or nothing to anchor it against)."""
    if xy is None or scenario_ego_xy is None:
        return xy
    return (xy[0] + ego_xy[0] - scenario_ego_xy[0], xy[1] + ego_xy[1] - scenario_ego_xy[1])


def reanchor(obstacles, ego_xy, scenario_ego_xy):
    """Translate every given obstacle's initial + trajectory positions so
    the scenario's recorded position *relative to its own planningProblem
    ego* is preserved, but re-centered on wherever the ego actually spawns
    in the current world (ego_xy) instead of the scenario's own local-frame
    origin. Without this, a real Marine Cadastre scenario's obstacles can
    land kilometers from the ego (they're recorded in the scenario's own
    geo-anchored local frame, not relative to the ego) and never appear in
    a small demo world. No rotation is applied -- orientation values are
    already relative headings, not bearings tied to the scenario's frame.

    Returns a new list of ScenarioObstacle (does not mutate the input). If
    scenario_ego_xy is None (no <planningProblem> in the file), returns
    obstacles unchanged -- there's nothing to anchor against.
    """
    if scenario_ego_xy is None:
        return obstacles

    def _shift(sample: StateSample) -> StateSample:
        x, y = reanchor_xy((sample.x, sample.y), ego_xy, scenario_ego_xy)
        return StateSample(t=sample.t, x=x, y=y, yaw=sample.yaw, v=sample.v)

    return [
        ScenarioObstacle(
            id=obstacle.id, source=obstacle.source, raw_type=obstacle.raw_type,
            initial=_shift(obstacle.initial),
            trajectory=[_shift(sample) for sample in obstacle.trajectory],
        )
        for obstacle in obstacles
    ]


def assign_vessel_types(obstacles, catalog_names, seed=None) -> dict:
    """Randomly assign one of catalog_names to each obstacle's id --
    independent draws (duplicates allowed, no uniqueness requirement).
    Deterministic when seed is given: calling this again with the same
    obstacles (same ids, same order) and the same seed reproduces the same
    assignment -- this is what lets gz_sim.launch.py and
    obstacle_vessel_controler.py agree on the fleet without passing
    anything but scenario_file + scenario_seed between them."""
    rng = random.Random(seed)
    return {obstacle.id: rng.choice(catalog_names) for obstacle in obstacles}


def resolve_seed(seed_str: str) -> int:
    """Turn a possibly-empty scenario_seed launch-argument string into a
    concrete int: the given value if non-empty, otherwise a freshly-drawn
    random one. Meant to be called exactly once per launch (by
    gz_sim.launch.py), so the *resolved* value can be handed to
    obstacle_vessel_controler.py as its own scenario_seed parameter --
    letting both sides reach the same assignment even when the user never
    specified a seed themselves."""
    return int(seed_str) if seed_str else random.SystemRandom().randrange(2**31)


def vessel_tag(vessel_type: str, obstacle_id: int) -> str:
    """The spawn name / ROS namespace tag for a scenario-assigned vessel --
    shared by gz_sim.launch.py (spawn) and obstacle_vessel_controler.py
    (replay) so both land on the same /<tag>/cmd_vel, /<tag>/odom topics."""
    return f'{vessel_type}_{obstacle_id}'
