"""
vessel_catalog

The "type profile" table for scenario-driven obstacle vessels: which
urdf/*_vessel.urdf models exist, their waterline z-spawn-offset, and where
to source their mock AIS particulars (loa/beam/mmsi/safety_radius/type)
from. Used by launch/gz_sim.launch.py (to spawn a scenario's ship-like
obstacles as one of these models) and
hybraut_nav_colreg_sim/obstacle_vessel_controler.py (to reconstruct the
same fleet at runtime -- see scenario_loader.py's module docstring).

Deliberately independent of gz_sim.launch.py's own hardcoded DEFAULT_VESSELS
table (which has its own copy of these same z-offsets, for its own 5 named
vessels) rather than refactoring that table to import from here -- keeps
the scenario_file:=unset default path completely unaffected by anything in
this module.
"""

import yaml

# urdf/<name>.urdf stems available to be randomly assigned to a scenario's
# ship-like obstacles.
VESSEL_TYPES = [
    'hydrofoil_vessel',
    'sail_boat_vessel',
    'small_motor_boat_vessel',
    'super_tanker_vessel',
    'trauler_vessel',
]

# Per-type waterline z-spawn-offset -- same values and reasoning as
# gz_sim.launch.py's own DEFAULT_VESSELS table (each hull's lowest point
# sits at world z=0 once spawned at this height). Keep the two in sync by
# hand if a vessel model's geometry ever changes.
SPAWN_Z = {
    'hydrofoil_vessel': 3.425,
    'sail_boat_vessel': 5.136,
    'small_motor_boat_vessel': 1.1,
    'super_tanker_vessel': 14.0,
    'trauler_vessel': 7.357,
}

# Where the ego always spawns -- gz_sim.launch.py's spawn_entity uses this
# for -x/-y unconditionally, *never* a scenario's own recorded
# <planningProblem> position, even when one is present. CommonOcean/Marine
# Cadastre scenario files aren't guaranteed to use a small local frame (e.g.
# scenarios/USA_NYM-1_20190613_T-1.xml's ego sits at real UTM-projected
# coordinates in the hundreds-of-thousands of metres) -- spawning there
# would put the ego millions of metres from Gazebo's own origin. Instead,
# scenario_loader.reanchor() rigidly shifts every obstacle by
# (EGO_SPAWN_XY - scenario_ego_xy), so each keeps its real recorded
# distance/bearing from the ego but re-centered near this small anchor. A
# single shared constant rather than a hardcoded (0.0, 0.0) in two places,
# since gz_sim.launch.py and obstacle_vessel_controler.py both need to
# reanchor identically.
EGO_SPAWN_XY = (0.0, 0.0)


def load_particulars(mock_ais_yaml_path: str) -> dict:
    """Load config/mock_ais_vessels.yaml and return {vessel_type:
    {mmsi, type, loa, beam, safety_radius}}, keyed by the static file's
    existing tags (which double as this catalog's VESSEL_TYPES) -- reuses
    those particulars as this catalog's per-type profile instead of
    hand-duplicating loa/beam/mmsi numbers a third time. Excludes the
    'agent' entry (not a vessel type)."""
    with open(mock_ais_yaml_path) as f:
        data = yaml.safe_load(f) or {}
    return {
        vessel['tag']: {k: vessel[k] for k in ('mmsi', 'type', 'loa', 'beam', 'safety_radius')}
        for vessel in data.get('vessels', [])
        if vessel['tag'] != 'agent'
    }


def load_agent_entry(mock_ais_yaml_path: str) -> dict:
    """Load config/mock_ais_vessels.yaml's own 'agent' entry verbatim, so a
    scenario-generated AIS particulars file can carry it through too (the
    ego's own safety-domain marker depends on it -- see
    vessel_ais_bridge.py's _publish_cb). Returns {} if not found."""
    with open(mock_ais_yaml_path) as f:
        data = yaml.safe_load(f) or {}
    for vessel in data.get('vessels', []):
        if vessel['tag'] == 'agent':
            return vessel
    return {}
