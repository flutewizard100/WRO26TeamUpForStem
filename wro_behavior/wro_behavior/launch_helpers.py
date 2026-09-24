"""Launch-time helpers shared by the challenge launch files.

Read once, apply once, in-process at launch-generation time. Avoids
lazy substitution (RewrittenYaml) which materializes the merged yaml
per Nav2 node and forces a full re-parse of the base yaml each time.
"""
import os
from typing import Any, Dict

import yaml


def load_mission_params(tuning_yaml_path: str, node_name: str) -> Dict[str, Any]:
    """Extract `<node_name>.ros__parameters` from the tuning yaml as
    an inline dict, ready to hand to Node(parameters=[...])."""
    with open(tuning_yaml_path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return (data.get(node_name, {}) or {}).get('ros__parameters', {}) or {}


def build_merged_nav2_params(
    tuning_yaml_path: str,
    nav2_base_path: str,
    out_name: str = 'wro_nav2_merged.yaml',
    out_dir: str = '/tmp',
) -> str:
    """Deep-merge `nav2_overrides` from the tuning yaml onto the base
    Nav2 yaml. Overrides use dotted paths (e.g.
    "planner_server.ros__parameters.GridBased.cost_penalty") so a same-
    named key elsewhere in the base isn't accidentally clobbered.

    Writes the merged yaml to `<out_dir>/<out_name>` and returns the
    path. Overwrites on every launch — safe because the launch is
    what re-generates it.
    """
    with open(tuning_yaml_path, 'r') as f:
        tuning = yaml.safe_load(f) or {}
    overrides = tuning.get('nav2_overrides', {}) or {}

    with open(nav2_base_path, 'r') as f:
        nav2_data = yaml.safe_load(f) or {}

    for dotted_key, value in overrides.items():
        _deep_set(nav2_data, dotted_key.split('.'), value)

    out_path = os.path.join(out_dir, out_name)
    with open(out_path, 'w') as f:
        yaml.safe_dump(nav2_data, f, default_flow_style=False, sort_keys=False)
    return out_path


def _deep_set(root: dict, path: list, value: Any) -> None:
    node = root
    for p in path[:-1]:
        nxt = node.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            node[p] = nxt
        node = nxt
    node[path[-1]] = value
