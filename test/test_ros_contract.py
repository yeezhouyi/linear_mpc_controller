"""Structural ROS2 contract tests (run WITHOUT ROS on any host).

These verify file-level contracts only (single /cmd_vel writer, node
parameters declared in the yaml, interface files present).  Runtime topic
tests belong to the WSL2 colcon gate (test_ros_contract.py extension there).
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
ROS2 = ROOT / "ros2"
CONFIG = ROOT / "config"


def _src_text() -> str:
    return "\n".join(p.read_text(encoding="utf-8") for p in ROS2.glob("*.cpp"))


def test_single_cmd_vel_writer():
    """Exactly one publisher may write /cmd_vel: the velocity arbiter."""
    text = _src_text()
    pubs = re.findall(r'create_publisher<[^>]+>\(\s*"/cmd_vel"', text)
    assert len(pubs) == 1, f"expected 1 /cmd_vel publisher, found {len(pubs)}"


def test_mpc_publishes_candidate_not_cmd_vel():
    node = (ROS2 / "linear_mpc_node.cpp").read_text(encoding="utf-8")
    assert '"cmd_vel_mpc"' in node
    assert '"/cmd_vel"' not in node


def test_node_parameters_covered_by_yaml():
    node = (ROS2 / "linear_mpc_node.cpp").read_text(encoding="utf-8")
    declared = set(re.findall(r'declare_parameter\("([^"]+)"', node))
    yaml = (CONFIG / "linear_mpc_params.yaml").read_text(encoding="utf-8")
    # ros2 maps nested yaml keys like adapter/v_default to "adapter.v_default";
    # check the leaf key appears in the yaml document.
    missing = [d for d in declared if d.split(".")[-1] not in yaml]
    assert not missing, f"yaml missing parameters: {missing}"


def test_core_files_present():
    expected = [
        "include/linear_mpc_controller/model/differential_drive_model.hpp",
        "include/linear_mpc_controller/mpc/linear_mpc.hpp",
        "include/linear_mpc_controller/mpc/qp_problem.hpp",
        "include/linear_mpc_controller/mpc/fallback_policy.hpp",
        "include/linear_mpc_controller/safety/actuation_projection.hpp",
    ]
    for rel in expected:
        assert (ROOT / rel).exists(), f"missing {rel}"


def test_worlds_are_valid_sdf_documents():
    sdf = (ROOT / "worlds" / "tracking_empty.sdf").read_text(encoding="utf-8")
    assert "<sdf version=" in sdf and "<world" in sdf and "</sdf>" in sdf


def test_node_uses_odom_freshness_gate():
    # R9: odom staleness must gate output (review fix 1).
    node = (ROS2 / "linear_mpc_node.cpp").read_text(encoding="utf-8")
    assert "odomIsStale(" in node
    assert "odom_max_age_s_" in node
    assert "STATE_STALE" in node
    assert "odom_backwards_" in node


def test_seeking_guard_comes_before_qp_fail_count():
    # SEEKING/probation handled before QP-failure counting so a no-QP
    # seeking cycle (default kFailed) cannot trip qp_fail_max (fix 3).
    text = (ROS2 / "nav2_mpc_controller.cpp").read_text(encoding="utf-8")
    i_seek = text.find("if (res.reacquire_seeking || res.in_probation)")
    i_fail = text.find("res.qp_status == QpSolution::Status::kFailed")
    assert -1 not in (i_seek, i_fail)
    assert i_seek < i_fail, (
        "SEEKING/probation must be handled BEFORE the QP-failure counter"
    )


def test_plugin_uses_plugin_name_after_move():
    # After `plugin_name_ = std::move(name);` the moved-from `name` must
    # never be used for parameter namespaces (fix 4).
    text = (ROS2 / "nav2_mpc_controller.cpp").read_text(encoding="utf-8")
    head = text.find("plugin_name_ = std::move(name);")
    assert head != -1
    tail = text.find("void LinearMpcNav2Controller::cleanup()")
    body = text[head:tail if tail != -1 else len(text)]
    assert "name + " not in body, (
        "moved-from 'name' still used for parameter strings after the move"
    )
    assert "plugin_name_ + " in body


def test_plugin_parameter_namespace_suffixes_covered_by_yaml():
    # Every parameter the plugin reads under plugin_name_ must exist in
    # the controller_server yaml block (non-default load path).
    cpp = (ROS2 / "nav2_mpc_controller.cpp").read_text(encoding="utf-8")
    declared = set(re.findall(r'plugin_name_ \+ "\.([A-Za-z0-9_]+)"', cpp))
    assert declared, "no plugin parameters found"
    yaml = (CONFIG / "controller_server.yaml").read_text(encoding="utf-8")
    missing = [d for d in declared if d not in yaml]
    assert not missing, f"controller_server.yaml missing plugin params: {missing}"


def test_staleness_header_present():
    assert (ROOT / "include/linear_mpc_controller/safety/odom_staleness.hpp").exists()
