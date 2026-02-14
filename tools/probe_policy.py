#!/usr/bin/env python3
"""Probe an ONNX locomotion policy to diagnose observation mismatches.

Tests joint ordering (type-first vs leg-first), checks for baked-in
normalization, and simulates feedback loops to identify the root cause
of policy railing.

Usage:
    python3 tools/probe_policy.py
    python3 tools/probe_policy.py --model path/to/model.onnx
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    sys.exit("onnxruntime not installed. pip install onnxruntime")

try:
    import onnx
    from onnx import numpy_helper

    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False

# Default model path (relative to repo root).
DEFAULT_MODEL = os.path.join(
    os.path.dirname(__file__),
    "..", "src", "blind_locomotion", "share", "models",
    "locomotion_policy_v2.onnx",
)

# Joint names in both orderings.
TYPE_FIRST_NAMES = [
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf",
]
LEG_FIRST_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]

# Default joint positions per joint type.
HIP_DEFAULT = 0.0
THIGH_DEFAULT = 0.8
CALF_DEFAULT = -1.5

# Build default arrays for each ordering.
TYPE_FIRST_DEFAULTS = np.array([
    HIP_DEFAULT] * 4 + [THIGH_DEFAULT] * 4 + [CALF_DEFAULT] * 4,
    dtype=np.float32,
)
LEG_FIRST_DEFAULTS = np.array([
    HIP_DEFAULT, THIGH_DEFAULT, CALF_DEFAULT,  # FL
    HIP_DEFAULT, THIGH_DEFAULT, CALF_DEFAULT,  # FR
    HIP_DEFAULT, THIGH_DEFAULT, CALF_DEFAULT,  # RL
    HIP_DEFAULT, THIGH_DEFAULT, CALF_DEFAULT,  # RR
], dtype=np.float32)

# Permutation: type-first index -> leg-first index.
# type-first: [FL_h, FR_h, RL_h, RR_h, FL_t, FR_t, RL_t, RR_t, FL_c, FR_c, RL_c, RR_c]
# leg-first:  [FL_h, FL_t, FL_c, FR_h, FR_t, FR_c, RL_h, RL_t, RL_c, RR_h, RR_t, RR_c]
TYPE_TO_LEG = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
LEG_TO_TYPE = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]


def make_standing_obs(base_height=0.27, ordering="type_first"):
    """Create a realistic standing observation (49-dim)."""
    obs = np.zeros(49, dtype=np.float32)
    obs[0:3] = [0.0, 0.0, 0.0]       # base_lin_vel
    obs[3:6] = [0.0, 0.0, 0.0]       # base_ang_vel
    obs[6] = base_height               # base_height
    obs[7:10] = [0.0, 0.0, -1.0]      # projected_gravity (inverse)
    obs[10:13] = [0.0, 0.0, 0.0]      # velocity_commands
    obs[13:25] = 0.0                   # joint_pos (default-subtracted = 0)
    obs[25:37] = 0.0                   # joint_vel
    obs[37:49] = 0.0                   # last_actions
    return obs


def reorder_joints_in_obs(obs, from_order, to_order):
    """Reorder joint_pos [13:25], joint_vel [25:37], last_actions [37:49]."""
    obs_new = obs.copy()
    if from_order == "type_first" and to_order == "leg_first":
        perm = TYPE_TO_LEG
    elif from_order == "leg_first" and to_order == "type_first":
        perm = LEG_TO_TYPE
    else:
        return obs_new
    obs_new[13:25] = obs[13:25][perm]
    obs_new[25:37] = obs[25:37][perm]
    obs_new[37:49] = obs[37:49][perm]
    return obs_new


def inspect_graph(model_path):
    """Check ONNX graph for normalization ops."""
    if not HAS_ONNX:
        print("  [onnx package not installed — skipping graph inspection]")
        return None, None

    model = onnx.load(model_path)
    graph = model.graph
    input_name = graph.input[0].name

    # Find the first few ops that consume the input.
    print(f"  Graph: {len(graph.node)} nodes, {len(graph.initializer)} initializers")
    print(f"  Input: {input_name}")

    # Map initializer names to arrays for extraction.
    init_map = {}
    for init in graph.initializer:
        init_map[init.name] = numpy_helper.to_array(init)

    # Trace ops from input.
    current_outputs = {input_name}
    norm_mean = None
    norm_std = None
    first_ops = []

    for node in graph.node[:10]:
        if any(inp in current_outputs for inp in node.input):
            first_ops.append(node)
            current_outputs.update(node.output)

    print(f"\n  First ops from input:")
    for i, node in enumerate(first_ops[:8]):
        const_inputs = []
        for inp in node.input:
            if inp in init_map:
                arr = init_map[inp]
                const_inputs.append(
                    f"{inp}: shape={arr.shape} range=[{arr.min():.4f}, {arr.max():.4f}]"
                )
        print(f"    [{i}] {node.op_type} inputs={list(node.input)} -> {list(node.output)}")
        for ci in const_inputs:
            print(f"         const: {ci}")

    # Check for Sub (mean) followed by Div (std) or Mul (1/std) pattern.
    for i, node in enumerate(first_ops):
        if node.op_type == "Sub" and i + 1 < len(first_ops):
            next_node = first_ops[i + 1]
            # Get the constant being subtracted (the mean).
            for inp in node.input:
                if inp in init_map and init_map[inp].size == 49:
                    norm_mean = init_map[inp].flatten().astype(np.float32)
                    print(f"\n  FOUND normalization mean: {inp} shape={norm_mean.shape}")
            if next_node.op_type in ("Div", "Mul"):
                for inp in next_node.input:
                    if inp in init_map and init_map[inp].size == 49:
                        arr = init_map[inp].flatten().astype(np.float32)
                        if next_node.op_type == "Div":
                            norm_std = arr
                            print(f"  FOUND normalization std: {inp} shape={norm_std.shape}")
                        else:
                            # Mul by 1/std
                            norm_std = 1.0 / (arr + 1e-8)
                            print(f"  FOUND normalization 1/std: {inp} shape={norm_std.shape}")
            break

        # Also check for combined pattern: (input - mean) * (1/std)
        if node.op_type == "Mul":
            for inp in node.input:
                if inp in init_map and init_map[inp].size == 49:
                    arr = init_map[inp].flatten().astype(np.float32)
                    if arr.min() > 0:  # 1/std is always positive
                        norm_std = 1.0 / (arr + 1e-8)
                        print(f"\n  FOUND possible 1/std (Mul): {inp}")

    # Also check for Clip after normalization.
    for node in first_ops:
        if node.op_type == "Clip":
            print(f"  FOUND Clip node (likely obs clip to [-5, 5])")

    return norm_mean, norm_std


def run_inference(session, obs):
    """Run single inference."""
    inp_name = session.get_inputs()[0].name
    return session.run(None, {inp_name: obs.reshape(1, -1)})[0].flatten()


def test_standing(session):
    """Test B: Standing with zero commands, both orderings."""
    print("\n  Standing + zero cmd (base_height=0.27, gravity=[0,0,-1]):")
    obs = make_standing_obs(base_height=0.27)
    out_tf = run_inference(session, obs)

    # For leg-first, the joint slices need reordering.
    obs_lf = reorder_joints_in_obs(obs, "type_first", "leg_first")
    out_lf = run_inference(session, obs_lf)

    max_tf = np.max(np.abs(out_tf))
    max_lf = np.max(np.abs(out_lf))
    l2_tf = np.linalg.norm(out_tf)
    l2_lf = np.linalg.norm(out_lf)

    print(f"    TYPE-FIRST:  max|a|={max_tf:.4f}, L2={l2_tf:.4f}, actions={np.array2string(out_tf, precision=3)}")
    print(f"    LEG-FIRST:   max|a|={max_lf:.4f}, L2={l2_lf:.4f}, actions={np.array2string(out_lf, precision=3)}")

    return out_tf, out_lf


def test_forward_cmd(session):
    """Test C: Forward velocity command, both orderings."""
    print("\n  Standing + cmd_vx=0.5:")
    obs = make_standing_obs(base_height=0.27)
    obs[10] = 0.5  # cmd_vx

    out_tf = run_inference(session, obs)
    obs_lf = reorder_joints_in_obs(obs, "type_first", "leg_first")
    out_lf = run_inference(session, obs_lf)

    max_tf = np.max(np.abs(out_tf))
    max_lf = np.max(np.abs(out_lf))

    # Check left/right symmetry for straight-line walking.
    # Type-first: FL=0, FR=1, RL=2, RR=3 (for each joint type group of 4)
    # Symmetric walk: FL≈FR, RL≈RR
    def symmetry_score_tf(a):
        # In type-first: [FL_h FR_h RL_h RR_h FL_t FR_t RL_t RR_t FL_c FR_c RL_c RR_c]
        pairs = [(0, 1), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11)]
        return np.mean([abs(a[i] - a[j]) for i, j in pairs])

    def symmetry_score_lf(a):
        # In leg-first: [FL_h FL_t FL_c FR_h FR_t FR_c RL_h RL_t RL_c RR_h RR_t RR_c]
        pairs = [(0, 3), (1, 4), (2, 5), (6, 9), (7, 10), (8, 11)]
        return np.mean([abs(a[i] - a[j]) for i, j in pairs])

    sym_tf = symmetry_score_tf(out_tf)
    sym_lf = symmetry_score_lf(out_lf)

    print(f"    TYPE-FIRST:  max|a|={max_tf:.4f}, L/R_asymmetry={sym_tf:.4f}")
    print(f"    LEG-FIRST:   max|a|={max_lf:.4f}, L/R_asymmetry={sym_lf:.4f}")
    print(f"    TYPE-FIRST actions: {np.array2string(out_tf, precision=3)}")
    print(f"    LEG-FIRST  actions: {np.array2string(out_lf, precision=3)}")

    return out_tf, out_lf


def test_perturbation(session):
    """Test D: Single-joint perturbation sweep."""
    print("\n  Single-joint perturbation (+0.3 rad in joint_pos):")
    base_obs = make_standing_obs(base_height=0.27)
    base_out = run_inference(session, base_obs)

    print(f"    {'Perturbed joint':>20s} | {'Max responding action idx':>25s} | {'Response magnitude':>18s}")
    print(f"    {'-'*20} | {'-'*25} | {'-'*18}")

    for j in range(12):
        obs = base_obs.copy()
        obs[13 + j] += 0.3  # Perturb one joint position
        out = run_inference(session, obs)
        delta = out - base_out
        max_idx = np.argmax(np.abs(delta))
        max_val = delta[max_idx]
        print(f"    {TYPE_FIRST_NAMES[j]:>20s} | {max_idx:>2d} ({TYPE_FIRST_NAMES[max_idx]}){'':<6s} | {max_val:+.4f}")


def test_feedback_loop(session, ordering="type_first", cmd_vx=0.0, steps=20):
    """Test E: Simulate feedback loop."""
    label = ordering.upper().replace("_", "-")
    print(f"\n  Feedback loop ({label}, cmd_vx={cmd_vx}, {steps} steps):")
    obs = make_standing_obs(base_height=0.27)
    obs[10] = cmd_vx

    if ordering == "leg_first":
        obs = reorder_joints_in_obs(obs, "type_first", "leg_first")

    for step in range(steps):
        out = run_inference(session, obs)
        clipped = np.clip(out, -1.0, 1.0)
        max_abs = np.max(np.abs(out))
        pct_rail = 100.0 * np.mean(np.abs(out) > 0.95)
        if step < 5 or step == steps - 1 or step % 5 == 0:
            print(f"    step {step:2d}: max|a|={max_abs:.4f}, %railing={pct_rail:.0f}%, "
                  f"actions={np.array2string(clipped, precision=2, suppress_small=True)}")
        # Feed clipped actions back as last_actions.
        obs[37:49] = clipped


def test_feedback_loop_with_real_obs(session, npz_path, steps=20):
    """Test F: Use real recorded observation as starting point."""
    if npz_path is None or not os.path.isfile(npz_path):
        return

    d = dict(np.load(npz_path, allow_pickle=False))
    if "obs_data" not in d:
        return

    print(f"\n  Feedback loop with REAL initial obs from {os.path.basename(npz_path)}:")
    real_obs = d["obs_data"][0].copy()  # First recorded observation.
    real_obs[37:49] = 0.0  # Reset last_actions to zero.

    # Test with type-first (as recorded).
    print("    TYPE-FIRST (as recorded):")
    obs = real_obs.copy()
    for step in range(steps):
        out = run_inference(session, obs)
        clipped = np.clip(out, -1.0, 1.0)
        max_abs = np.max(np.abs(out))
        if step < 3 or step == steps - 1:
            print(f"      step {step:2d}: max|a|={max_abs:.4f}, "
                  f"actions={np.array2string(clipped, precision=2, suppress_small=True)}")
        obs[37:49] = clipped

    # Test with leg-first reordering.
    print("    LEG-FIRST (reordered):")
    obs = reorder_joints_in_obs(real_obs, "type_first", "leg_first")
    for step in range(steps):
        out = run_inference(session, obs)
        clipped = np.clip(out, -1.0, 1.0)
        max_abs = np.max(np.abs(out))
        if step < 3 or step == steps - 1:
            print(f"      step {step:2d}: max|a|={max_abs:.4f}, "
                  f"actions={np.array2string(clipped, precision=2, suppress_small=True)}")
        obs[37:49] = clipped


def main():
    parser = argparse.ArgumentParser(description="Probe ONNX locomotion policy.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="Path to ONNX model")
    parser.add_argument("--npz", default=None, help="Path to recorded .npz for real-obs test")
    args = parser.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        sys.exit(f"Model not found: {model_path}")

    # Auto-detect latest npz if not provided.
    if args.npz is None:
        import glob
        npz_files = sorted(glob.glob(
            os.path.join(os.path.dirname(__file__), "..", "locomotion_diag_*.npz")
        ))
        if npz_files:
            args.npz = npz_files[-1]

    print(f"Model: {model_path}")
    print(f"NPZ:   {args.npz or 'none'}")

    # --- Test A: Graph Inspection ---
    print("\n" + "=" * 72)
    print("TEST A: ONNX Graph Inspection (normalization check)")
    print("=" * 72)
    norm_mean, norm_std = inspect_graph(model_path)
    if norm_mean is not None:
        print(f"\n  Normalization MEAN (49-dim):")
        print(f"    {np.array2string(norm_mean, precision=4, suppress_small=True)}")
    if norm_std is not None:
        print(f"  Normalization STD (49-dim):")
        print(f"    {np.array2string(norm_std, precision=4, suppress_small=True)}")
    if norm_mean is None and norm_std is None:
        print("\n  No normalization pattern detected in graph.")
        print("  (Normalization may be embedded as regular ops or truly absent.)")

    # Load session for inference tests.
    session = ort.InferenceSession(model_path)
    inp = session.get_inputs()[0]
    out = session.get_outputs()[0]
    print(f"\n  Runtime: input={inp.name} shape={inp.shape}, output={out.name} shape={out.shape}")

    # --- Test B: Standing ---
    print("\n" + "=" * 72)
    print("TEST B: Standing + Zero Command")
    print("=" * 72)
    out_tf_stand, out_lf_stand = test_standing(session)

    # --- Test C: Forward Command ---
    print("\n" + "=" * 72)
    print("TEST C: Standing + Forward Command (cmd_vx=0.5)")
    print("=" * 72)
    out_tf_fwd, out_lf_fwd = test_forward_cmd(session)

    # --- Test D: Perturbation ---
    print("\n" + "=" * 72)
    print("TEST D: Single-Joint Perturbation Sweep")
    print("=" * 72)
    test_perturbation(session)

    # --- Test E: Feedback Loop ---
    print("\n" + "=" * 72)
    print("TEST E: Feedback Loop Simulation (20 steps)")
    print("=" * 72)
    test_feedback_loop(session, "type_first", cmd_vx=0.0, steps=20)
    test_feedback_loop(session, "leg_first", cmd_vx=0.0, steps=20)
    test_feedback_loop(session, "type_first", cmd_vx=0.5, steps=20)
    test_feedback_loop(session, "leg_first", cmd_vx=0.5, steps=20)

    # --- Test F: Real Observation ---
    print("\n" + "=" * 72)
    print("TEST F: Feedback Loop with Real Initial Observation")
    print("=" * 72)
    test_feedback_loop_with_real_obs(session, args.npz, steps=20)

    # --- Verdict ---
    print("\n" + "=" * 72)
    print("VERDICT")
    print("=" * 72)

    # Compare standing outputs.
    max_tf = np.max(np.abs(out_tf_stand))
    max_lf = np.max(np.abs(out_lf_stand))
    print(f"\n  Standing max|a|: TYPE-FIRST={max_tf:.4f}, LEG-FIRST={max_lf:.4f}")
    if max_tf < max_lf * 0.7:
        print("  -> TYPE-FIRST produces smaller standing actions (likely correct ordering)")
        ordering_verdict = "TYPE-FIRST"
    elif max_lf < max_tf * 0.7:
        print("  -> LEG-FIRST produces smaller standing actions (likely correct ordering)")
        ordering_verdict = "LEG-FIRST"
    else:
        print("  -> Inconclusive from standing test alone (both similar magnitude)")
        ordering_verdict = "INCONCLUSIVE"

    # Check normalization.
    if norm_mean is not None or norm_std is not None:
        print(f"\n  NORMALIZATION: Detected in ONNX graph (baked-in)")
        norm_verdict = "BAKED-IN"
    else:
        if max_tf > 0.8 and max_lf > 0.8:
            print(f"\n  NORMALIZATION: Both orderings produce large standing actions.")
            print(f"  This could indicate missing normalization OR the policy has a non-zero standing output.")
            norm_verdict = "POSSIBLY MISSING"
        else:
            print(f"\n  NORMALIZATION: Standing outputs are reasonable — likely baked-in or not needed.")
            norm_verdict = "LIKELY OK"

    print(f"\n  JOINT ORDERING: {ordering_verdict}")
    print(f"  NORMALIZATION:  {norm_verdict}")
    print("=" * 72)


if __name__ == "__main__":
    main()
