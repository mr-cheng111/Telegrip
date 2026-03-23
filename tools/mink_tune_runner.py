#!/usr/bin/env python3
"""
Mink 参数调参与评估工具。

功能：
1) evaluate: 运行仿真，生成随机目标点，执行 Mink 跟踪，记录轨迹并计算评分。
2) search: 自动探索参数（支持 random / llm 策略），输出最优参数与每轮结果。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from telegrip.core.mink_kinematics import MinkIKSolver
from telegrip.core.visualizer import MuJoCoVisualizer


@dataclass
class TunableParams:
    """可调参数集合。"""

    position_cost: float = 1.0
    orientation_cost: float = 5.0
    posture_cost: float = 0.01
    lm_damping: float = 1.0
    solve_damping: float = 1e-3
    ik_dt: float = 0.005
    max_iters: int = 20
    pos_threshold: float = 1e-4
    ori_threshold: float = 1e-3
    kp_scale: float = 1.0
    kd_scale: float = 1.0


@dataclass
class EvalConfig:
    """评估配置。"""

    arm: str = "left"
    seed: int = 42
    episodes: int = 4
    episode_seconds: float = 5.0
    control_hz: float = 200.0
    target_hold_seconds: float = 0.5
    target_radius: float = 0.10
    z_min: float = 0.10
    z_max: float = 1.00


_PARAM_BOUNDS: dict[str, tuple[float, float, bool]] = {
    # (min, max, is_int)
    "position_cost": (0.3, 8.0, False),
    "orientation_cost": (0.3, 10.0, False),
    "posture_cost": (1e-4, 1e-1, False),
    "lm_damping": (0.2, 5.0, False),
    "solve_damping": (1e-5, 1e-1, False),
    "ik_dt": (0.001, 0.02, False),
    "max_iters": (6, 60, True),
    "pos_threshold": (1e-5, 1e-3, False),
    "ori_threshold": (1e-4, 1e-2, False),
    "kp_scale": (0.6, 1.8, False),
    "kd_scale": (0.6, 1.8, False),
}


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_scene_xml() -> Path:
    """生成并返回统一基座双臂场景 XML。"""
    root = _project_root()
    examples_dir = root / "mink" / "examples"
    output = root / "mink" / "examples" / "arm620" / "_tmp_dual_unified_saved.xml"

    import sys

    if str(examples_dir) not in sys.path:
        sys.path.insert(0, str(examples_dir))

    from arm620.scene_dual_unified_base_builder import save_dual_unified_xml

    output.parent.mkdir(parents=True, exist_ok=True)
    save_dual_unified_xml(output)
    return output


def _joint_names_for_arm(arm: str) -> list[str]:
    return [f"{arm}/joint{i}" for i in range(1, 7)]


def _sample_target(rng: np.random.Generator, center: np.ndarray, radius: float, z_min: float, z_max: float) -> np.ndarray:
    """在球体内均匀采样目标点。"""
    while True:
        delta = rng.uniform(-radius, radius, size=3)
        if np.linalg.norm(delta) <= radius:
            target = center + delta
            target[2] = float(np.clip(target[2], z_min, z_max))
            return target


def _ema_filter(x: np.ndarray, alpha: float = 0.15) -> np.ndarray:
    """一阶 EMA 低通滤波。"""
    if x.size == 0:
        return x
    y = np.zeros_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def _rms(x: np.ndarray) -> float:
    if x.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(x))))


def _compute_metrics(rows: list[dict[str, Any]], dt: float) -> dict[str, float]:
    if not rows:
        raise ValueError("没有采样数据，无法计算指标")

    err = np.array([r["err_pos"] for r in rows], dtype=float)
    qvel = np.array([r["qvel_norm"] for r in rows], dtype=float)
    tau = np.array([r["tau_norm"] for r in rows], dtype=float)

    ee = np.array([[r["ee_x"], r["ee_y"], r["ee_z"]] for r in rows], dtype=float)
    ee_speed = np.linalg.norm(np.diff(ee, axis=0), axis=1) / dt if len(ee) > 1 else np.zeros(1, dtype=float)

    # 收敛速度公式: v_track(t) = -(e_t - e_{t-1}) / dt
    # 公式来源: 对误差 e(t) 做一阶差分近似导数，并取负号表示“收敛”为正值。
    derr = np.diff(err) / dt if len(err) > 1 else np.zeros(1, dtype=float)
    track_speed = np.maximum(0.0, -derr)

    # 抖动定义（高频分量 RMS）:
    # jitter = RMS(x - LPF(x))
    # 其中 LPF 采用一阶 EMA: y_t = alpha * x_t + (1-alpha) * y_{t-1}。
    # 公式来源: 控制领域常见“信号 = 低频趋势 + 高频扰动”分解。
    qvel_lpf = _ema_filter(qvel)
    tau_lpf = _ema_filter(tau)
    qvel_jitter = _rms(qvel - qvel_lpf)
    tau_jitter = _rms(tau - tau_lpf)

    rmse_pos = _rms(err)
    p95_pos = float(np.percentile(err, 95))
    mean_track_speed = float(np.mean(track_speed))
    mean_ee_speed = float(np.mean(ee_speed))

    # 目标函数（越小越好）:
    # J = w1*RMSE_mm + w2*P95_mm + w3*jitter_tau + w4*jitter_qvel - w5*track_speed
    # 公式来源: 多目标加权和，位置误差与抖动惩罚，收敛速度作为奖励项。
    rmse_mm = rmse_pos * 1000.0
    p95_mm = p95_pos * 1000.0
    objective = (
        1.0 * rmse_mm
        + 0.6 * p95_mm
        + 0.25 * tau_jitter
        + 0.25 * qvel_jitter
        - 0.10 * mean_track_speed
    )
    objective = float(max(objective, 1e-6))

    # 归一化得分（越高越好）:
    # score = 100 / (1 + J)
    score = float(100.0 / (1.0 + objective))

    return {
        "rmse_pos_m": rmse_pos,
        "p95_pos_m": p95_pos,
        "mean_track_speed_mps": mean_track_speed,
        "mean_ee_speed_mps": mean_ee_speed,
        "qvel_jitter": qvel_jitter,
        "tau_jitter": tau_jitter,
        "objective": objective,
        "score": score,
    }


def _sanitize_params(values: dict[str, Any]) -> TunableParams:
    """对 LLM / 外部输入做边界裁剪，保证参数可执行。"""
    sanitized: dict[str, Any] = {}
    for k, (lo, hi, is_int) in _PARAM_BOUNDS.items():
        v = float(values.get(k, getattr(TunableParams(), k)))
        v = min(max(v, lo), hi)
        if is_int:
            sanitized[k] = int(round(v))
        else:
            sanitized[k] = float(v)
    return TunableParams(**sanitized)


def run_evaluation(
    params: TunableParams,
    cfg: EvalConfig,
    scene_xml: Path,
    out_dir: Path,
    tag: str,
    write_trajectory: bool = True,
    use_gui: bool = False,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)

    visualizer = MuJoCoVisualizer(str(scene_xml), use_gui=bool(use_gui), log_level="warning")
    if not visualizer.setup():
        raise RuntimeError("MuJoCo visualizer 初始化失败")

    try:
        visualizer.kp = visualizer.kp * float(params.kp_scale)
        visualizer.kd = visualizer.kd * float(params.kd_scale)

        arm = cfg.arm
        joint_names = _joint_names_for_arm(arm)
        ee_site = f"{arm}/tools_link"
        ik_solver = MinkIKSolver(
            xml_path=str(scene_xml),
            end_effector_site=ee_site,
            joint_names=joint_names,
            num_ik_joints=6,
            position_cost=params.position_cost,
            orientation_cost=params.orientation_cost,
            posture_cost=params.posture_cost,
            lm_damping=params.lm_damping,
            solve_damping=params.solve_damping,
            dt=params.ik_dt,
            max_iters=params.max_iters,
            pos_threshold=params.pos_threshold,
            ori_threshold=params.ori_threshold,
        )

        dt = 1.0 / cfg.control_hz
        model_dt = float(visualizer.model.opt.timestep)
        substeps = max(1, int(round(dt / model_dt))) if model_dt > 0.0 else 1

        ee0 = visualizer.get_end_effector_position(arm)
        if ee0 is None:
            raise RuntimeError(f"无法读取 {arm} 臂末端位置")
        target = ee0.copy()

        rng = np.random.default_rng(cfg.seed)

        steps_per_episode = max(1, int(round(cfg.episode_seconds * cfg.control_hz)))
        hold_steps = max(1, int(round(cfg.target_hold_seconds * cfg.control_hz)))

        rows: list[dict[str, Any]] = []

        step_idx = 0
        for _ep in range(cfg.episodes):
            center = ee0.copy()
            for _k in range(steps_per_episode):
                if step_idx % hold_steps == 0:
                    target = _sample_target(rng, center, cfg.target_radius, cfg.z_min, cfg.z_max)
                    visualizer.update_marker_position(f"{arm}_target", target)

                current_deg = visualizer.get_joint_angles_deg(arm)
                if current_deg is None:
                    raise RuntimeError(f"无法读取 {arm} 臂关节角")

                q_des_deg = ik_solver.solve(
                    target_position=target,
                    current_angles_deg=current_deg,
                )
                visualizer.update_robot_pose(q_des_deg, arm=arm)
                visualizer.step_simulation(substeps=substeps)

                with visualizer._mujoco_lock:
                    qidx = visualizer.joint_qpos_indices[arm]
                    didx = visualizer.joint_dof_indices[arm]
                    aidx = visualizer.actuator_indices[arm]
                    if qidx is None or didx is None or aidx is None:
                        raise RuntimeError(f"场景中未解析到 {arm} 臂关节/驱动索引")

                    ee_sid = visualizer.left_ee_site_id if arm == "left" else visualizer.right_ee_site_id
                    ee_pos = visualizer.data.site_xpos[ee_sid].copy()
                    q_deg = np.rad2deg(visualizer.data.qpos[qidx]).copy()
                    qvel = visualizer.data.qvel[didx].copy()
                    tau = visualizer.data.ctrl[aidx].copy()
                    sim_t = float(visualizer.data.time)

                err_pos = float(np.linalg.norm(target - ee_pos))
                rows.append(
                    {
                        "step": step_idx,
                        "sim_time": sim_t,
                        "target_x": float(target[0]),
                        "target_y": float(target[1]),
                        "target_z": float(target[2]),
                        "ee_x": float(ee_pos[0]),
                        "ee_y": float(ee_pos[1]),
                        "ee_z": float(ee_pos[2]),
                        "err_pos": err_pos,
                        "qvel_norm": float(np.linalg.norm(qvel)),
                        "tau_norm": float(np.linalg.norm(tau)),
                        "q1_deg": float(q_deg[0]),
                        "q2_deg": float(q_deg[1]),
                        "q3_deg": float(q_deg[2]),
                        "q4_deg": float(q_deg[3]),
                        "q5_deg": float(q_deg[4]),
                        "q6_deg": float(q_deg[5]),
                    }
                )
                step_idx += 1

        metrics = _compute_metrics(rows, dt=dt)
        result = {
            "tag": tag,
            "scene_xml": str(scene_xml),
            "arm": cfg.arm,
            "seed": cfg.seed,
            "use_gui": bool(use_gui),
            "eval_config": asdict(cfg),
            "params": asdict(params),
            "metrics": metrics,
            "timestamp": int(time.time()),
        }

        if write_trajectory:
            csv_path = out_dir / f"trajectory_{tag}.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            result["trajectory_csv"] = str(csv_path)

        json_path = out_dir / f"result_{tag}.json"
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        result["result_json"] = str(json_path)

        return result
    finally:
        visualizer.disconnect()


def _sample_params(rng: np.random.Generator) -> TunableParams:
    """随机采样一组参数（含对数采样）。"""
    return TunableParams(
        position_cost=float(rng.uniform(0.3, 8.0)),
        orientation_cost=float(rng.uniform(0.3, 10.0)),
        posture_cost=float(10 ** rng.uniform(-4.0, -1.0)),
        lm_damping=float(rng.uniform(0.2, 5.0)),
        solve_damping=float(10 ** rng.uniform(-5.0, -1.0)),
        ik_dt=float(rng.uniform(0.001, 0.02)),
        max_iters=int(rng.integers(6, 61)),
        pos_threshold=float(10 ** rng.uniform(-5.0, -3.0)),
        ori_threshold=float(10 ** rng.uniform(-4.0, -2.0)),
        kp_scale=float(rng.uniform(0.6, 1.8)),
        kd_scale=float(rng.uniform(0.6, 1.8)),
    )


def _extract_json_object(text: str) -> dict[str, Any]:
    text = text.strip()
    try:
        return json.loads(text)
    except Exception:
        pass

    match = re.search(r"\{[\s\S]*\}", text)
    if not match:
        raise ValueError("LLM 输出中没有检测到 JSON 对象")
    return json.loads(match.group(0))


def _llm_suggest_params(
    history: list[dict[str, Any]],
    model: str,
    api_key_env: str,
) -> TunableParams:
    """调用 OpenAI 生成下一组待评估参数。"""
    try:
        from openai import OpenAI
    except Exception as exc:
        raise RuntimeError("未安装 openai 包，请先执行: pip install openai") from exc

    api_key = os.getenv(api_key_env)
    if not api_key:
        raise RuntimeError(f"环境变量 {api_key_env} 未设置，无法调用 LLM")

    client = OpenAI(api_key=api_key)

    compact_history = history[-12:]
    hist_payload = [
        {
            "trial": h["trial"],
            "objective": h["objective"],
            "score": h["score"],
            "params": h["params"],
        }
        for h in compact_history
    ]

    bounds_desc = {
        k: {"min": lo, "max": hi, "int": is_int}
        for k, (lo, hi, is_int) in _PARAM_BOUNDS.items()
    }

    system_prompt = (
        "你是机器人控制参数优化器。目标是最小化 objective。"
        "只输出 JSON 对象，不要输出 markdown。"
    )
    user_prompt = {
        "task": "根据历史试验结果，提出下一组 Mink+PD 参数",
        "goal": "minimize objective",
        "bounds": bounds_desc,
        "history": hist_payload,
        "output_schema": {
            "position_cost": "float",
            "orientation_cost": "float",
            "posture_cost": "float",
            "lm_damping": "float",
            "solve_damping": "float",
            "ik_dt": "float",
            "max_iters": "int",
            "pos_threshold": "float",
            "ori_threshold": "float",
            "kp_scale": "float",
            "kd_scale": "float",
        },
    }

    resp = client.chat.completions.create(
        model=model,
        temperature=0.2,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(user_prompt, ensure_ascii=False)},
        ],
    )

    content = resp.choices[0].message.content or ""
    parsed = _extract_json_object(content)
    return _sanitize_params(parsed)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Mink 参数评估与自动搜参")
    sub = parser.add_subparsers(dest="command", required=True)

    p_eval = sub.add_parser("evaluate", help="运行一次评估")
    p_eval.add_argument("--scene", type=str, default="auto", help="MuJoCo 场景 XML，默认 auto(自动生成统一双臂场景)")
    p_eval.add_argument("--arm", type=str, default="left", choices=["left", "right"], help="评估哪只手臂")
    p_eval.add_argument("--seed", type=int, default=42)
    p_eval.add_argument("--episodes", type=int, default=4)
    p_eval.add_argument("--episode-seconds", type=float, default=5.0)
    p_eval.add_argument("--control-hz", type=float, default=200.0)
    p_eval.add_argument("--target-hold-seconds", type=float, default=0.5)
    p_eval.add_argument("--target-radius", type=float, default=0.10)
    p_eval.add_argument("--z-min", type=float, default=0.10)
    p_eval.add_argument("--z-max", type=float, default=1.00)
    p_eval.add_argument("--out", type=str, default="runs/mink_tuning")
    p_eval.add_argument("--tag", type=str, default="eval")
    p_eval.add_argument("--gui", action="store_true", help="启用 MuJoCo GUI 可视化")

    p_eval.add_argument("--position-cost", type=float, default=1.0)
    p_eval.add_argument("--orientation-cost", type=float, default=5.0)
    p_eval.add_argument("--posture-cost", type=float, default=0.01)
    p_eval.add_argument("--lm-damping", type=float, default=1.0)
    p_eval.add_argument("--solve-damping", type=float, default=1e-3)
    p_eval.add_argument("--ik-dt", type=float, default=0.005)
    p_eval.add_argument("--max-iters", type=int, default=20)
    p_eval.add_argument("--pos-threshold", type=float, default=1e-4)
    p_eval.add_argument("--ori-threshold", type=float, default=1e-3)
    p_eval.add_argument("--kp-scale", type=float, default=1.0)
    p_eval.add_argument("--kd-scale", type=float, default=1.0)

    p_search = sub.add_parser("search", help="自动搜索最优参数")
    p_search.add_argument("--scene", type=str, default="auto")
    p_search.add_argument("--arm", type=str, default="left", choices=["left", "right"])
    p_search.add_argument("--seed", type=int, default=123)
    p_search.add_argument("--trials", type=int, default=20)
    p_search.add_argument("--episodes", type=int, default=3)
    p_search.add_argument("--episode-seconds", type=float, default=4.0)
    p_search.add_argument("--control-hz", type=float, default=200.0)
    p_search.add_argument("--target-hold-seconds", type=float, default=0.5)
    p_search.add_argument("--target-radius", type=float, default=0.10)
    p_search.add_argument("--z-min", type=float, default=0.10)
    p_search.add_argument("--z-max", type=float, default=1.00)
    p_search.add_argument("--out", type=str, default="runs/mink_tuning")
    p_search.add_argument("--gui", action="store_true", help="启用 MuJoCo GUI 可视化")
    p_search.add_argument("--strategy", type=str, default="llm", choices=["llm", "random"], help="参数提议策略")
    p_search.add_argument("--model", type=str, default="gpt-5-mini", help="OpenAI 模型名")
    p_search.add_argument("--api-key-env", type=str, default="OPENAI_API_KEY", help="存放 OpenAI API Key 的环境变量")

    return parser


def _resolve_scene(scene_arg: str) -> Path:
    if scene_arg == "auto":
        return _default_scene_xml()
    return Path(scene_arg).expanduser().resolve()


def _print_result_brief(result: dict[str, Any]) -> None:
    m = result["metrics"]
    print(
        f"[{result['tag']}] score={m['score']:.4f}, objective={m['objective']:.4f}, "
        f"rmse={m['rmse_pos_m'] * 1000:.2f}mm, p95={m['p95_pos_m'] * 1000:.2f}mm, "
        f"tau_jitter={m['tau_jitter']:.4f}, qvel_jitter={m['qvel_jitter']:.4f}"
    )


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    out_dir = Path(args.out).expanduser().resolve()
    scene_xml = _resolve_scene(args.scene)

    if not scene_xml.exists():
        raise FileNotFoundError(f"场景文件不存在: {scene_xml}")

    cfg = EvalConfig(
        arm=args.arm,
        seed=int(args.seed),
        episodes=int(args.episodes),
        episode_seconds=float(args.episode_seconds),
        control_hz=float(args.control_hz),
        target_hold_seconds=float(args.target_hold_seconds),
        target_radius=float(args.target_radius),
        z_min=float(args.z_min),
        z_max=float(args.z_max),
    )

    if args.command == "evaluate":
        params = TunableParams(
            position_cost=float(args.position_cost),
            orientation_cost=float(args.orientation_cost),
            posture_cost=float(args.posture_cost),
            lm_damping=float(args.lm_damping),
            solve_damping=float(args.solve_damping),
            ik_dt=float(args.ik_dt),
            max_iters=int(args.max_iters),
            pos_threshold=float(args.pos_threshold),
            ori_threshold=float(args.ori_threshold),
            kp_scale=float(args.kp_scale),
            kd_scale=float(args.kd_scale),
        )
        result = run_evaluation(
            params=params,
            cfg=cfg,
            scene_xml=scene_xml,
            out_dir=out_dir,
            tag=args.tag,
            write_trajectory=True,
            use_gui=bool(args.gui),
        )
        _print_result_brief(result)
        print(f"result_json={result['result_json']}")
        print(f"trajectory_csv={result.get('trajectory_csv', '')}")
        return

    rng = np.random.default_rng(args.seed)
    best: dict[str, Any] | None = None
    summary_rows: list[dict[str, Any]] = []

    baseline = TunableParams()
    baseline_result = run_evaluation(
        params=baseline,
        cfg=cfg,
        scene_xml=scene_xml,
        out_dir=out_dir,
        tag="trial_000_baseline",
        write_trajectory=False,
        use_gui=bool(args.gui),
    )
    _print_result_brief(baseline_result)
    summary_rows.append(
        {
            "trial": 0,
            "tag": baseline_result["tag"],
            "strategy": "baseline",
            "score": baseline_result["metrics"]["score"],
            "objective": baseline_result["metrics"]["objective"],
            "result_json": baseline_result["result_json"],
            "params": json.dumps(baseline_result["params"], ensure_ascii=False),
        }
    )
    best = baseline_result

    trials = int(args.trials)
    for i in range(1, trials + 1):
        if args.strategy == "llm":
            history_for_llm = [
                {
                    "trial": r["trial"],
                    "score": r["score"],
                    "objective": r["objective"],
                    "params": json.loads(r["params"]),
                }
                for r in summary_rows
            ]
            params = _llm_suggest_params(
                history=history_for_llm,
                model=args.model,
                api_key_env=args.api_key_env,
            )
        else:
            params = _sample_params(rng)

        tag = f"trial_{i:03d}"
        result = run_evaluation(
            params=params,
            cfg=cfg,
            scene_xml=scene_xml,
            out_dir=out_dir,
            tag=tag,
            write_trajectory=False,
            use_gui=bool(args.gui),
        )
        _print_result_brief(result)

        summary_rows.append(
            {
                "trial": i,
                "tag": tag,
                "strategy": args.strategy,
                "score": result["metrics"]["score"],
                "objective": result["metrics"]["objective"],
                "result_json": result["result_json"],
                "params": json.dumps(asdict(params), ensure_ascii=False),
            }
        )

        if result["metrics"]["objective"] < best["metrics"]["objective"]:
            best = result

    summary_csv = out_dir / "search_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    best_json = out_dir / "best_result.json"
    best_json.write_text(json.dumps(best, ensure_ascii=False, indent=2), encoding="utf-8")

    print("=" * 80)
    print("搜索完成")
    _print_result_brief(best)
    print(f"best_result={best_json}")
    print(f"summary_csv={summary_csv}")


if __name__ == "__main__":
    main()
