"""
水域块采样点 TSP/PAR 文件生成模块（通用）

本模块为单块水域采样点生成 LKH-3 所需的 TSP 与 PAR 文件，与 LKH-3 Path TSP 示例一致：
- 使用 Path TSP 的虚拟节点变换：增加节点 n，仅与起点 0、终点 1 以 0 成本相连，与其余点以 12345000 相连
- 输出 TSPLIB EXPLICIT FULL_MATRIX 格式的 TSP 文件
- PAR 使用 RUNS（可配置）, MAX_TRIALS=4005, MOVE_TYPE=5, MAX_CANDIDATES=50, TRACE_LEVEL=1
"""

import os
import numpy as np

# 与 LKH-3 可用的 Path TSP 示例一致：虚拟节点到非起/终点的边权
VIRTUAL_NODE_PENALTY = 12345000


def generate_water_tsp_files(
    cost_matrix: np.ndarray,
    sampling_points: list,
    output_dir: str,
    tsp_name: str = "sampling_points.tsp",
    par_name: str = "sampling_points.par",
    solution_name: str = "sampling_points.tour",
    lkh_runs: int = 10,
) -> tuple:
    """
    生成 Path TSP 的 TSP 与 PAR 文件（含虚拟节点）。

    :param cost_matrix: n×n 成本矩阵，索引 0 为起点，1 为终点
    :param sampling_points: 采样点列表（仅用于校验维度）
    :param output_dir: 输出目录
    :param tsp_name: TSP 文件名
    :param par_name: PAR 文件名
    :param solution_name: LKH 解文件名（.tour）
    :param lkh_runs: PAR 中的 RUNS（LKH 独立运行次数，越大解越优、越慢）
    :return: (success: bool, info: dict)，成功时 info 含 'tsp_file','par_file','solution_file'
    """
    n = cost_matrix.shape[0] if hasattr(cost_matrix, 'shape') else len(cost_matrix)
    if n != cost_matrix.shape[1]:
        return False, {}
    if n < 2:
        return False, {}
    if sampling_points is not None and len(sampling_points) != n:
        return False, {}

    # 原矩阵中 inf 用与虚拟节点一致的大整数，便于 LKH-3 与示例行为一致
    unreachable_cost = VIRTUAL_NODE_PENALTY

    # 构建 (n+1)×(n+1) 矩阵：前 n 行为原矩阵（inf→VIRTUAL_NODE_PENALTY），第 n 行为虚拟节点
    # 虚拟节点 n：与 0、1 的成本为 0，与 2..n-1 为 VIRTUAL_NODE_PENALTY；M[n,n]=0
    full = np.zeros((n + 1, n + 1), dtype=np.float64)
    for i in range(n):
        for j in range(n):
            v = cost_matrix[i, j]
            full[i, j] = int(round(v)) if np.isfinite(v) else unreachable_cost
    full[n, n] = 0
    for i in range(n):
        full[i, n] = 0 if i in (0, 1) else VIRTUAL_NODE_PENALTY
        full[n, i] = full[i, n]

    tsp_path = os.path.join(output_dir, tsp_name)
    par_path = os.path.join(output_dir, par_name)
    solution_path = os.path.join(output_dir, solution_name)

    try:
        os.makedirs(output_dir, exist_ok=True)
    except Exception as e:
        print(f"  ✗ 创建输出目录失败: {e}")
        return False, {}

    # 写 TSP
    try:
        with open(tsp_path, 'w', encoding='utf-8') as f:
            f.write(f"NAME: {tsp_name.replace('.tsp', '')}\n")
            f.write("TYPE: TSP\n")
            f.write("COMMENT: Water block sampling points with virtual node for Path TSP\n")
            f.write(f"DIMENSION: {n + 1}\n")
            f.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
            f.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
            f.write("EDGE_WEIGHT_SECTION\n")
            for i in range(n + 1):
                row = [str(int(round(full[i, j]))) for j in range(n + 1)]
                f.write(" ".join(row) + "\n")
            f.write("EOF\n")
    except Exception as e:
        print(f"  ✗ 保存TSP文件时出错: {e}")
        return False, {}

    # 写 PAR（与 LKH-3 可用的 Path TSP 示例一致）
    try:
        with open(par_path, 'w', encoding='utf-8') as f:
            f.write(f"PROBLEM_FILE = {tsp_name}\n")
            f.write(f"OUTPUT_TOUR_FILE = {solution_name}\n")
            f.write(f"RUNS = {lkh_runs}\n")
            f.write("MAX_TRIALS = 4005\n")
            f.write("MOVE_TYPE = 5\n")
            f.write("MAX_CANDIDATES = 50\n")
            f.write("TRACE_LEVEL = 1\n")
    except Exception as e:
        print(f"  ✗ 保存PAR文件时出错: {e}")
        return False, {}

    return True, {
        'tsp_file': tsp_path,
        'par_file': par_path,
        'solution_file': solution_path,
    }
