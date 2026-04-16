"""
水域块覆盖路径处理模块（通用：任意块号）

本模块负责单块水域的覆盖路径生成（起点/终点由外部已计算或人工标注）：
1. 生成采样点
2. 计算成本矩阵
3. 生成TSP和PAR文件
4. 调用LKH求解器
5. 处理求解结果并写入 data/coverage_paths/block_XX_coverage_path.txt
"""

import os
import numpy as np
from typing import Dict, List, Optional, Tuple
from shapely.geometry import Polygon
from water_coverage import generate_water_sampling_points
from water_cost_matrix import calculate_sampling_points_cost_matrix
from water_tsp_generator import generate_water_tsp_files
from lkh_solver import solve_tsp_with_lkh, parse_lkh_solution
from paths import DATA_DIR


# 覆盖路径结果目录名（相对 data_dir）
COVERAGE_PATHS_SUBDIR = "coverage_paths"

# 覆盖路径结果文件名模式：block_XX_coverage_path.txt
COVERAGE_PATH_FILENAME_PATTERN = "block_{:02d}_coverage_path.txt"


def load_coverage_paths_from_files(
    coverage_paths_dir: str,
) -> Dict[int, Dict]:
    """
    从 data/coverage_paths/ 目录加载所有 block_XX_coverage_path.txt，解析为块号 → 采样点 + tour 顺序。
    开关为 False 时由 main 调用，用于可视化等。

    :param coverage_paths_dir: 覆盖路径结果目录，如 data/coverage_paths
    :return: { block_no: { 'sampling_points': [[x,y],...], 'tour_order': [0, 5, 1, ...] } }，块号 1-based
    """
    import re
    result = {}
    if not os.path.isdir(coverage_paths_dir):
        return result

    # 匹配 block_01_coverage_path.txt, block_02_coverage_path.txt, ...
    pattern = re.compile(r"^block_(\d+)_coverage_path\.txt$")
    names = sorted(os.listdir(coverage_paths_dir))
    for name in names:
        m = pattern.match(name)
        if not m:
            continue
        block_no = int(m.group(1))
        filepath = os.path.join(coverage_paths_dir, name)
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                lines = [line.strip() for line in f.readlines()]
            block_no_read = None
            points_count = None
            sampling_points = []
            tour_order = []

            i = 0
            while i < len(lines):
                line = lines[i]
                if line.startswith("BLOCK_NO:"):
                    block_no_read = int(line.split(":", 1)[1].strip())
                elif line.startswith("POINTS:"):
                    points_count = int(line.split(":", 1)[1].strip())
                elif line == "COORDINATES:":
                    i += 1
                    while i < len(lines) and lines[i] and not lines[i].startswith("TOUR_ORDER"):
                        parts = lines[i].split()
                        if len(parts) >= 3:
                            x, y = float(parts[1]), float(parts[2])
                            sampling_points.append([x, y])
                        i += 1
                    continue
                elif line.startswith("TOUR_ORDER:"):
                    rest = line.split(":", 1)[1].strip() if ":" in line else ""
                    if rest:
                        try:
                            tour_order = [int(x) for x in rest.split()]
                        except ValueError:
                            tour_order = []
                    if not tour_order:
                        i += 1
                        while i < len(lines) and not lines[i].strip():
                            i += 1
                        if i < len(lines):
                            try:
                                tour_order = [int(x) for x in lines[i].strip().split()]
                            except ValueError:
                                tour_order = []
                    break
                i += 1

            if block_no_read is not None and sampling_points and tour_order:
                result[block_no] = {
                    "sampling_points": sampling_points,
                    "tour_order": tour_order,
                }
        except Exception as e:
            print(f"  ⚠ 加载 {name} 失败: {e}")

    return result


def process_water_block_coverage(
    block_no: int,
    water_info: Dict,
    water_poly: Polygon,
    start_point: List[float],
    exit_point: List[float],
    grid_data: Dict,
    env_data: Dict,
    coverage_paths_dir: str,
    skip_sampling_and_path: bool = False,
    fill_spacing: float = 4.0,
    x_scale_factor: float = 0.85,
    y_scale_factor: float = 1.0,
    degenerate_threshold: float = 1.0,
    lkh_runs: int = 10,
) -> Tuple[Optional[List[float]], Optional[List[List[float]]], Optional[List[int]], Optional[List[float]]]:
    """
    通用：处理第 block_no 块水域的覆盖路径。起点/终点由调用方传入。
    结果写入 coverage_paths_dir/block_XX_coverage_path.txt（便于后续加载）。

    :param block_no: 块号（1-based，骨架路径访问顺序）
    :param water_info: 当前块水域信息（与单块水域结构一致）
    :param water_poly: 当前块水域多边形（Shapely Polygon）
    :param start_point: 覆盖路径起点 [x, y]
    :param exit_point: 覆盖路径终点 [x, y]
    :param grid_data: 栅格数据
    :param env_data: 环境数据
    :param coverage_paths_dir: 覆盖路径结果目录，如 data/coverage_paths
    :param skip_sampling_and_path: True 时跳过采样/LKH，仅返回起终点
    :param fill_spacing: 采样点间距
    :param x_scale_factor: X 轴缩放因子
    :param y_scale_factor: Y 轴缩放因子
    :param degenerate_threshold: 起点与退出点距离小于此值时视为同一点，用地图轴生成网格；建议取水域块50的起退点距离
    :param lkh_runs: PAR 中的 RUNS（LKH 运行次数，越大解越优、越慢）
    :return: (start_point, sampling_points, tour_order, exit_point)
    """
    out_start = start_point
    out_sampling_points = None
    out_tour_order = None
    out_exit = exit_point

    if start_point is None or exit_point is None:
        print(f"  ✗ 第{block_no}块：起点或终点未提供")
        return out_start, out_sampling_points, out_tour_order, out_exit

    prefix = f"block_{block_no:02d}"
    os.makedirs(coverage_paths_dir, exist_ok=True)

    if skip_sampling_and_path:
        print(f"  [提示] 第{block_no}块：跳过采样/成本矩阵/LKH，仅返回起终点。")
        return out_start, None, None, out_exit

    try:
        print(f"  [调试] 第{block_no}块：生成覆盖路径（起点/终点已传入）")

        boundary_poly = Polygon(env_data["boundary_polygon"]) if env_data.get("boundary_polygon") is not None else None
        sampling_points = generate_water_sampling_points(
            grid_data,
            water_info,
            water_poly,
            start_point=start_point,
            exit_point=exit_point,
            fill_spacing=fill_spacing,
            x_scale_factor=x_scale_factor,
            y_scale_factor=y_scale_factor,
            degenerate_threshold=degenerate_threshold,
            boundary_polygon=boundary_poly,
        )

        if not sampling_points or len(sampling_points) < 2:
            print(f"  ✗ 第{block_no}块：采样点不足")
            return out_start, sampling_points, out_tour_order, out_exit

        cost_matrix, cost_info = calculate_sampling_points_cost_matrix(
            sampling_points,
            water_poly,
            boundary_polygon=boundary_poly,
        )

        if cost_matrix is None:
            print(f"  ✗ 第{block_no}块：成本矩阵计算失败")
            return out_start, sampling_points, out_tour_order, out_exit

        # 中间文件（可选，便于调试）写在同一目录，带块号前缀
        cost_matrix_path = os.path.join(coverage_paths_dir, f"{prefix}_cost_matrix.txt")
        np.savetxt(cost_matrix_path, cost_matrix, fmt='%d', delimiter=' ')
        sampling_points_path = os.path.join(coverage_paths_dir, f"{prefix}_sampling_points.txt")
        with open(sampling_points_path, 'w', encoding='utf-8') as f:
            f.write(f"# 第{block_no}块采样点\n")
            f.write(f"# 总点数: {len(sampling_points)}\n")
            f.write("# 索引0: 起点, 1: 退出点, 2+: 网格点\n")
            f.write("# 格式: 索引 X Y\n\n")
            for idx, pt in enumerate(sampling_points):
                f.write(f"{idx} {pt[0]:.6f} {pt[1]:.6f}\n")

        tsp_name = f"{prefix}.tsp"
        par_name = f"{prefix}.par"
        solution_name = f"{prefix}.tour"
        success, tsp_info = generate_water_tsp_files(
            cost_matrix,
            sampling_points,
            output_dir=coverage_paths_dir,
            tsp_name=tsp_name,
            par_name=par_name,
            solution_name=solution_name,
            lkh_runs=lkh_runs,
        )

        if not success:
            print(f"  ✗ 第{block_no}块：TSP/PAR 文件生成失败")
            return out_start, sampling_points, out_tour_order, out_exit

        # LKH 从 coverage_paths 目录调用（LKH-3.exe 放在 data/coverage_paths/ 下），par/tour 同目录
        data_dir_for_lkh = coverage_paths_dir
        tour_order_raw = solve_tsp_with_lkh(
            data_dir_for_lkh,
            par_name=par_name,
            solution_name=solution_name,
            output_name=f"{prefix}_tsp_path.txt",
        )

        if tour_order_raw is None:
            solution_path = os.path.join(data_dir_for_lkh, solution_name)
            if os.path.exists(solution_path):
                tour_order_raw = parse_lkh_solution(solution_path)
                if tour_order_raw is not None:
                    print(f"  [提示] 第{block_no}块：LKH 未返回结果，使用已有 .tour 文件")
            if tour_order_raw is None:
                print(f"  ✗ 第{block_no}块：LKH 求解失败")
                return out_start, sampling_points, out_tour_order, out_exit

        n_points = len(sampling_points)
        virtual_node_idx = n_points
        tour_order = [i for i in tour_order_raw if i != virtual_node_idx]

        if len(tour_order) == 0:
            print(f"  ✗ 第{block_no}块：访问顺序为空")
            return out_start, sampling_points, out_tour_order, out_exit

        if tour_order[0] != 0 and 0 in tour_order:
            start_pos = tour_order.index(0)
            tour_order = tour_order[start_pos:] + tour_order[:start_pos]
        if tour_order[-1] != 1 and 1 in tour_order:
            tour_order = [i for i in tour_order if i != 1] + [1]

        out_sampling_points = sampling_points
        out_tour_order = tour_order

        # 写入最终结果：block_XX_coverage_path.txt（供后续加载）
        out_path = os.path.join(coverage_paths_dir, f"{prefix}_coverage_path.txt")
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(f"BLOCK_NO: {block_no}\n")
            f.write(f"POINTS: {len(sampling_points)}\n")
            f.write("COORDINATES:\n")
            for idx, pt in enumerate(sampling_points):
                f.write(f"{idx} {pt[0]:.6f} {pt[1]:.6f}\n")
            f.write("TOUR_ORDER:\n")
            f.write(" ".join(map(str, tour_order)) + "\n")

        print(f"  ✓ 第{block_no}块：覆盖路径已保存 {out_path}")

    except Exception as e:
        print(f"  ⚠ 第{block_no}块处理出错: {e}")
        import traceback
        traceback.print_exc()

    return out_start, out_sampling_points, out_tour_order, out_exit


def process_first_block_coverage(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    grid_data: Dict,
    env_data: Dict,
    start_point: Optional[List[float]] = None,
    exit_point: Optional[List[float]] = None,
    core_points_data: Optional[Dict] = None,
    skip_sampling_and_path: bool = False,
    data_dir: Optional[str] = None,
) -> Tuple[Optional[List[float]], Optional[List[List[float]]], Optional[List[int]], Optional[List[float]]]:
    """
    处理第 1 块水域的覆盖路径（兼容旧调用）。内部委托给 process_water_block_coverage(block_no=1)。
    结果写入 data_dir/coverage_paths/block_01_coverage_path.txt。
    """
    data_dir = data_dir or str(DATA_DIR)
    block1_start = start_point
    block1_sampling_points = None
    block1_tour = None
    block1_exit = exit_point

    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        print(f"  ✗ TSP结果无效或点数量不足（需要至少2个点）")
        return block1_start, block1_sampling_points, block1_tour, block1_exit

    if start_point is None or exit_point is None:
        print(f"  ✗ 起点或终点未提供（需由调用方计算或人工标注后传入）")
        return block1_start, block1_sampling_points, block1_tour, block1_exit

    tour_order = tsp_tour_result['tour_order']
    merged_points = tsp_tour_result['merged_points']
    first_idx = tour_order[0]

    block1_water_info = None
    block1_water_idx = None
    if first_idx < len(merged_points):
        point_info = merged_points[first_idx]
        block1_water_idx = point_info.get('water_idx')
        if point_info.get('type') == 'no_hole':
            for water in water_no_hole:
                if water.get('idx') == block1_water_idx:
                    block1_water_info = water
                    break
        elif point_info.get('type') == 'with_hole':
            for water in water_with_hole:
                if water.get('idx') == block1_water_idx:
                    block1_water_info = water
                    break

    if not block1_water_info:
        print(f"  ✗ 未找到第 1 块水域（water_idx={block1_water_idx}）的信息")
        return block1_start, block1_sampling_points, block1_tour, block1_exit

    water_poly = Polygon(block1_water_info['outer'], holes=block1_water_info.get('holes', []))
    coverage_paths_dir = os.path.join(data_dir, COVERAGE_PATHS_SUBDIR)

    block1_start, block1_sampling_points, block1_tour, block1_exit = process_water_block_coverage(
        block_no=1,
        water_info=block1_water_info,
        water_poly=water_poly,
        start_point=start_point,
        exit_point=exit_point,
        grid_data=grid_data,
        env_data=env_data,
        coverage_paths_dir=coverage_paths_dir,
        skip_sampling_and_path=skip_sampling_and_path,
    )

    return block1_start, block1_sampling_points, block1_tour, block1_exit


# 兼容旧命名
process_first_water_coverage = process_first_block_coverage
