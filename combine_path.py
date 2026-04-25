"""
整体路径合并模块

将各水域块覆盖路径与骨架路径（出口→入口段）按访问顺序拼接，
得到完整的点坐标序列并保存，供后续统计与分析。
"""

import os
from typing import Dict, List, Optional, Tuple


def _get_coverage_path_points(
    sampling_points: List[List[float]],
    tour_order: List[int],
) -> List[List[float]]:
    """按 tour_order 顺序将采样点展开为坐标序列。"""
    if not tour_order or not sampling_points:
        return []
    return [list(sampling_points[i]) for i in tour_order if i < len(sampling_points)]


def _get_skeleton_path_between_cores(
    paths_dict: Dict,
    from_idx: int,
    to_idx: int,
) -> Optional[List[List[float]]]:
    """取骨架路径 from_idx → to_idx（核心点索引）。paths_dict 的 key 为 'i->j' 且 i<j。"""
    i, j = min(from_idx, to_idx), max(from_idx, to_idx)
    key = f"{i}->{j}"
    if key not in paths_dict:
        return None
    path = paths_dict[key].get("path")
    if not path:
        return None
    path = [[float(p[0]), float(p[1])] for p in path]
    if from_idx > to_idx:
        path = path[::-1]
    return path


def _skeleton_segment_exit_to_entrance(
    path_core_to_core: List[List[float]],
    exit_point: List[float],
    start_point: List[float],
) -> List[List[float]]:
    """
    从「核心点→核心点」整段路径中截取出口→入口段。
    在 path 上找到离 exit_point 最近的点索引 i、离 start_point 最近的点索引 j（i<=j），
    返回 [exit_point] + path[i:j+1] + [start_point]。
    """
    if not path_core_to_core:
        return [list(exit_point), list(start_point)]

    def dist2(p, q):
        return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2

    best_i = 0
    best_j = 0
    d_exit = dist2(path_core_to_core[0], exit_point)
    d_start = dist2(path_core_to_core[0], start_point)
    for k, p in enumerate(path_core_to_core):
        if dist2(p, exit_point) < d_exit:
            d_exit = dist2(p, exit_point)
            best_i = k
        if dist2(p, start_point) < d_start:
            d_start = dist2(p, start_point)
            best_j = k

    i, j = min(best_i, best_j), max(best_i, best_j)
    mid = [list(p) for p in path_core_to_core[i : j + 1]]
    return [list(exit_point)] + mid + [list(start_point)]


def build_combined_path(
    all_blocks_coverage_paths: Dict,
    tsp_tour_result: Dict,
    all_water_entry_exit: Dict,
    max_blocks: Optional[int] = None,
) -> Tuple[List[List[float]], Dict]:
    """
    按访问顺序拼接：块1覆盖路径 + (出口1→入口2) + 块2覆盖路径 + (出口2→入口3) + ...

    骨架段严格为「出口→入口」：在核心点间骨架路径上截取离出口/入口最近的子段，
    并在首尾加上出口点、入口点。

    :param all_blocks_coverage_paths: { block_no: {'sampling_points': [[x,y],...], 'tour_order': [...]} }
    :param tsp_tour_result: { 'tour_order': [...], 'paths_dict': { 'i->j': {'path': [...]} } }
    :param all_water_entry_exit: { block_no: { 'start_point': [x,y], 'exit_point': [x,y] } }
    :param max_blocks: 最多合并到第几块，None 表示全部有覆盖路径的块
    :return: (full_path 点序列 [[x,y],...], info 统计信息)
    """
    tour_order = (tsp_tour_result or {}).get("tour_order", [])
    paths_dict = (tsp_tour_result or {}).get("paths_dict", {})
    if not tour_order or not paths_dict:
        return [], {"error": "缺少 TSP 访问顺序或骨架路径"}

    blocks = sorted(k for k in all_blocks_coverage_paths.keys() if k >= 1)
    if max_blocks is not None:
        blocks = [k for k in blocks if k <= max_blocks]
    if not blocks:
        return [], {"blocks_used": 0}

    full_path = []
    segment_counts = {"coverage": 0, "skeleton": 0}

    for idx, block_no in enumerate(blocks):
        info = all_blocks_coverage_paths[block_no]
        sp = info.get("sampling_points")
        tour = info.get("tour_order")
        coverage_pts = _get_coverage_path_points(sp or [], tour or [])
        if not coverage_pts:
            continue
        segment_counts["coverage"] += 1
        # 第一块：整段覆盖路径；后续块：跳过首点（与上一段骨架末尾入口重复）
        if idx == 0:
            full_path.extend(coverage_pts)
        else:
            full_path.extend(coverage_pts[1:])

        # 骨架段：当前块出口 → 下一块入口（仅出口→入口段）
        next_block = block_no + 1
        if next_block not in blocks:
            break
        exit_pt = (all_water_entry_exit.get(block_no) or {}).get("exit_point")
        start_next = (all_water_entry_exit.get(next_block) or {}).get("start_point")
        if not exit_pt or not start_next:
            continue
        i_tour = block_no - 1
        j_tour = next_block - 1
        if i_tour >= len(tour_order) or j_tour >= len(tour_order):
            continue
        from_idx = tour_order[i_tour]
        to_idx = tour_order[j_tour]
        path_core = _get_skeleton_path_between_cores(paths_dict, from_idx, to_idx)
        if not path_core:
            full_path.append(list(exit_pt))
            full_path.append(list(start_next))
            segment_counts["skeleton"] += 1
            continue
        segment = _skeleton_segment_exit_to_entrance(path_core, exit_pt, start_next)
        # 首点=出口（与当前覆盖路径末点重复），只追加 segment[1:]
        if len(segment) >= 2:
            full_path.extend(segment[1:])
        segment_counts["skeleton"] += 1

    info_dict = {
        "blocks_used": len(blocks),
        "total_points": len(full_path),
        "segment_counts": segment_counts,
    }
    return full_path, info_dict


def save_combined_path(
    full_path: List[List[float]],
    output_dir: str,
    filename: str = "combined_path.txt",
) -> str:
    """
    将完整点序列保存为文本：每行一个点 "x y"。

    :param full_path: [[x, y], ...]
    :param output_dir: 输出目录
    :param filename: 文件名
    :return: 写入的完整路径
    """
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, filename)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# 整体路径点序列（覆盖路径 + 骨架出口→入口段）\n")
        f.write("# 每行一个点: x y\n")
        for p in full_path:
            if len(p) >= 2:
                f.write(f"{p[0]} {p[1]}\n")
    return out_path
