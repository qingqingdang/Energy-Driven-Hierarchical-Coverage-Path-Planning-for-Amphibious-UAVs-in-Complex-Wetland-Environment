"""
水域块覆盖路径起点终点计算模块

本模块用于计算中间水域块的覆盖路径起点和终点：
- 起点（入口点）：入口骨架路径与当前水域块边界的交点；若有多交点，沿路径方向第一个交点为起点
- 终点（出口点）：出口骨架路径与当前水域块边界的交点；若有多交点，沿路径方向最后一个交点为终点
"""

from typing import Dict, List, Optional, Tuple
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union


def calculate_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    water_idx_in_tour: int = None,
    debug_print: bool = False
) -> Tuple[Optional[List[float]], Optional[List[float]], Optional[int]]:
    """
    计算指定水域块的覆盖路径起点和终点
    
    规则：
    - 起点：入口路径与当前水域边界的交点；若骨架路径与水域有多个交点，沿路径方向第一个交点为起点
    - 终点：出口路径与当前水域边界的交点；若有多交点，沿路径方向最后一个交点为终点
    - 聚类水域：边界为聚类凸包∩水域；单连通：边界为 water["outer"]
    
    :param tsp_tour_result: TSP骨架路径结果字典，包含tour_order, paths_dict, merged_points
    :param water_no_hole: 无孔洞水域列表
    :param water_with_hole: 有孔洞水域列表
    :param core_points_data: 核心点数据，包含water_with_hole_clusters信息（用于判断是否是聚类水域块）
    :param water_idx_in_tour: 水域块在tour_order中的索引位置（0-based，0是第一个水域块）
                            如果为None，则自动找到第二个不同的水域块
    :return: (start_point, exit_point, water_idx)，如果计算失败返回(None, None, None)
    """
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return None, None, None
    
    tour_order = tsp_tour_result['tour_order']
    paths_dict = tsp_tour_result['paths_dict']
    merged_points = tsp_tour_result['merged_points']
    
    # 如果water_idx_in_tour为None，自动找到第二个不同的水域块
    if water_idx_in_tour is None:
        # 找到第二个不同的水域块
        # 首先获取第一个水域块的water_idx
        first_idx = tour_order[0]
        if first_idx >= len(merged_points):
            return None, None, None
        
        first_point_info = merged_points[first_idx]
        first_water_idx = first_point_info.get('water_idx')
        
        # 从tour_order中找到第二个不同的水域块
        current_idx = None
        for i in range(1, len(tour_order)):
            point_idx = tour_order[i]
            if point_idx >= len(merged_points):
                continue
            
            point_info = merged_points[point_idx]
            point_water_idx = point_info.get('water_idx')
            
            # 如果找到不同的水域块
            if point_water_idx != first_water_idx:
                water_idx_in_tour = i
                current_idx = point_idx
                break
        
        if current_idx is None:
            return None, None, None
    else:
        # 使用指定的索引
        if water_idx_in_tour < 0 or water_idx_in_tour >= len(tour_order):
            return None, None, None
        current_idx = tour_order[water_idx_in_tour]
    
    if current_idx >= len(merged_points):
        return None, None, None
    
    current_point_info = merged_points[current_idx]
    current_water_idx = current_point_info.get('water_idx')
    current_water_type = current_point_info.get('type')

    # 使用water_idx_in_tour作为actual_water_idx_in_tour（已经在上面确定）
    actual_water_idx_in_tour = water_idx_in_tour
    block_no = actual_water_idx_in_tour + 1  # 骨架路径上的块序号（1-based）
    
    # 查找当前水域块信息
    current_water_info = None
    if current_water_type == 'no_hole':
        for water in water_no_hole:
            if water.get('idx') == current_water_idx:
                current_water_info = water
                break
    elif current_water_type == 'with_hole':
        for water in water_with_hole:
            if water.get('idx') == current_water_idx:
                current_water_info = water
                break
    
    if not current_water_info:
        return None, None, None
    
    # 创建水域多边形（用于后续的交集计算）
    water_poly = Polygon(
        current_water_info['outer'], 
        holes=current_water_info.get('holes', [])
    )
    
    # 判断是否是聚类产生的水域块，并获取对应的聚类边界（交集边界）
    cluster_boundary_poly = None
    hull_poly = None  # 当前cluster的凸包（仅用于构造 cluster_boundary_poly，可视化仍使用其与水域的交集）
    if core_points_data and current_water_type == 'with_hole':
        clusters_map = core_points_data.get('water_with_hole_clusters', {})
        if current_water_idx in clusters_map:
            clusters = clusters_map[current_water_idx]
            # 找到当前核心点属于哪个cluster
            current_point = merged_points[current_idx]['point']
            for cluster in clusters:
                hull_coords = cluster.get('hull')
                if hull_coords and len(hull_coords) >= 3:
                    try:
                        hull_poly_candidate = Polygon(hull_coords)
                        if not hull_poly_candidate.is_empty and hull_poly_candidate.contains(Point(current_point[0], current_point[1])):
                            # 记录当前cluster的凸包
                            hull_poly = hull_poly_candidate
                            # 计算聚类凸包与水域的交集（新的聚类边界）
                            intersection = hull_poly.intersection(water_poly)
                            if not intersection.is_empty:
                                # 如果交集是Polygon，直接使用
                                if intersection.geom_type == 'Polygon':
                                    cluster_boundary_poly = intersection
                                    break
                                # 如果交集是MultiPolygon，使用第一个子多边形
                                elif intersection.geom_type == 'MultiPolygon' and len(intersection.geoms) > 0:
                                    cluster_boundary_poly = intersection.geoms[0]
                                    break
                    except Exception:
                        continue
    
    # 确定用于计算交点的边界多边形
    # 如果是聚类水域块，使用聚类边界与水域的交集；否则使用水域块实际边界
    if cluster_boundary_poly is not None:
        boundary_poly = cluster_boundary_poly
    else:
        boundary_poly = water_poly
    
    # 计算起点（入口点）：从上一个水域块到当前水域块的路径；第一块由「最后一块→第一块」路径与边界求交得到（与求解函数一致）
    # 注意：路径可能先经过其他水域块，需要找到路径与当前水域块相关边界（cluster_boundary_poly + water_poly）的正确交点
    start_point = None
    if actual_water_idx_in_tour == 0:
        # 第一块：用「最后一块→第一块」闭圈路径与当前块边界求交，得到入口点（与终点对称的求解方式）
        last_idx = tour_order[-1]
        entry_path_key = f"{last_idx}->{current_idx}"
        if entry_path_key not in paths_dict:
            entry_path_key = f"{current_idx}->{last_idx}"
        if debug_print:
            entry_path = paths_dict.get(entry_path_key, {}).get('path', []) if entry_path_key in paths_dict else []
            print(f"    [第一块入口路径] 第{block_no}块 key={entry_path_key} 在paths_dict={entry_path_key in paths_dict} 路径点数={len(entry_path)}")
        if entry_path_key in paths_dict:
            entry_path = paths_dict[entry_path_key].get('path', [])
            if entry_path:
                # 入口为「从最后一块到第一块」方向上的第一个交点；若 key 是 first->last 则需反转路径
                if entry_path_key == f"{current_idx}->{last_idx}":
                    entry_path = entry_path[::-1]
                ref_point = merged_points[last_idx]['point']
                start_point = _find_path_water_intersection(
                    entry_path,
                    boundary_poly,
                    ref_point,
                    is_entry=True,
                    water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if start_point is None and cluster_boundary_poly is not None:
                    start_point = _find_path_water_intersection(
                        entry_path,
                        water_poly,
                        ref_point,
                        is_entry=True,
                        water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
        if start_point is None:
            pt = merged_points[current_idx].get('point')
            if pt is not None:
                start_point = [float(pt[0]), float(pt[1])]
            if debug_print and start_point:
                print(f"    [第一块起点] 第{block_no}块 求交无结果，回退到骨架代表点: ({start_point[0]:.2f}, {start_point[1]:.2f})")
    elif actual_water_idx_in_tour > 0:
        prev_idx = tour_order[actual_water_idx_in_tour - 1]
        entry_path_key = f"{prev_idx}->{current_idx}"
        if entry_path_key not in paths_dict:
            entry_path_key = f"{current_idx}->{prev_idx}"
        if debug_print:
            entry_path = paths_dict.get(entry_path_key, {}).get('path', []) if entry_path_key in paths_dict else []
            print(f"    [入口路径] 第{block_no}块 key={entry_path_key} 在paths_dict={entry_path_key in paths_dict} 路径点数={len(entry_path)}")
        
        if entry_path_key in paths_dict:
            entry_path = paths_dict[entry_path_key].get('path', [])
            if entry_path:
                # 统一使用“水域边界 + 聚类边界（cluster_boundary_poly）”的合集，根据沿路径距离排序交点：
                # 第一个交点作为入口起点。避免只看 cluster_boundary_poly 或 water_poly 导致漏掉最先接触的边界。
                ref_point = merged_points[prev_idx]['point']
                boundary_geoms = [water_poly]
                if cluster_boundary_poly is not None:
                    boundary_geoms.append(cluster_boundary_poly)

                # 将所有边界的 exterior 合并成一组几何，用于一次性交点计算
                boundary_lines = [LineString(p.exterior.coords) for p in boundary_geoms if not p.is_empty]
                if boundary_lines:
                    full_path = LineString(entry_path)
                    inter = full_path.intersection(boundary_lines[0] if len(boundary_lines) == 1 else unary_union(boundary_lines))
                    pts = _extract_points_from_intersection(inter)

                    if pts:
                        candidates = []
                        for pt in pts:
                            d = float(full_path.project(Point(pt[0], pt[1])))
                            candidates.append((pt[0], pt[1], d))
                        candidates.sort(key=lambda c: c[2])  # 按沿路径距离从小到大排序
                        best = candidates[0]
                        start_point = [best[0], best[1]]

                # 若统一边界合集无交点，退回原有逻辑（按 boundary_poly / water_poly 逐步求交）
                if start_point is None:
                    start_point = _find_path_water_intersection(
                        entry_path,
                        boundary_poly,
                        ref_point,
                        is_entry=True,
                        water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
                if start_point is None and cluster_boundary_poly is not None:
                    start_point = _find_path_water_intersection(
                        entry_path,
                        water_poly,
                        ref_point,
                        is_entry=True,
                        water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
    
    # 计算终点（出口点）：从当前水域块到下一个水域块的路径
    # 注意：路径可能先经过其他水域块，需要找到路径离开当前水域块的那个点
    # 注意：最后一个点使用 当前->起点 的闭圈路径计算出口
    exit_point = None
    if actual_water_idx_in_tour < len(tour_order) - 1:
        # 非最后一点：使用 当前->下一个 的路径
        next_idx = tour_order[actual_water_idx_in_tour + 1]
        exit_path_key = f"{current_idx}->{next_idx}"
        if exit_path_key not in paths_dict:
            exit_path_key = f"{next_idx}->{current_idx}"
        if debug_print:
            exit_path = paths_dict.get(exit_path_key, {}).get('path', []) if exit_path_key in paths_dict else []
            print(f"    [出口路径] 第{block_no}块 key={exit_path_key} 在paths_dict={exit_path_key in paths_dict} 路径点数={len(exit_path)}")
        
        if exit_path_key in paths_dict:
            exit_path = paths_dict[exit_path_key].get('path', [])
            if exit_path:
                exit_point = _find_path_water_intersection(
                    exit_path,
                    boundary_poly,
                    merged_points[next_idx]['point'],
                    is_entry=False,
                    water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                # 聚类边界无交点时，尝试用全水域边界
                if exit_point is None and cluster_boundary_poly is not None:
                    exit_point = _find_path_water_intersection(
                        exit_path,
                        water_poly,
                        merged_points[next_idx]['point'],
                        is_entry=False,
                        water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
    else:
        # 最后一个点：使用 当前->起点 的 TSP 闭圈路径计算出口
        first_idx = tour_order[0]
        exit_path_key = f"{current_idx}->{first_idx}"
        if exit_path_key not in paths_dict:
            exit_path_key = f"{first_idx}->{current_idx}"
        if debug_print:
            exit_path = paths_dict.get(exit_path_key, {}).get('path', []) if exit_path_key in paths_dict else []
            print(f"    [出口路径-闭圈] 第{block_no}块 key={exit_path_key} 在paths_dict={exit_path_key in paths_dict} 路径点数={len(exit_path)}")
        
        if exit_path_key in paths_dict:
            exit_path = paths_dict[exit_path_key].get('path', [])
            if exit_path:
                # 出口：路径从当前点走向起点，选离 path 终点（起点）最近的在边界上的交点
                exit_point = _find_path_water_intersection(
                    exit_path,
                    boundary_poly,
                    merged_points[first_idx]['point'],
                    is_entry=False,
                    water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if exit_point is None and cluster_boundary_poly is not None:
                    exit_point = _find_path_water_intersection(
                        exit_path,
                        water_poly,
                        merged_points[first_idx]['point'],
                        is_entry=False,
                        water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
    
    # 手动矫正：按骨架路径上的块序号（第2块、第3块…）查找，与 main 和配置一致
    overrides = MANUAL_ENTRY_EXIT_OVERRIDES.get(block_no)
    if overrides:
        manual_start = overrides.get('start')
        manual_exit = overrides.get('exit')
        if manual_start is not None:
            start_point = manual_start
        if manual_exit is not None:
            exit_point = manual_exit
        # 终端提示：与 P19 手动矫正一致，便于之后知道哪些块被手动修改
        start_str = f"({start_point[0]:.2f}, {start_point[1]:.2f})" if start_point else "None"
        exit_str = f"({exit_point[0]:.2f}, {exit_point[1]:.2f})" if exit_point else "None"
        print(f"      [手动矫正] 第{block_no}块(water_idx={current_water_idx}): 起点={start_str} 终点={exit_str}")

    return start_point, exit_point, current_water_idx


def compute_all_water_entry_exit(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    debug_print: bool = False,
) -> Dict[int, Dict]:
    """
    计算所有水域块的起点/终点并合并为一份字典（含手动覆盖），供 main 用于可视化和覆盖路径生成。
    :return: { block_no: {'start_point': [x,y]|None, 'exit_point': [x,y]|None, 'tour_idx': int} }，块号 1-based
    """
    all_water_entry_exit = {}
    if not tsp_tour_result or len(tsp_tour_result.get("tour_order", [])) < 2:
        return all_water_entry_exit

    tour_order = tsp_tour_result["tour_order"]
    first_block_start, first_block_exit, _ = calculate_water_entry_exit_points(
        tsp_tour_result,
        water_no_hole,
        water_with_hole,
        core_points_data=core_points_data,
        water_idx_in_tour=0,
        debug_print=False,
    )
    all_water_entry_exit_for_viz = {}
    for i in range(1, len(tour_order)):
        debug_this = (i + 1) == 52
        start_pt, exit_pt, calc_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result,
            water_no_hole,
            water_with_hole,
            core_points_data=core_points_data,
            water_idx_in_tour=i,
            debug_print=bool(debug_print or debug_this),
        )
        block_no = i + 1
        if start_pt is None and block_no in (MANUAL_ENTRY_EXIT_OVERRIDES or {}):
            overrides = MANUAL_ENTRY_EXIT_OVERRIDES[block_no]
            all_water_entry_exit_for_viz[block_no] = {
                "start_point": overrides.get("start"),
                "exit_point": overrides.get("exit"),
                "tour_idx": i,
            }
            print(f"  [提示] 第{block_no}块(water_idx={calc_water_idx})自动求交无起点，已用手动矫正加入")
            continue
        if start_pt is None:
            continue
        all_water_entry_exit_for_viz[block_no] = {
            "start_point": start_pt,
            "exit_point": exit_pt,
            "tour_idx": i,
        }
        if exit_pt is None:
            print(f"  [提示] 第{block_no}块(water_idx={calc_water_idx})自动求交无终点；可配置 MANUAL_ENTRY_EXIT_OVERRIDES[{block_no}]['exit']")
    for block_no, overrides in (MANUAL_ENTRY_EXIT_OVERRIDES or {}).items():
        if block_no not in all_water_entry_exit_for_viz:
            continue
        entry = all_water_entry_exit_for_viz[block_no]
        if overrides.get("start") is not None:
            entry["start_point"] = overrides["start"]
        if overrides.get("exit") is not None:
            entry["exit_point"] = overrides["exit"]
        if overrides.get("start") is not None or overrides.get("exit") is not None:
            print(f"  [手动矫正] 第{block_no}块: 起点={entry.get('start_point')} 终点={entry.get('exit_point')}")

    all_water_entry_exit = {1: {"start_point": first_block_start, "exit_point": first_block_exit, "tour_idx": 0}}
    all_water_entry_exit.update(all_water_entry_exit_for_viz)
    for block_no, overrides in (MANUAL_ENTRY_EXIT_OVERRIDES or {}).items():
        if block_no not in all_water_entry_exit:
            continue
        entry = all_water_entry_exit[block_no]
        if overrides.get("start") is not None:
            entry["start_point"] = overrides["start"]
        if overrides.get("exit") is not None:
            entry["exit_point"] = overrides["exit"]
        if (overrides.get("start") is not None or overrides.get("exit") is not None) and block_no == 1:
            print(f"  [手动矫正] 第{block_no}块: 起点={entry.get('start_point')} 终点={entry.get('exit_point')}")
    return all_water_entry_exit


def _extract_points_from_intersection(geom) -> List[List[float]]:
    """
    从 intersection 结果中提取所有点坐标。支持 Point, MultiPoint, LineString,
    MultiLineString, GeometryCollection。LineString/MultiLineString 提取全部顶点，
    避免只取首尾导致中间转折点或重合段漏点。
    """
    out = []
    if geom is None or geom.is_empty:
        return out
    if geom.geom_type == 'Point':
        out.append([geom.x, geom.y])
    elif geom.geom_type == 'MultiPoint':
        for p in geom.geoms:
            out.append([p.x, p.y])
    elif geom.geom_type == 'LineString':
        out.extend([list(p) for p in geom.coords])
    elif geom.geom_type == 'MultiLineString':
        for line in geom.geoms:
            out.extend([list(p) for p in line.coords])
    elif geom.geom_type == 'GeometryCollection':
        for g in geom.geoms:
            out.extend(_extract_points_from_intersection(g))
    return out


# 设为指定 water_idx 时打印该块与骨架路径的全部交点；设为 None 可关闭
_DEBUG_P19_INTERSECTION = 19

# 手动矫正配置：当自动求解不稳定时，对少数水域块强制指定起点/终点
# 统一约定：key = 骨架路径上的块序号（第2块、第7块…），不是水域标号 Pxx
# 例如 P19 是水域标号，对应骨架路径第 36 块，则写 36
MANUAL_ENTRY_EXIT_OVERRIDES: Dict[int, Dict[str, Optional[List[float]]]] = {
    # 第2块：起点修正
    2: {
        'start': [64.1, -32.0],
        'exit':  None,
    },
    # 第7块：起点/终点修正
    7: {
        'start': [55.8, 168.3],
        'exit':  [49.8, 183.8],
    },
    # 第12块：仅终点修正
    12: {
        'start': None,
        'exit':  [-60.8, 15.8],
    },
    # 第36块（水域标号 P19）：起点/终点手动矫正
    36: {
        'start': [-326.0, 87.5],
        'exit':  [-236.5, -44.5],
    },
    # 第37块：起点手动设置；终点自动求交若无则保持 None
    37: {
        'start': [-251.0, -59.5],
        'exit':  None,
    },
    # 第42块：起点手动设置
    42: {
        'start': [-278.4, -212.3],
        'exit':  None,
    },
    # 第52块：骨架路径与该水域边应有两个交点（起点/终点）；若自动求交失败可在此填写
    # 填写后运行会显示 [手动矫正] 并参与可视化
    52: {
        'start': None,  # 例: [-x, -y]
        'exit':  None,  # 例: [-x, -y]
    },
}


def _find_path_water_intersection(
    path: List[List[float]],
    water_poly: Polygon,
    reference_point: List[float],
    is_entry: bool = True,
    water_idx: Optional[int] = None,
    debug_print_intersections: bool = False,
) -> Optional[List[float]]:
    """
    找到路径与水域块外边界（不含孔洞）的交点。

    1. 使用 water_poly.exterior（仅外边界，不含孔洞）求交。
    2. 提取所有交点后，用 LineString.project 得到每个点在路径上的距离 d。
    3. 入口取 d 最小（最先接触），出口取 d 最大（最后离开）。
    4. 若主求交为空，用 exterior.buffer(0.01) 做一次容错求交，缓解浮点精度导致的不相交。
    """
    if not path or len(path) < 2:
        return None

    full_path = LineString(path)
    # 只使用外边界（exterior），排除孔洞边界
    exterior = LineString(water_poly.exterior.coords)
    inter = full_path.intersection(exterior)
    points = _extract_points_from_intersection(inter)

    if not points:
        # 容错：使用 buffer 后的外边界
        inter = full_path.intersection(exterior.buffer(0.01))
        points = _extract_points_from_intersection(inter)
    if not points:
        # 进一步容错：稍大 buffer，缓解浮点精度或路径与边界相切导致的不相交
        inter = full_path.intersection(exterior.buffer(0.1))
        points = _extract_points_from_intersection(inter)

    if not points:
        return None

    candidates = []
    for pt in points:
        d = float(full_path.project(Point(pt[0], pt[1])))
        candidates.append((pt[0], pt[1], d))

    # 通用 debug：打印骨架路径与水域边界的全部交点（按沿路径距离排序）
    if debug_print_intersections:
        role = "入口路径" if is_entry else "出口路径"
        wlabel = f"water_idx={water_idx}" if water_idx is not None else "water_idx=?"
        print(f"    [{role}] 水域块 {wlabel}: 共 {len(candidates)} 个交点")
        for i, c in enumerate(sorted(candidates, key=lambda x: x[2]), 1):
            print(f"      交点{i}: ({c[0]:.4f}, {c[1]:.4f})  沿路径距离: {c[2]:.2f}")
    elif water_idx is not None and _DEBUG_P19_INTERSECTION is not None and water_idx == _DEBUG_P19_INTERSECTION:
        role = "入口路径" if is_entry else "出口路径"
        print(f"    [P19] 骨架路径与 P19 边界求交（{role}）：共 {len(candidates)} 个交点")
        for i, c in enumerate(sorted(candidates, key=lambda x: x[2]), 1):
            print(f"      交点{i}: ({c[0]:.4f}, {c[1]:.4f})  沿路径距离: {c[2]:.2f}")

    if is_entry:
        best = min(candidates, key=lambda c: c[2])
    else:
        best = max(candidates, key=lambda c: c[2])
    return [best[0], best[1]]


def _extract_intersection_point(
    intersection,
    reference_point: List[float]
) -> Optional[List[float]]:
    """
    从交点几何对象中提取最合适的交点坐标（保留用于向后兼容）
    
    :param intersection: Shapely几何对象（Point, MultiPoint等）
    :param reference_point: 参考点坐标 [x, y]，用于选择最合适的交点
    :return: 交点坐标 [x, y]，如果无法提取返回None
    """
    if intersection.is_empty:
        return None
    
    if intersection.geom_type == 'Point':
        return [intersection.x, intersection.y]
    elif intersection.geom_type == 'MultiPoint':
        # 如果有多个交点，选择距离参考点最近的点
        points = [[p.x, p.y] for p in intersection.geoms]
        if points:
            return min(
                points,
                key=lambda p: np.sqrt(
                    (p[0] - reference_point[0])**2 + 
                    (p[1] - reference_point[1])**2
                )
            )
    elif intersection.geom_type == 'LineString':
        # 如果交点是线段，选择距离参考点最近的端点
        coords = list(intersection.coords)
        if coords:
            points = [list(coords[0]), list(coords[-1])]
            return min(
                points,
                key=lambda p: np.sqrt(
                    (p[0] - reference_point[0])**2 + 
                    (p[1] - reference_point[1])**2
                )
            )
    
    return None


def calculate_all_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None
) -> Dict[int, Dict]:
    """
    一次性计算所有中间水域块（非第一个和最后一个）的覆盖路径起点和终点
    
    规则：
    - 骨架路径与边界求交：若有多交点，沿路径方向第一个交点为起点，最后一个交点为终点
    - 起点=入口路径与边界的交点；终点=出口路径与边界的交点
    - 聚类水域：边界=聚类凸包∩水域；单连通：边界=water["outer"]
    
    :param tsp_tour_result: TSP骨架路径结果字典，包含tour_order, paths_dict, merged_points
    :param water_no_hole: 无孔洞水域列表
    :param water_with_hole: 有孔洞水域列表
    :param core_points_data: 核心点数据，包含water_with_hole_clusters信息（用于判断是否是聚类水域块）
    :return: 字典 {water_idx: {'start_point': [x, y], 'exit_point': [x, y], 'tour_idx': int}}, 
             如果某个水域块计算失败，则不包含在字典中
    """
    result = {}
    
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return result
    
    tour_order = tsp_tour_result['tour_order']
    merged_points = tsp_tour_result['merged_points']
    
    # 找到所有不同的水域块及其在tour_order中的位置
    water_indices_map = {}  # {water_idx: [tour_order中的索引列表]}
    
    for i, point_idx in enumerate(tour_order):
        if point_idx >= len(merged_points):
            continue
        
        point_info = merged_points[point_idx]
        water_idx = point_info.get('water_idx')
        
        if water_idx not in water_indices_map:
            water_indices_map[water_idx] = []
        water_indices_map[water_idx].append(i)
    
    # 遍历所有水域块（除了第一个和最后一个）
    # 第一个水域块：tour_order[0]对应的water_idx
    # 最后一个水域块：tour_order[-1]对应的water_idx
    first_water_idx = merged_points[tour_order[0]].get('water_idx')
    last_water_idx = merged_points[tour_order[-1]].get('water_idx')

    # 计算每个中间水域块的起点和终点
    for water_idx, tour_indices in water_indices_map.items():
        if water_idx == first_water_idx:
            continue
        is_last_water = (water_idx == last_water_idx)
        water_tour_idx = min(tour_indices)
        start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result,
            water_no_hole,
            water_with_hole,
            core_points_data=core_points_data,
            water_idx_in_tour=water_tour_idx
        )
        
        # 对于最后一个水域块，只需要起点（入口点），exit_point可以为None
        # 对于中间水域块，需要起点和终点都存在
        if is_last_water:
            if start_point is not None and calculated_water_idx == water_idx:
                result[water_idx] = {
                    'start_point': start_point,
                    'exit_point': None,
                    'tour_idx': water_tour_idx
                }
        else:
            if start_point is not None and exit_point is not None and calculated_water_idx == water_idx:
                result[water_idx] = {
                    'start_point': start_point,
                    'exit_point': exit_point,
                    'tour_idx': water_tour_idx
                }
    
    return result


def calculate_all_clustered_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    debug_print: bool = False
) -> Dict[tuple, Dict]:
    """
    计算所有聚类水域块的覆盖路径起点和终点
    
    规则：
    - 只处理 type='with_hole' 且在 core_points_data 中有聚类信息的水域
    - 骨架路径与聚类边界求交：若有多交点，沿路径方向第一个交点为起点，最后一个交点为终点
    - 起点=入口路径（上一→当前）与聚类边界的交点；终点=出口路径（当前→下一）与聚类边界的交点
    - 边界为聚类凸包∩水域；对 tour 中每个聚类核心点分别计算起点和终点
    
    :param tsp_tour_result: TSP骨架路径结果字典，包含tour_order, paths_dict, merged_points
    :param water_no_hole: 无孔洞水域列表
    :param water_with_hole: 有孔洞水域列表
    :param core_points_data: 核心点数据，包含water_with_hole_clusters信息（用于判断是否是聚类水域块）
    :return: 字典 {(water_idx, point_idx): {'start_point': [x, y], 'exit_point': [x, y], 'tour_idx': int, 'point_idx': int, 'water_idx': int}}, 
             如果某个水域块计算失败，则不包含在字典中
    """
    result = {}
    
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return result
    
    if not core_points_data:
        return result
    
    clusters_map = core_points_data.get('water_with_hole_clusters', {})
    if not clusters_map:
        return result
    
    tour_order = tsp_tour_result['tour_order']
    merged_points = tsp_tour_result['merged_points']
    
    # 找到所有聚类水域块及其在tour_order中的位置
    clustered_water_indices_map = {}  # {water_idx: {point_idx: tour_idx}}
    
    for i, point_idx in enumerate(tour_order):
        if point_idx >= len(merged_points):
            continue
        
        point_info = merged_points[point_idx]
        water_idx = point_info.get('water_idx')
        water_type = point_info.get('type')
        
        # 只处理聚类水域块（type='with_hole' 且在 clusters_map 中）
        if water_type == 'with_hole' and water_idx in clusters_map:
            if water_idx not in clustered_water_indices_map:
                clustered_water_indices_map[water_idx] = {}
            clustered_water_indices_map[water_idx][point_idx] = i
    
    for water_idx, point_indices_map in clustered_water_indices_map.items():
        for point_idx, tour_idx in point_indices_map.items():
            # 判断是否是最后一个点（最后一个聚类水域块可能没有出口点）
            is_last_point = (tour_idx == len(tour_order) - 1)
            
            # 计算该核心点的起点和终点
            start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
                tsp_tour_result,
                water_no_hole,
                water_with_hole,
                core_points_data=core_points_data,
                water_idx_in_tour=tour_idx,
                debug_print=debug_print
            )
            
            # 如果计算成功，保存结果
            # 对于最后一个点，exit_point 可能为 None（因为没有下一个水域块），只要有 start_point 即可
            if start_point is not None and calculated_water_idx == water_idx:
                if is_last_point:
                    pass  # 最后一个点：只需起点，exit_point 可认为 None
                else:
                    if exit_point is None:
                        continue
                if debug_print:
                    ep_str = f"({exit_point[0]:.4f}, {exit_point[1]:.4f})" if exit_point is not None else "None（最后一个点）"
                    print(f"聚类水域块{water_idx} 核心点{point_idx}: 起点=({start_point[0]:.4f}, {start_point[1]:.4f}) 终点={ep_str}")
                key = (water_idx, point_idx)
                result[key] = {
                    'start_point': start_point,
                    'exit_point': exit_point,
                    'tour_idx': tour_idx,
                    'point_idx': point_idx,
                    'water_idx': water_idx
                }
    
    return result


def calculate_all_single_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    debug_print: bool = False
) -> Dict[int, Dict]:
    """
    计算所有单连通水域块的覆盖路径起点和终点（跳过第一个水域块，其起点终点单独求解）
    
    规则：
    - 只处理 type='no_hole'，跳过 tour 中第一个水域块
    - 骨架路径与水域边界求交：若有多交点，沿路径方向第一个交点为起点，最后一个交点为终点
    - 起点=入口路径（上一→当前）与边界的交点；终点=出口路径（当前→下一）与边界的交点
    - 最后一点用 当前→起点 的闭圈路径求终点；边界为 water["outer"]（exterior）
    
    :param tsp_tour_result: TSP骨架路径结果，包含 tour_order, paths_dict, merged_points
    :param water_no_hole: 无孔洞水域列表
    :param water_with_hole: 有孔洞水域列表
    :param core_points_data: 核心点数据（单连通未使用，仅为接口一致）
    :return: {water_idx: {'start_point': [x,y]|None, 'exit_point': [x,y]|None, 'tour_idx': int}}
    """
    result = {}
    
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return result
    
    tour_order = tsp_tour_result['tour_order']
    merged_points = tsp_tour_result['merged_points']
    first_water_idx = merged_points[tour_order[0]].get('water_idx')

    for i, point_idx in enumerate(tour_order):
        if point_idx >= len(merged_points):
            continue
        point_info = merged_points[point_idx]
        if point_info.get('type') != 'no_hole':
            continue
        water_idx = point_info.get('water_idx')
        if water_idx == first_water_idx:
            continue  # 第一个水域块跳过，起点终点单独求解
        
        is_last_point = (i == len(tour_order) - 1)
        
        start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result,
            water_no_hole,
            water_with_hole,
            core_points_data=core_points_data,
            water_idx_in_tour=i,
            debug_print=debug_print
        )

        # 手动矫正在 calculate_water_entry_exit_points 内已通过 MANUAL_ENTRY_EXIT_OVERRIDES 统一处理

        if start_point is None or calculated_water_idx != water_idx:
            continue
        if not is_last_point and exit_point is None:
            continue
        if debug_print:
            ep_str = f"({exit_point[0]:.4f}, {exit_point[1]:.4f})" if exit_point is not None else "None"
            print(f"单连通水域块{water_idx}: 起点=({start_point[0]:.4f}, {start_point[1]:.4f}) 终点={ep_str}")
        result[water_idx] = {
            'start_point': start_point,
            'exit_point': exit_point,
            'tour_idx': i
        }
    
    return result
