"""
水域块覆盖路径起点终点计算模块

本模块用于计算中间水域块的覆盖路径起点和终点：
- 起点（入口点）：入口骨架路径与当前水域块边界的交点；若有多交点，沿路径方向第一个交点为起点
- 终点（出口点）：出口骨架路径与当前水域块边界的交点；若有多交点，沿路径方向最后一个交点为终点
"""

from typing import Dict, List, Optional, Tuple
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union, nearest_points


# ============================================================
# 模块级常量与配置
# ============================================================

# 求交容差（浮点精度兜底）
_INTERSECTION_TOLERANCE = 0.1

# 投影兜底的最大容许距离（米）；超过则认为是真实几何异常
# 设为 5.0 米以匹配栅格粒度(fill_spacing=4.0)
_PROJECTION_MAX_DIST = 5.0

# 设为指定 water_idx 时打印该块与骨架路径的全部交点；设为 None 可关闭
_DEBUG_P19_INTERSECTION = 19

# 手动矫正配置：当自动求解不稳定时，对少数水域块强制指定起点/终点
# 统一约定：key = 骨架路径上的块序号（第2块、第7块…），不是水域标号 Pxx
MANUAL_ENTRY_EXIT_OVERRIDES: Dict[int, Dict[str, Optional[List[float]]]] = {}


# ============================================================
# 辅助函数：几何处理
# ============================================================

def _pick_polygon_containing_point(geom, pt: Point) -> Optional[Polygon]:
    """从 Polygon / MultiPolygon / GeometryCollection 中挑出包含指定点的那个子多边形。"""
    if geom.is_empty:
        return None
    if geom.geom_type == 'Polygon':
        return geom if geom.covers(pt) else None
    if geom.geom_type == 'MultiPolygon':
        for sub in geom.geoms:
            if sub.covers(pt):
                return sub
        return None
    if hasattr(geom, 'geoms'):
        for sub in geom.geoms:
            if sub.geom_type == 'Polygon' and sub.covers(pt):
                return sub
    return None


def _pick_nearest_polygon(geom, pt: Point) -> Optional[Polygon]:
    """从 MultiPolygon 中挑出离指定点最近的子多边形(兜底用)。"""
    if geom.is_empty:
        return None
    if geom.geom_type == 'Polygon':
        return geom
    polys = [g for g in getattr(geom, 'geoms', []) if g.geom_type == 'Polygon']
    if not polys:
        return None
    return min(polys, key=lambda p: p.distance(pt))


def _extract_points_from_intersection(geom) -> List[List[float]]:
    """
    从 intersection 结果中提取所有点坐标。支持 Point, MultiPoint, LineString,
    MultiLineString, GeometryCollection。
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


def _collect_intersection_points(
    path_line: LineString,
    boundary: LineString,
    tolerance: float = 0,
) -> List[List[float]]:
    """求 path 与 boundary 的交点。tolerance>0 时用 buffer 容错。"""
    target = boundary if tolerance == 0 else boundary.buffer(tolerance)
    inter = path_line.intersection(target)
    return _extract_points_from_intersection(inter)


def _debug_log_candidates(candidates, is_entry, water_idx,
                          debug_flag, degraded, projection_used):
    """debug 打印逻辑，避免主函数过长。"""
    should_log = debug_flag or (
        water_idx is not None
        and _DEBUG_P19_INTERSECTION is not None
        and water_idx == _DEBUG_P19_INTERSECTION
    )
    if not should_log:
        return

    role = "入口路径" if is_entry else "出口路径"
    mode = "投影兜底" if projection_used else ("buffer容错" if degraded else "精确求交")
    print(f"    [{role}] water_idx={water_idx} ({mode}): 共 {len(candidates)} 个交点")
    for i, c in enumerate(sorted(candidates, key=lambda x: x[2]), 1):
        print(f"      交点{i}: ({c[0]:.4f}, {c[1]:.4f})  沿路径距离: {c[2]:.2f}")


def _find_path_water_intersection(
    path: List[List[float]],
    water_poly: Polygon,
    reference_point: List[float],
    is_entry: bool = True,
    water_idx: Optional[int] = None,
    debug_print_intersections: bool = False,
) -> Optional[List[float]]:
    """
    找路径与水域外边界（不含孔洞）的交点。

    策略(按优先级):
    1. 精确求交 —— 路径真的穿过边界。
    2. 小容差 buffer 求交 —— 缓解浮点精度导致的"擦边不相交"。
    3. 端点投影兜底 —— 路径端点在水域内或离边界足够近时,
       把靠水域的那个端点投影到最近边界点。
       若端点既不在水域内、距离也超过阈值,判定为真实几何异常,返回 None。
    """
    if not path or len(path) < 2:
        return None

    full_path = LineString(path)
    exterior = LineString(water_poly.exterior.coords)

    # ---------- 策略 1 & 2: 精确求交 + buffer 容错 ----------
    points = _collect_intersection_points(full_path, exterior, tolerance=0)
    degraded = False
    if not points:
        points = _collect_intersection_points(full_path, exterior, _INTERSECTION_TOLERANCE)
        degraded = bool(points)

    # ---------- 策略 3: 端点投影兜底 ----------
    projection_used = False
    if not points:
        near_end = Point(path[-1]) if is_entry else Point(path[0])

        _, proj_pt = nearest_points(near_end, exterior)
        proj_dist = near_end.distance(proj_pt)

        # 判据: 端点在水域内 OR 离边界足够近(兼容浮点精度/栅格离散化造成的微小偏差)
        # 端点既不在水域内、距离也超过阈值 -> 真实几何异常,放弃
        if not water_poly.covers(near_end) and proj_dist > _PROJECTION_MAX_DIST:
            role = '入口' if is_entry else '出口'
            print(
                f"    [警告] water_idx={water_idx} {role}路径端点既不在水域内、"
                f"到边界距离也达 {proj_dist:.3f},超出容错范围({_PROJECTION_MAX_DIST}),跳过"
            )
            return None

        points = [[proj_pt.x, proj_pt.y]]
        projection_used = True

    # ---------- 选出入口/出口点 ----------
    candidates = [
        (pt[0], pt[1], float(full_path.project(Point(pt[0], pt[1]))))
        for pt in points
    ]

    _debug_log_candidates(
        candidates, is_entry, water_idx,
        debug_print_intersections, degraded, projection_used,
    )

    best = min(candidates, key=lambda c: c[2]) if is_entry \
        else max(candidates, key=lambda c: c[2])
    return [best[0], best[1]]


def _extract_intersection_point(
    intersection,
    reference_point: List[float]
) -> Optional[List[float]]:
    """
    从交点几何对象中提取最合适的交点坐标（保留用于向后兼容）
    """
    if intersection.is_empty:
        return None

    if intersection.geom_type == 'Point':
        return [intersection.x, intersection.y]
    elif intersection.geom_type == 'MultiPoint':
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


# ============================================================
# 核心函数：单个水域块起点/终点计算
# ============================================================

def calculate_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    water_idx_in_tour: int = None,
    debug_print: bool = False
) -> Tuple[Optional[List[float]], Optional[List[float]], Optional[int]]:
    """
    计算指定水域块的覆盖路径起点和终点。
    """
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return None, None, None

    tour_order = tsp_tour_result['tour_order']
    paths_dict = tsp_tour_result['paths_dict']
    merged_points = tsp_tour_result['merged_points']

    if water_idx_in_tour is None:
        first_idx = tour_order[0]
        if first_idx >= len(merged_points):
            return None, None, None
        first_water_idx = merged_points[first_idx].get('water_idx')

        current_idx = None
        for i in range(1, len(tour_order)):
            point_idx = tour_order[i]
            if point_idx >= len(merged_points):
                continue
            if merged_points[point_idx].get('water_idx') != first_water_idx:
                water_idx_in_tour = i
                current_idx = point_idx
                break

        if current_idx is None:
            return None, None, None
    else:
        if water_idx_in_tour < 0 or water_idx_in_tour >= len(tour_order):
            return None, None, None
        current_idx = tour_order[water_idx_in_tour]

    if current_idx >= len(merged_points):
        return None, None, None

    current_point_info = merged_points[current_idx]
    current_water_idx = current_point_info.get('water_idx')
    current_water_type = current_point_info.get('type')

    actual_water_idx_in_tour = water_idx_in_tour
    block_no = actual_water_idx_in_tour + 1

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

    water_poly = Polygon(
        current_water_info['outer'],
        holes=current_water_info.get('holes', [])
    )

    # ---------- 判断是否是聚类水域块 ----------
    cluster_boundary_poly = None
    hull_poly = None
    if core_points_data and current_water_type == 'with_hole':
        clusters_map = core_points_data.get('water_with_hole_clusters', {})
        if current_water_idx in clusters_map:
            clusters = clusters_map[current_water_idx]
            current_point = merged_points[current_idx]['point']
            current_pt_geom = Point(current_point[0], current_point[1])

            for cluster in clusters:
                hull_coords = cluster.get('hull')
                if not (hull_coords and len(hull_coords) >= 3):
                    continue
                try:
                    hull_poly_candidate = Polygon(hull_coords)
                    if hull_poly_candidate.is_empty:
                        continue
                    if not hull_poly_candidate.contains(current_pt_geom):
                        continue

                    hull_poly = hull_poly_candidate
                    intersection = hull_poly.intersection(water_poly)
                    if intersection.is_empty:
                        continue
                    #陈改1
                    cluster_boundary_poly = _pick_polygon_containing_point(
                        intersection, current_pt_geom
                    )
                    if cluster_boundary_poly is not None:
                        break
                    cluster_boundary_poly = _pick_nearest_polygon(
                        intersection, current_pt_geom
                    )
                    if cluster_boundary_poly is not None:
                        break
                except Exception:
                    continue

    boundary_poly = cluster_boundary_poly if cluster_boundary_poly is not None else water_poly

    # ---------- 计算起点（入口点）----------
    start_point = None
    if actual_water_idx_in_tour == 0:
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
                if entry_path_key == f"{current_idx}->{last_idx}":
                    entry_path = entry_path[::-1]
                ref_point = merged_points[last_idx]['point']
                start_point = _find_path_water_intersection(
                    entry_path, boundary_poly, ref_point,
                    is_entry=True, water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if start_point is None and cluster_boundary_poly is not None:
                    start_point = _find_path_water_intersection(
                        entry_path, water_poly, ref_point,
                        is_entry=True, water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )

        if start_point is None:
            pt = merged_points[current_idx].get('point')
            if pt is not None:
                start_point = [float(pt[0]), float(pt[1])]
            if debug_print and start_point:
                print(f"    [第一块起点] 第{block_no}块 求交无结果，回退到骨架代表点: ({start_point[0]:.2f}, {start_point[1]:.2f})")

    else:
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
                # 若 key 是反向的(current->prev),反转 path,保证方向是 prev->current
                if entry_path_key == f"{current_idx}->{prev_idx}":
                    entry_path = entry_path[::-1]
                ref_point = merged_points[prev_idx]['point']

                # 优先用 "聚类边界 or 水域边界" 求交(逐级 fallback)
                # 注意: 不能把两者合并后一起求交,因为聚类外的 water_poly 外边界
                # 可能离实际聚类很远,会导致入口点错误地落在聚类范围外。
                # 出口路径也遵循同样的原则(见下方 "计算终点" 部分)。
                start_point = _find_path_water_intersection(
                    entry_path, boundary_poly, ref_point,
                    is_entry=True, water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if start_point is None and cluster_boundary_poly is not None:
                    start_point = _find_path_water_intersection(
                        entry_path, water_poly, ref_point,
                        is_entry=True, water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )

    # ---------- 计算终点（出口点）----------
    exit_point = None
    if actual_water_idx_in_tour < len(tour_order) - 1:
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
                # 若 key 是反向的(next->current),反转 path,保证方向是 current->next
                if exit_path_key == f"{next_idx}->{current_idx}":
                    exit_path = exit_path[::-1]
                exit_point = _find_path_water_intersection(
                    exit_path, boundary_poly, merged_points[next_idx]['point'],
                    is_entry=False, water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if exit_point is None and cluster_boundary_poly is not None:
                    exit_point = _find_path_water_intersection(
                        exit_path, water_poly, merged_points[next_idx]['point'],
                        is_entry=False, water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )
    else:
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
                # 若 key 是反向的(first->current),反转 path,保证方向是 current->first
                if exit_path_key == f"{first_idx}->{current_idx}":
                    exit_path = exit_path[::-1]
                exit_point = _find_path_water_intersection(
                    exit_path, boundary_poly, merged_points[first_idx]['point'],
                    is_entry=False, water_idx=current_water_idx,
                    debug_print_intersections=debug_print,
                )
                if exit_point is None and cluster_boundary_poly is not None:
                    exit_point = _find_path_water_intersection(
                        exit_path, water_poly, merged_points[first_idx]['point'],
                        is_entry=False, water_idx=current_water_idx,
                        debug_print_intersections=debug_print,
                    )

    # ---------- 手动矫正 ----------
    overrides = MANUAL_ENTRY_EXIT_OVERRIDES.get(block_no)
    if overrides:
        manual_start = overrides.get('start')
        manual_exit = overrides.get('exit')
        if manual_start is not None:
            start_point = manual_start
        if manual_exit is not None:
            exit_point = manual_exit
        start_str = f"({start_point[0]:.2f}, {start_point[1]:.2f})" if start_point else "None"
        exit_str = f"({exit_point[0]:.2f}, {exit_point[1]:.2f})" if exit_point else "None"
        print(f"      [手动矫正] 第{block_no}块(water_idx={current_water_idx}): 起点={start_str} 终点={exit_str}")

    return start_point, exit_point, current_water_idx


# ============================================================
# 批量计算函数
# ============================================================

def compute_all_water_entry_exit(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None,
    debug_print: bool = False,
) -> Dict[int, Dict]:
    """
    计算所有水域块的起点/终点并合并为一份字典（含手动覆盖）。
    """
    all_water_entry_exit = {}
    if not tsp_tour_result or len(tsp_tour_result.get("tour_order", [])) < 2:
        return all_water_entry_exit

    tour_order = tsp_tour_result["tour_order"]
    first_block_start, first_block_exit, _ = calculate_water_entry_exit_points(
        tsp_tour_result, water_no_hole, water_with_hole,
        core_points_data=core_points_data,
        water_idx_in_tour=0, debug_print=False,
    )

    all_water_entry_exit_for_viz = {}
    for i in range(1, len(tour_order)):
        debug_this = (i + 1) == 52
        start_pt, exit_pt, calc_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result, water_no_hole, water_with_hole,
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


def calculate_all_water_entry_exit_points(
    tsp_tour_result: Dict,
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    core_points_data: Optional[Dict] = None
) -> Dict[int, Dict]:
    """一次性计算所有中间水域块（非第一个和最后一个）的覆盖路径起点和终点。"""
    result = {}
    if not tsp_tour_result or len(tsp_tour_result.get('tour_order', [])) < 2:
        return result

    tour_order = tsp_tour_result['tour_order']
    merged_points = tsp_tour_result['merged_points']

    water_indices_map = {}
    for i, point_idx in enumerate(tour_order):
        if point_idx >= len(merged_points):
            continue
        water_idx = merged_points[point_idx].get('water_idx')
        if water_idx not in water_indices_map:
            water_indices_map[water_idx] = []
        water_indices_map[water_idx].append(i)

    first_water_idx = merged_points[tour_order[0]].get('water_idx')
    last_water_idx = merged_points[tour_order[-1]].get('water_idx')

    for water_idx, tour_indices in water_indices_map.items():
        if water_idx == first_water_idx:
            continue
        is_last_water = (water_idx == last_water_idx)
        water_tour_idx = min(tour_indices)
        start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result, water_no_hole, water_with_hole,
            core_points_data=core_points_data,
            water_idx_in_tour=water_tour_idx
        )

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
    """计算所有聚类水域块的覆盖路径起点和终点。"""
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

    clustered_water_indices_map = {}
    for i, point_idx in enumerate(tour_order):
        if point_idx >= len(merged_points):
            continue
        point_info = merged_points[point_idx]
        water_idx = point_info.get('water_idx')
        water_type = point_info.get('type')
        if water_type == 'with_hole' and water_idx in clusters_map:
            if water_idx not in clustered_water_indices_map:
                clustered_water_indices_map[water_idx] = {}
            clustered_water_indices_map[water_idx][point_idx] = i

    for water_idx, point_indices_map in clustered_water_indices_map.items():
        for point_idx, tour_idx in point_indices_map.items():
            is_last_point = (tour_idx == len(tour_order) - 1)
            start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
                tsp_tour_result, water_no_hole, water_with_hole,
                core_points_data=core_points_data,
                water_idx_in_tour=tour_idx, debug_print=debug_print
            )

            if start_point is not None and calculated_water_idx == water_idx:
                if is_last_point:
                    pass
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
    """计算所有单连通水域块的覆盖路径起点和终点（跳过第一个水域块）。"""
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
            continue

        is_last_point = (i == len(tour_order) - 1)
        start_point, exit_point, calculated_water_idx = calculate_water_entry_exit_points(
            tsp_tour_result, water_no_hole, water_with_hole,
            core_points_data=core_points_data,
            water_idx_in_tour=i, debug_print=debug_print
        )

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