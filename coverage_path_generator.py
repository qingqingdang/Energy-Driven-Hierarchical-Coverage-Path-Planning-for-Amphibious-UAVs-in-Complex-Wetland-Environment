"""
覆盖巡检路径生成模块 (V0.1 - 方法1)

本模块实现基于水域块访问顺序的覆盖巡检路径生成：
- 为每个水域块生成覆盖巡检路径
- 路径起点和终点基于骨架路径与水域边界的交点
- 支持之字形扫描和螺旋扫描（待实现）

方法1逻辑：
1. 第一个水域块：
   - 起点：从水域块2几何中心指向水域块1几何中心的射线，与水域块1边界的交点
   - 终点：从水域块1几何中心到水域块2几何中心的骨架路径，与水域块1边界的交点
2. 其他水域块：
   - 起点和终点：骨架路径与该水域块边界的两个交点
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Optional
import numpy as np
from shapely.geometry import Polygon, Point, LineString, LinearRing
from shapely.ops import unary_union


def generate_coverage_paths(
    tsp_tour_result: dict,
    env_data: dict,
    core_points_data: dict,
    scan_mode: str = 'zigzag'
) -> Dict[str, any]:
    """
    为所有水域块生成覆盖巡检路径
    
    :param tsp_tour_result: TSP路径结果，包含:
        - tour_order: 访问顺序列表（点索引）
        - paths_dict: 路径字典 {f"{i}->{j}": {'path': list, ...}}
        - merged_points: 合并后的核心点列表
    :param env_data: 环境数据，包含水域信息
    :param core_points_data: 核心点数据
    :param scan_mode: 扫描模式 ('zigzag' 或 'spiral')
    :return: 覆盖路径结果字典
    """
    tour_order = tsp_tour_result.get('tour_order')
    paths_dict = tsp_tour_result.get('paths_dict')
    merged_points = tsp_tour_result.get('merged_points')
    
    if not tour_order or not paths_dict or not merged_points:
        print("  ✗ TSP路径数据不完整，无法生成覆盖路径")
        return None
    
    if len(tour_order) < 2:
        print("  ✗ TSP访问顺序点数不足，无法生成覆盖路径")
        return None
    
    print(f"\n生成覆盖巡检路径（方法1，扫描模式: {scan_mode}）...")
    print(f"  总水域块数: {len(tour_order)}")
    
    # 获取水域数据
    water_with_hole = env_data.get('water_with_hole', [])
    water_no_hole = env_data.get('water_no_hole', [])
    
    # 创建水域索引映射（从核心点索引到水域信息）
    water_info_map = _create_water_info_map(merged_points, water_with_hole, water_no_hole)
    
    coverage_paths = []
    total_coverage_points = 0
    
    # 处理每个水域块
    for seg_idx in range(len(tour_order)):
        from_idx = tour_order[seg_idx]
        to_idx = tour_order[(seg_idx + 1) % len(tour_order)]  # 最后一段返回起点
        
        # 获取当前水域块信息
        water_info = water_info_map.get(from_idx)
        if not water_info:
            print(f"    ⚠ 警告: 未找到水域块 {from_idx} 的信息，跳过")
            continue
        
        # 计算起点和终点
        if seg_idx == 0:
            # 第一个水域块：特殊处理
            start_point, end_point = _calculate_first_water_entry_exit(
                from_idx, to_idx, 
                merged_points, paths_dict, 
                water_info
            )
        else:
            # 其他水域块：骨架路径与边界的交点
            start_point, end_point = _calculate_water_entry_exit(
                from_idx, to_idx,
                merged_points, paths_dict,
                water_info
            )
        
        if start_point is None or end_point is None:
            print(f"    ⚠ 警告: 水域块 {from_idx} 无法计算入口/出口点，跳过")
            continue
        
        # 确保起点和终点是列表格式（如果不是）
        # 处理 Shapely Point 对象
        if hasattr(start_point, 'x') and hasattr(start_point, 'y'):
            start_point = [float(start_point.x), float(start_point.y)]
        elif not isinstance(start_point, (list, tuple, np.ndarray)):
            print(f"    ⚠ 警告: 水域块 {from_idx} 起点格式错误: {type(start_point)}")
            continue
        
        if hasattr(end_point, 'x') and hasattr(end_point, 'y'):
            end_point = [float(end_point.x), float(end_point.y)]
        elif not isinstance(end_point, (list, tuple, np.ndarray)):
            print(f"    ⚠ 警告: 水域块 {from_idx} 终点格式错误: {type(end_point)}")
            continue
        
        # 转换为列表格式
        start_point = list(start_point)
        end_point = list(end_point)
        
        # 生成覆盖路径
        if scan_mode == 'zigzag':
            coverage_path = _generate_zigzag_coverage(
                water_info, start_point, end_point
            )
        elif scan_mode == 'spiral':
            coverage_path = _generate_spiral_coverage(
                water_info, start_point, end_point
            )
        else:
            print(f"    ✗ 未知的扫描模式: {scan_mode}")
            continue
        
        if not coverage_path:
            print(f"    ⚠ 警告: 水域块 {from_idx} 无法生成覆盖路径")
            continue
        
        coverage_paths.append({
            'water_idx': water_info['water_idx'],
            'water_type': water_info['type'],
            'start_point': start_point,
            'end_point': end_point,
            'coverage_path': coverage_path,
            'path_length': len(coverage_path)
        })
        
        total_coverage_points += len(coverage_path)
        print(f"    水域块 {from_idx} (P{water_info['water_idx']}): "
              f"生成 {len(coverage_path)} 个覆盖点")
    
    print(f"  ✓ 覆盖路径生成完成！总覆盖点数: {total_coverage_points}")
    
    return {
        'coverage_paths': coverage_paths,
        'total_points': total_coverage_points,
        'scan_mode': scan_mode,
        'water_count': len(coverage_paths)
    }


def _create_water_info_map(merged_points, water_with_hole, water_no_hole):
    """
    创建从核心点索引到水域信息的映射
    
    :return: {point_idx: {'water_idx': int, 'type': str, 'outer': list, 'holes': list}}
    """
    water_info_map = {}
    
    for point_idx, point_info in enumerate(merged_points):
        water_idx = point_info.get('water_idx')
        point_type = point_info.get('type')
        
        # 查找对应的水域信息
        water_info = None
        if point_type == 'no_hole':
            for water in water_no_hole:
                if water['idx'] == water_idx:
                    water_info = {
                        'water_idx': water_idx,
                        'type': 'no_hole',
                        'outer': water['outer'],
                        'holes': []
                    }
                    break
        elif point_type == 'with_hole':
            for water in water_with_hole:
                if water['idx'] == water_idx:
                    water_info = {
                        'water_idx': water_idx,
                        'type': 'with_hole',
                        'outer': water['outer'],
                        'holes': water['holes']
                    }
                    break
        
        if water_info:
            water_info_map[point_idx] = water_info
    
    return water_info_map


def _calculate_first_water_entry_exit(
    from_idx: int, to_idx: int,
    merged_points: List[dict],
    paths_dict: Dict[str, dict],
    water_info: dict
) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """
    计算第一个水域块的入口和出口点
    
    入口点：从水域块2几何中心指向水域块1几何中心的射线，与水域块1边界的交点
    出口点：从水域块1几何中心到水域块2几何中心的骨架路径，与水域块1边界的交点
    """
    # 获取几何中心
    water1_center = merged_points[from_idx]['point']
    water2_center = merged_points[to_idx]['point']
    
    # 创建水域多边形
    water_poly = Polygon(water_info['outer'], holes=water_info['holes'])
    
    # 计算入口点：从water2_center指向water1_center的射线
    # 创建一条足够长的射线
    dx = water1_center[0] - water2_center[0]
    dy = water1_center[1] - water2_center[1]
    length = np.sqrt(dx**2 + dy**2)
    
    if length == 0:
        return None, None
    
    # 归一化方向向量
    dx_norm = dx / length
    dy_norm = dy / length
    
    # 创建射线（从water2_center出发，指向water1_center，延长足够远）
    ray_length = length * 10  # 延长10倍确保穿过水域
    ray_end = [
        water2_center[0] + dx_norm * ray_length,
        water2_center[1] + dy_norm * ray_length
    ]
    ray = LineString([water2_center, ray_end])
    
    # 计算射线与水域边界的交点
    entry_point = _find_line_polygon_intersection(ray, water_poly, water2_center)
    
    # 计算出口点：骨架路径与水域边界的交点
    path_key = f"{from_idx}->{to_idx}"
    if path_key not in paths_dict:
        path_key = f"{to_idx}->{from_idx}"
    
    if path_key not in paths_dict:
        return None, None
    
    skeleton_path = paths_dict[path_key].get('path', [])
    if not skeleton_path:
        return None, None
    
    # 创建骨架路径线段
    skeleton_line = LineString(skeleton_path)
    
    # 找到骨架路径与水域边界的交点（靠近出口的点）
    exit_point = _find_line_polygon_intersection(
        skeleton_line, water_poly, water1_center, prefer_far=True
    )
    
    return entry_point, exit_point


def _calculate_water_entry_exit(
    from_idx: int, to_idx: int,
    merged_points: List[dict],
    paths_dict: Dict[str, dict],
    water_info: dict
) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """
    计算其他水域块的入口和出口点
    
    起点和终点：骨架路径与该水域块边界的两个交点
    """
    # 获取骨架路径
    path_key = f"{from_idx}->{to_idx}"
    if path_key not in paths_dict:
        path_key = f"{to_idx}->{from_idx}"
    
    if path_key not in paths_dict:
        return None, None
    
    skeleton_path = paths_dict[path_key].get('path', [])
    if not skeleton_path:
        return None, None
    
    # 创建水域多边形
    water_poly = Polygon(water_info['outer'], holes=water_info['holes'])
    
    # 创建骨架路径线段
    skeleton_line = LineString(skeleton_path)
    
    # 获取起点和终点的几何中心（用于确定哪个交点是入口，哪个是出口）
    from_center = merged_points[from_idx]['point']
    to_center = merged_points[to_idx]['point']
    
    # 找到骨架路径与水域边界的两个交点
    intersections = _find_all_line_polygon_intersections(skeleton_line, water_poly)
    
    if len(intersections) < 2:
        # 如果交点不足2个，尝试使用路径的起点和终点
        if len(skeleton_path) >= 2:
            # 检查路径起点和终点是否在水域内
            start_in = water_poly.contains(Point(skeleton_path[0]))
            end_in = water_poly.contains(Point(skeleton_path[-1]))
            
            if start_in and end_in:
                # 如果都在水域内，使用路径的起点和终点
                return skeleton_path[0], skeleton_path[-1]
            elif start_in:
                # 起点在水域内，终点不在，找出口点
                exit_point = _find_line_polygon_intersection(
                    skeleton_line, water_poly, to_center, prefer_far=True
                )
                return skeleton_path[0], exit_point
            elif end_in:
                # 终点在水域内，起点不在，找入口点
                entry_point = _find_line_polygon_intersection(
                    skeleton_line, water_poly, from_center, prefer_far=False
                )
                return entry_point, skeleton_path[-1]
        
        return None, None
    
    # 根据与几何中心的距离确定入口和出口
    # 入口点：靠近from_center的交点
    # 出口点：靠近to_center的交点
    entry_point = min(intersections, 
                     key=lambda p: np.sqrt((p[0] - from_center[0])**2 + 
                                          (p[1] - from_center[1])**2))
    exit_point = min(intersections,
                    key=lambda p: np.sqrt((p[0] - to_center[0])**2 + 
                                         (p[1] - to_center[1])**2))
    
    # 如果两个交点相同，使用路径的起点和终点
    if entry_point == exit_point:
        if len(skeleton_path) >= 2:
            return skeleton_path[0], skeleton_path[-1]
        return None, None
    
    return entry_point, exit_point


def _find_line_polygon_intersection(
    line: LineString,
    polygon: Polygon,
    reference_point: List[float],
    prefer_far: bool = False
) -> Optional[List[float]]:
    """
    找到线段与多边形外边界（不含孔洞）的交点
    
    :param line: 线段
    :param polygon: 多边形
    :param reference_point: 参考点（用于选择最合适的交点）
    :param prefer_far: 如果为True，选择距离参考点较远的交点；否则选择较近的
    :return: 交点坐标 [x, y]，如果没有交点返回None
    """
    # 只使用外边界（exterior），排除孔洞边界
    exterior = LineString(polygon.exterior.coords)
    intersection = line.intersection(exterior)
    
    if intersection.is_empty:
        return None
    
    # 处理不同的几何类型
    if intersection.geom_type == 'Point':
        return [intersection.x, intersection.y]
    elif intersection.geom_type == 'MultiPoint':
        points = list(intersection.geoms)
        if not points:
            return None
        # 根据参考点选择最合适的点
        if prefer_far:
            selected_point = max(points, 
                      key=lambda p: np.sqrt((p.x - reference_point[0])**2 + 
                                           (p.y - reference_point[1])**2))
        else:
            selected_point = min(points,
                     key=lambda p: np.sqrt((p.x - reference_point[0])**2 + 
                                         (p.y - reference_point[1])**2))
        # 确保返回列表格式
        return [selected_point.x, selected_point.y]
    elif intersection.geom_type == 'LineString':
        # 如果交点是线段，取中点
        coords = list(intersection.coords)
        if coords:
            mid_idx = len(coords) // 2
            return list(coords[mid_idx])
    
    return None


def _find_all_line_polygon_intersections(
    line: LineString,
    polygon: Polygon
) -> List[List[float]]:
    """
    找到线段与多边形外边界（不含孔洞）的所有交点
    
    :return: 交点列表 [[x, y], ...]
    """
    # 只使用外边界（exterior），排除孔洞边界
    exterior = LineString(polygon.exterior.coords)
    intersection = line.intersection(exterior)
    
    if intersection.is_empty:
        return []
    
    points = []
    if intersection.geom_type == 'Point':
        points.append([intersection.x, intersection.y])
    elif intersection.geom_type == 'MultiPoint':
        points.extend([[p.x, p.y] for p in intersection.geoms])
    elif intersection.geom_type == 'LineString':
        # 如果交点是线段，取起点和终点
        coords = list(intersection.coords)
        if len(coords) >= 2:
            points.append(list(coords[0]))
            points.append(list(coords[-1]))
    
    return points


def _generate_zigzag_coverage(
    water_info: dict,
    start_point: List[float],
    end_point: List[float],
    grid_size: float = 4.0
) -> Optional[List[List[float]]]:
    """
    生成之字形覆盖路径
    
    :param grid_size: 栅格大小（米），用于确定扫描线间距
    :return: 覆盖路径点列表 [[x, y], ...]
    """
    # 创建水域多边形
    water_poly = Polygon(water_info['outer'], holes=water_info['holes'])
    
    if water_poly.is_empty or water_poly.area < 1.0:
        return None
    
    # 1. 确定扫描方向（从start_point到end_point的方向）
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    scan_length = np.sqrt(dx**2 + dy**2)
    
    if scan_length < 1.0:
        # 如果起点和终点太近，使用水域的主方向
        bounds = water_poly.bounds
        dx = bounds[2] - bounds[0]  # max_x - min_x
        dy = bounds[3] - bounds[1]  # max_y - min_y
        scan_length = np.sqrt(dx**2 + dy**2)
    
    # 扫描方向向量（归一化）
    if scan_length > 0:
        scan_dir = [dx / scan_length, dy / scan_length]
    else:
        scan_dir = [1.0, 0.0]  # 默认X方向
    
    # 垂直方向（用于生成扫描线）
    perp_dir = [-scan_dir[1], scan_dir[0]]
    
    # 2. 获取水域边界框
    bounds = water_poly.bounds
    minx, miny, maxx, maxy = bounds
    
    # 3. 计算扫描线范围（垂直于扫描方向）
    # 投影边界框到垂直方向
    corners = [
        [minx, miny], [maxx, miny],
        [maxx, maxy], [minx, maxy]
    ]
    proj_values = [np.dot(corner, perp_dir) for corner in corners]
    proj_min = min(proj_values)
    proj_max = max(proj_values)
    
    # 4. 生成扫描线（间距为grid_size）
    num_lines = int((proj_max - proj_min) / grid_size) + 1
    scan_lines = []
    
    for i in range(num_lines):
        proj_value = proj_min + i * grid_size
        
        # 计算扫描线的两个端点（在边界框外，确保穿过整个水域）
        # 找到垂直于扫描方向，投影值为proj_value的直线
        # 直线方程：perp_dir[0] * x + perp_dir[1] * y = proj_value
        
        # 计算直线与边界框的交点
        line_points = []
        
        # 与左边界（x = minx）的交点
        if abs(perp_dir[1]) > 1e-6:
            y = (proj_value - perp_dir[0] * minx) / perp_dir[1]
            if miny <= y <= maxy:
                line_points.append([minx, y])
        
        # 与右边界（x = maxx）的交点
        if abs(perp_dir[1]) > 1e-6:
            y = (proj_value - perp_dir[0] * maxx) / perp_dir[1]
            if miny <= y <= maxy:
                line_points.append([maxx, y])
        
        # 与下边界（y = miny）的交点
        if abs(perp_dir[0]) > 1e-6:
            x = (proj_value - perp_dir[1] * miny) / perp_dir[0]
            if minx <= x <= maxx:
                line_points.append([x, miny])
        
        # 与上边界（y = maxy）的交点
        if abs(perp_dir[0]) > 1e-6:
            x = (proj_value - perp_dir[1] * maxy) / perp_dir[0]
            if minx <= x <= maxx:
                line_points.append([x, maxy])
        
        # 去重并排序
        if len(line_points) >= 2:
            # 去重
            unique_points = []
            for p in line_points:
                is_duplicate = False
                for up in unique_points:
                    if np.sqrt((p[0] - up[0])**2 + (p[1] - up[1])**2) < 1e-6:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    unique_points.append(p)
            
            if len(unique_points) >= 2:
                # 创建扫描线（延长到边界框外）
                line_start = unique_points[0]
                line_end = unique_points[-1]
                
                # 延长线段确保穿过整个水域
                extend_length = scan_length * 2
                line_vec = [line_end[0] - line_start[0], line_end[1] - line_start[1]]
                line_len = np.sqrt(line_vec[0]**2 + line_vec[1]**2)
                if line_len > 0:
                    line_vec_norm = [line_vec[0] / line_len, line_vec[1] / line_len]
                    extended_start = [
                        line_start[0] - line_vec_norm[0] * extend_length,
                        line_start[1] - line_vec_norm[1] * extend_length
                    ]
                    extended_end = [
                        line_end[0] + line_vec_norm[0] * extend_length,
                        line_end[1] + line_vec_norm[1] * extend_length
                    ]
                    scan_lines.append(LineString([extended_start, extended_end]))
    
    # 5. 计算每条扫描线与水域的交点
    coverage_points = []
    for i, scan_line in enumerate(scan_lines):
        intersection = scan_line.intersection(water_poly)
        
        if intersection.is_empty:
            continue
        
        # 提取交点
        line_points = []
        if intersection.geom_type == 'Point':
            line_points.append([intersection.x, intersection.y])
        elif intersection.geom_type == 'MultiPoint':
            line_points.extend([[p.x, p.y] for p in intersection.geoms])
        elif intersection.geom_type == 'LineString':
            coords = list(intersection.coords)
            line_points.extend([[c[0], c[1]] for c in coords])
        elif intersection.geom_type == 'MultiLineString':
            for line in intersection.geoms:
                coords = list(line.coords)
                line_points.extend([[c[0], c[1]] for c in coords])
        
        if len(line_points) >= 2:
            # 按扫描方向排序点
            line_points.sort(key=lambda p: np.dot(p, scan_dir))
            
            # 之字形模式：奇数行正向，偶数行反向
            if i % 2 == 1:
                line_points.reverse()
            
            coverage_points.extend(line_points)
        elif len(line_points) == 1:
            coverage_points.append(line_points[0])
    
    if not coverage_points:
        return None
    
    # 6. 确保起点和终点在路径中（如果可能）
    # 找到最接近起点和终点的覆盖点
    if len(coverage_points) > 0:
        start_idx = min(range(len(coverage_points)),
                       key=lambda i: np.sqrt((coverage_points[i][0] - start_point[0])**2 +
                                            (coverage_points[i][1] - start_point[1])**2))
        end_idx = min(range(len(coverage_points)),
                     key=lambda i: np.sqrt((coverage_points[i][0] - end_point[0])**2 +
                                          (coverage_points[i][1] - end_point[1])**2))
        
        # 如果起点和终点不在路径中，尝试插入
        if start_idx != 0:
            # 将起点插入到路径开始
            coverage_points.insert(0, start_point)
        if end_idx != len(coverage_points) - 1:
            # 将终点插入到路径末尾
            coverage_points.append(end_point)
    
    return coverage_points


def _generate_spiral_coverage(
    water_info: dict,
    start_point: List[float],
    end_point: List[float],
    grid_size: float = 4.0
) -> Optional[List[List[float]]]:
    """
    生成螺旋覆盖路径
    
    :param grid_size: 栅格大小（米）
    :return: 覆盖路径点列表 [[x, y], ...]
    """
    # TODO: 实现螺旋扫描算法
    print(f"    ⚠ 螺旋扫描算法待实现")
    return None


__all__ = ['generate_coverage_paths']


