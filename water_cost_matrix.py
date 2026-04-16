"""
水域块采样点成本矩阵计算模块（通用）

本模块用于计算单块水域采样点之间的能量成本矩阵：
- 计算任意两点之间的能量成本
- 考虑线段是否经过陆地（包括孔洞）
- 生成成本矩阵用于TSP求解
"""

from typing import List, Optional, Dict, Tuple
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.prepared import prep
from shapely.ops import unary_union
import time


def calculate_sampling_points_cost_matrix(
    sampling_points: List[List[float]],
    water_polygon: Polygon,
    boundary_polygon: Optional[Polygon] = None
) -> Tuple[Optional[np.ndarray], Dict]:
    """
    计算采样点之间的能量成本矩阵
    
    成本计算规则：
    - 如果两点连线完全在水域内：成本 = 距离 × 1 × 1000（取整数）
    - 如果经过陆地（包括孔洞）：
      成本 = (水域距离 × 1 + 陆地距离 × 10) × 1000（取整数）
    - 如果经过边界外：成本 = (水域距离 × 1 + 陆地距离 × 10 + 边界外距离 × 1000) × 1000（取整数）
    
    :param sampling_points: 采样点列表 [[x, y], ...]，起点在索引0，退出点在索引1
    :param water_polygon: 水域多边形（Shapely Polygon，包含孔洞）
    :param boundary_polygon: 地图边界多边形（可选，用于检查边界外）
    :return: (成本矩阵, 详细信息字典)
    """
    n = len(sampling_points)
    if n < 2:
        print("  ✗ 采样点数量不足，无法计算成本矩阵")
        return None, {}
    
    # 准备多边形（用于快速contains检查）
    prepared_water_polygon = prep(water_polygon)
    prepared_boundary_polygon = prep(boundary_polygon) if boundary_polygon else None
    
    # 初始化成本矩阵
    cost_matrix = np.full((n, n), np.inf, dtype=np.float64)
    
    # 计算所有点对之间的成本
    total_pairs = n * (n - 1) // 2
    calculated = 0
    t0 = time.time()
    last_report = t0
    # 约每 3 秒或每 2% 打印一次（取较稀疏者），避免刷屏但能看出是否在跑
    report_every_pairs = max(1, total_pairs // 50)  # 2%
    print(f"  计算水域块成本矩阵：点数={n}，点对数={total_pairs}（O(n^2) 可能较慢）")
    
    for i in range(n):
        for j in range(i + 1, n):
            point_i = sampling_points[i]
            point_j = sampling_points[j]
            
            # 计算两点之间的成本
            cost = _calculate_point_pair_cost(
                point_i, point_j,
                water_polygon, prepared_water_polygon,
                boundary_polygon, prepared_boundary_polygon
            )
            
            if cost is not None:
                # 填充对称矩阵
                cost_matrix[i, j] = cost
                cost_matrix[j, i] = cost
                calculated += 1

            # 进度打印
            if calculated % report_every_pairs == 0:
                now = time.time()
                if now - last_report >= 3.0:
                    pct = 100.0 * calculated / max(1, total_pairs)
                    elapsed = now - t0
                    rate = calculated / max(1e-9, elapsed)
                    eta = (total_pairs - calculated) / max(1e-9, rate)
                    print(f"    进度: {pct:5.1f}%  ({calculated}/{total_pairs})  用时={elapsed:,.1f}s  预计剩余={eta:,.1f}s")
                    last_report = now
    
    # 对角线元素为0（点到自己的距离）
    np.fill_diagonal(cost_matrix, 0)
    
    # 统计信息
    valid_costs = cost_matrix[cost_matrix != np.inf]
    info_dict = {
        'total_points': n,
        'calculated_pairs': calculated,
        'total_pairs': total_pairs,
        'min_cost': float(np.min(valid_costs)) if len(valid_costs) > 0 else None,
        'max_cost': float(np.max(valid_costs)) if len(valid_costs) > 0 else None,
        'avg_cost': float(np.mean(valid_costs)) if len(valid_costs) > 0 else None
    }

    elapsed_total = time.time() - t0
    print(f"  ✓ 成本矩阵计算完成：有效点对={calculated}/{total_pairs}，用时={elapsed_total:,.1f}s")
    
    return cost_matrix, info_dict


def _calculate_point_pair_cost(
    point_a: List[float],
    point_b: List[float],
    water_polygon: Polygon,
    prepared_water_polygon,
    boundary_polygon: Optional[Polygon],
    prepared_boundary_polygon
) -> Optional[float]:
    """
    计算两个点之间的能量成本
    
    :param point_a: 点A坐标 [x, y]
    :param point_b: 点B坐标 [x, y]
    :param water_polygon: 水域多边形
    :param prepared_water_polygon: 准备好的水域多边形（用于快速检查）
    :param boundary_polygon: 地图边界多边形（可选）
    :param prepared_boundary_polygon: 准备好的边界多边形（可选）
    :return: 成本值（浮点数），如果计算失败返回None
    """
    try:
        # 创建线段
        line = LineString([point_a, point_b])
        total_distance = line.length
        
        if total_distance < 1e-10:
            # 两点重合，成本为0
            return 0.0
        
        # 找到线段与水域边界的交点
        water_intersections = line.intersection(water_polygon.boundary)
        
        # 找到线段与地图边界的交点（如果提供了边界）
        boundary_intersections = None
        if boundary_polygon:
            boundary_intersections = line.intersection(boundary_polygon.boundary)
        
        # 收集所有分段点（起点、终点、所有交点）
        segment_points = [Point(point_a), Point(point_b)]
        
        # 添加水域边界交点
        if not water_intersections.is_empty:
            if water_intersections.geom_type == 'Point':
                segment_points.append(water_intersections)
            elif water_intersections.geom_type == 'MultiPoint':
                segment_points.extend(water_intersections.geoms)
        
        # 添加地图边界交点（如果存在）
        if boundary_intersections and not boundary_intersections.is_empty:
            if boundary_intersections.geom_type == 'Point':
                if boundary_intersections not in segment_points:
                    segment_points.append(boundary_intersections)
            elif boundary_intersections.geom_type == 'MultiPoint':
                for p in boundary_intersections.geoms:
                    if p not in segment_points:
                        segment_points.append(p)
        
        # 按距离起点的距离排序分段点
        def distance_to_start(point):
            return Point(point_a).distance(point)
        
        segment_points.sort(key=distance_to_start)
        
        # 计算每段的类型和长度
        water_distance = 0.0
        land_distance = 0.0
        boundary_out_distance = 0.0
        
        for i in range(len(segment_points) - 1):
            seg_start = segment_points[i]
            seg_end = segment_points[i + 1]
            seg_length = seg_start.distance(seg_end)
            
            # 判断该段是否在水域内
            # 方法：检查该段的中点是否在水域内
            seg_mid = Point(
                (seg_start.x + seg_end.x) / 2,
                (seg_start.y + seg_end.y) / 2
            )
            
            # 检查是否在边界内
            in_boundary = True
            if prepared_boundary_polygon:
                in_boundary = prepared_boundary_polygon.contains(seg_mid)
            
            if not in_boundary:
                # 边界外
                boundary_out_distance += seg_length
            elif prepared_water_polygon.contains(seg_mid):
                # 在水域内
                water_distance += seg_length
            else:
                # 在陆地内（包括孔洞）
                land_distance += seg_length
        
        # 计算成本
        cost = (water_distance * 1.0 + land_distance * 10.0 + boundary_out_distance * 1000.0) * 1000.0
        
        # 取整数
        cost_int = int(round(cost))
        
        return float(cost_int)
        
    except Exception as e:
        print(f"    ⚠ 计算点对成本时出错: {e}")
        return None
