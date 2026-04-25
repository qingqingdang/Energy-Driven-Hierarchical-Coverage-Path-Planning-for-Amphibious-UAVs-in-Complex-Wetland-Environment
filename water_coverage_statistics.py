"""
水域块覆盖路径统计模块（通用）

本模块用于统计单块水域覆盖路径的相关信息：
1. 路径长度
2. 水域块面积
3. 覆盖率计算
4. 转弯次数评估
"""

from typing import List, Optional, Dict, Tuple
import numpy as np
from shapely.geometry import Polygon


def calculate_water_coverage_statistics(
    sampling_points: Optional[List[List[float]]],
    tour_order: Optional[List[int]],
    tsp_tour_result: Optional[Dict],
    water_no_hole: List[Dict],
    water_with_hole: List[Dict],
    coverage_width: float = 4.0
) -> Optional[Dict]:
    """
    计算单块水域覆盖路径的统计信息（常用于第 1 块，也可用于任意块）。

    :param sampling_points: 采样点列表 [[x, y], ...]
    :param tour_order: TSP 访问顺序列表（点索引，Python 0-based）
    :param tsp_tour_result: TSP 骨架路径结果字典（用于取第 1 块水域多边形）
    :param water_no_hole: 无孔洞水域列表
    :param water_with_hole: 有孔洞水域列表
    :param coverage_width: 覆盖宽度（米），默认 4.0
    :return: 统计信息字典，失败返回 None
    """
    if sampling_points is None or tour_order is None:
        return None

    if len(sampling_points) == 0 or len(tour_order) < 2:
        return None

    try:
        # 1. 计算路径长度
        path_length = 0.0
        for i in range(len(tour_order) - 1):
            from_idx = tour_order[i]
            to_idx = tour_order[i + 1]
            if from_idx < len(sampling_points) and to_idx < len(sampling_points):
                point_from = sampling_points[from_idx]
                point_to = sampling_points[to_idx]
                segment_length = np.sqrt(
                    (point_to[0] - point_from[0])**2 +
                    (point_to[1] - point_from[1])**2
                )
                path_length += segment_length

        # 2. 获取第 1 块水域多边形以计算面积（当前统计接口仍按第 1 块）
        water_poly = None
        if tsp_tour_result and len(tsp_tour_result.get('tour_order', [])) >= 2:
            tsp_order = tsp_tour_result['tour_order']
            merged_points = tsp_tour_result['merged_points']
            if tsp_order[0] < len(merged_points):
                point_info = merged_points[tsp_order[0]]
                water_idx = point_info.get('water_idx')
                if point_info.get('type') == 'no_hole':
                    for water in water_no_hole:
                        if water.get('idx') == water_idx:
                            water_poly = Polygon(water['outer'], holes=water.get('holes', []))
                            break
                elif point_info.get('type') == 'with_hole':
                    for water in water_with_hole:
                        if water.get('idx') == water_idx:
                            water_poly = Polygon(water['outer'], holes=water.get('holes', []))
                            break

        if water_poly is None:
            return {
                'path_length': path_length,
                'water_area': None,
                'coverage_area': None,
                'coverage_ratio': None,
                'turn_count': None
            }

        # 3. 水域面积与覆盖率
        water_area = water_poly.area
        coverage_area = path_length * coverage_width
        coverage_ratio = coverage_area / water_area if water_area > 0 else 0.0

        # 4. 转弯次数
        turn_count = _calculate_turn_count(
            sampling_points,
            tour_order,
            angle_threshold=30.0
        )
        
        statistics = {
            'path_length': path_length,
            'water_area': water_area,
            'coverage_area': coverage_area,
            'coverage_ratio': coverage_ratio,
            'coverage_width': coverage_width,
            'turn_count': turn_count
        }
        
        return statistics
        
    except Exception as e:
        print(f"  ⚠ 计算统计信息时出错: {e}")
        import traceback
        traceback.print_exc()
        return None


def _calculate_turn_count(
    sampling_points: List[List[float]],
    tour_order: List[int],
    angle_threshold: float = 30.0
) -> int:
    """
    计算路径转弯次数
    
    参考 ASwiftPath 3D-coin/evaluate.py 中的 calculate_turn_count 方法
    
    计算逻辑：
    1. 对于路径中的每两个相邻段，计算方向向量
    2. 归一化方向向量
    3. 计算两个相邻方向向量的夹角
    4. 如果夹角大于阈值（默认30度），则认为是转弯
    
    :param sampling_points: 采样点列表 [[x, y], ...]
    :param tour_order: TSP访问顺序列表（点索引，Python 0-based）
    :param angle_threshold: 转弯角度阈值（度），小于此角度不算转弯，默认30.0
    :return: 转弯次数
    """
    if len(tour_order) < 3:
        return 0
    
    turn_count = 0
    angle_threshold_rad = np.radians(angle_threshold)
    
    # 存储路径的方向向量
    directions = []
    
    # 计算每段路径的方向向量
    for i in range(len(tour_order) - 1):
        from_idx = tour_order[i]
        to_idx = tour_order[i + 1]
        
        if from_idx < len(sampling_points) and to_idx < len(sampling_points):
            p1 = sampling_points[from_idx]
            p2 = sampling_points[to_idx]
            
            # 计算方向向量
            direction = np.array([p2[0] - p1[0], p2[1] - p1[1]])
            direction_length = np.linalg.norm(direction)
            
            if direction_length > 0:
                # 归一化方向向量
                normalized_direction = direction / direction_length
                directions.append(normalized_direction)
    
    # 计算相邻方向向量之间的角度变化
    for i in range(len(directions) - 1):
        dir1 = directions[i]
        dir2 = directions[i + 1]
        
        # 计算两个方向向量的夹角
        cos_angle = np.dot(dir1, dir2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)  # 防止数值误差
        angle = np.arccos(cos_angle)
        
        # 如果角度大于阈值，认为是转弯
        if angle > angle_threshold_rad:
            turn_count += 1
    
    return turn_count
