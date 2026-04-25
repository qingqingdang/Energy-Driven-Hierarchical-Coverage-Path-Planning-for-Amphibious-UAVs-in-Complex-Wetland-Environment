"""
水域块覆盖路径采样点生成模块（通用）

本模块用于生成单块水域的覆盖路径采样点：
1. 生成采样点（基于基向量方法，Y轴方向平行于起点→退出点方向）
2. 生成覆盖路径（待实现）
"""

from typing import List, Optional, Dict, Tuple
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.prepared import prep
from shapely.ops import unary_union


def generate_water_sampling_points(
    grid_data: Dict,
    water_info: Dict,
    water_polygon: Polygon,
    start_point: List[float],
    exit_point: List[float],
    fill_spacing: float = 4.0,
    x_scale_factor: float = 1.0,
    y_scale_factor: float = 1.0,
    degenerate_threshold: float = 1.0,
    boundary_polygon: Optional[Polygon] = None,
) -> List[List[float]]:
    """
    使用基向量方法生成单块水域的采样点
    
    采样点规则：
    - 若起点→退出点距离 >= degenerate_threshold：Y轴=起点→退出点方向，X轴=垂直方向（逆时针90度）
    - 若起点→退出点距离 < degenerate_threshold（视为同一点）：X/Y轴与可视化地图一致（地图X=(1,0)、Y=(0,1)），原点=起点/终点
    - 只保留在水域块内的采样点（排除孔洞）
    - 若提供 boundary_polygon：剔除落在禁飞区（边界外）的采样点
    
    :param degenerate_threshold: 起点与退出点距离小于此值时视为「同一点」，使用地图轴生成网格；建议取水域块50的起退点距离
    :param boundary_polygon: 地图边界多边形（禁飞区=边界外）；None 则不按边界过滤
    :return: 采样点列表 [[x, y], ...]，前两项为起点、退出点
    """
    if start_point is None or exit_point is None:
        print("  ✗ 起点或退出点未提供，无法生成采样点")
        return []
    
    # 1. 计算起点到退出点的方向与长度
    direction_vector = np.array([
        exit_point[0] - start_point[0],
        exit_point[1] - start_point[1]
    ])
    length = np.linalg.norm(direction_vector)
    
    # 以起点作为基向量坐标系原点（与可视化地图一致时原点即该点）
    origin = np.array([start_point[0], start_point[1]], dtype=float)
    
    # 2. 选择基向量：距离过近或为零时用地图轴，否则用起点→退出点方向
    # 当 length=0（起点=退出点）时若用 length < threshold，threshold 也为 0 会走 else 导致除零，故用 <= 且单独判断数值零
    use_map_axes = (length < 1e-10) or (length <= degenerate_threshold)
    if use_map_axes:
        # 与 coverage path generation map 的坐标轴一致：X=(1,0)，Y=(0,1)，原点=起点/终点
        v1 = np.array([fill_spacing * x_scale_factor, 0.0])
        v2 = np.array([0.0, fill_spacing * y_scale_factor])
    else:
        direction_normalized = direction_vector / length
        v2 = direction_normalized * fill_spacing * y_scale_factor
        v1 = np.array([-direction_normalized[1], direction_normalized[0]]) * fill_spacing * x_scale_factor
    
    # 3. 构建基向量矩阵
    basis_matrix = np.column_stack([v1, v2])
    det = np.linalg.det(basis_matrix)
    if abs(det) < 1e-10:
        print("  ✗ 基向量线性相关，无法生成采样点")
        return []
    inv_basis_matrix = np.linalg.inv(basis_matrix)
    
    # 4. 计算水域边界框
    bounds = water_polygon.bounds
    min_x, min_y, max_x, max_y = bounds
    extent_x = max_x - min_x
    extent_y = max_y - min_y

    # 5. 在基向量空间中确定网格范围（origin 已在上面设为起点）
    # 计算边界框的四个角点
    corners = np.array([
        [min_x, min_y], [max_x, min_y], 
        [max_x, max_y], [min_x, max_y]
    ])
    
    # 转换到基向量空间
    corners_rel = corners - origin  # 平移到以起点为原点
    corners_basis = (inv_basis_matrix @ corners_rel.T).T
    
    # 确定整数网格范围（添加边距）
    margin = 2
    min_n1 = int(np.floor(np.min(corners_basis[:, 0]))) - margin
    max_n1 = int(np.ceil(np.max(corners_basis[:, 0]))) + margin
    min_n2 = int(np.floor(np.min(corners_basis[:, 1]))) - margin
    max_n2 = int(np.ceil(np.max(corners_basis[:, 1]))) + margin
    
    # 6. 准备多边形（用于快速contains检查）
    prepared_polygon = prep(water_polygon)
    
    # 7. 生成网格点
    sampling_points = []
    total_generated = 0
    points_in_water = 0
    points_too_close = 0
    min_boundary_distance = 0.5  # 最小边界距离（米）
    
    for n1 in range(min_n1, max_n1 + 1):
        for n2 in range(min_n2, max_n2 + 1):
            # 在基向量空间中生成点
            point_basis = np.array([n1, n2])
            
            # 转换回实际坐标（从基向量坐标系 -> 世界坐标系）
            point_real = origin + (basis_matrix @ point_basis)
            
            # 创建点对象
            point = Point(float(point_real[0]), float(point_real[1]))
            
            total_generated += 1
            
            # 检查点是否在水域块内（排除孔洞）
            if prepared_polygon.contains(point):
                # 检查点到边界的距离
                distance_to_boundary = water_polygon.boundary.distance(point)
                
                if distance_to_boundary >= min_boundary_distance:
                    # 距离边界足够远，保留该点
                    sampling_points.append([float(point_real[0]), float(point_real[1])])
                    points_in_water += 1
                else:
                    # 距离边界太近，过滤掉
                    points_too_close += 1
    
    # 剔除禁飞区内的采样点（边界多边形外 = 红色区域）
    no_fly_filtered = 0
    if boundary_polygon is not None and sampling_points:
        try:
            prepared_boundary = prep(boundary_polygon)
            in_boundary = []
            for p in sampling_points:
                pt = Point(p[0], p[1])
                if prepared_boundary.contains(pt) or pt.within(boundary_polygon):
                    in_boundary.append(p)
                else:
                    no_fly_filtered += 1
            sampling_points = in_boundary
        except Exception:
            pass
    
    # 将起点和退出点放在最前面
    # 采样点数组 = [起点(索引0), 退出点(索引1), 网格点1, 网格点2, ...]
    complete_sampling_points = [start_point, exit_point] + sampling_points

    # 进度信息：避免“看起来卡住”，便于估算成本矩阵规模；并提示单位以便核对
    try:
        msg = (
            "  ✓ 水域块采样点生成完成："
            f"总生成候选={total_generated}, 水域内保留={points_in_water},"
            f" 边界过滤={points_too_close}"
        )
        if no_fly_filtered > 0:
            msg += f", 禁飞区剔除={no_fly_filtered}"
        msg += f", 最终点数={len(complete_sampling_points)}"
        print(msg)
        print(
            f"  [单位] 采样间距 fill_spacing={fill_spacing}（与地图坐标同单位，当前地图为米）；"
            f"水域边界框约 {extent_x:.1f}×{extent_y:.1f}；"
            f"若点数异常多请检查坐标单位或增大 fill_spacing（如 10、20）"
        )
    except Exception:
        # 打印失败不影响主流程
        pass
    
    return complete_sampling_points


def generate_water_coverage_path(
    grid_data: Dict,
    water_info: Dict,
    start_point: List[float],
    exit_point: List[float],
    grid_size: float = 4.0
) -> Optional[Dict]:
    """
    生成单块水域的覆盖路径（待实现）
    
    :param grid_data: 栅格数据
    :param water_info: 水域信息字典
    :param start_point: 起点坐标 [x, y]
    :param exit_point: 退出点坐标 [x, y]
    :param grid_size: 栅格大小（米）
    :return: 覆盖路径结果字典
    """
    # TODO: 实现覆盖路径生成逻辑
    pass
