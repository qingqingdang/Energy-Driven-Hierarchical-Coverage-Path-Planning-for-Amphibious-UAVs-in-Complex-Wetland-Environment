##V0.1 - 水域巡检采样点生成器
import numpy as np
from shapely.geometry import Polygon, LineString, Point as ShapelyPoint
from shapely.ops import unary_union

def generate_inspection_points(env_data, offset_distance=5.0, sample_spacing=10.0, hole_strategy='outer_only'):
    """
    生成水域巡检采样点
    
    :param env_data: 来自 extract_env_cartesian 的环境数据
    :param offset_distance: 内缩距离（米），生成的采样点距离水域边界的距离
    :param sample_spacing: 采样点间距（米）
    :param hole_strategy: 有孔洞水域的处理策略
        - 'outer_only': 只在外轮廓上采样（忽略孔洞）
        - 'outer_and_holes': 在外轮廓和所有孔洞边界上都采样
        - 'avoid_holes': 在内缩后避开孔洞的区域采样
    :return: 包含采样点信息的字典
    """
    water_with_hole = env_data["water_with_hole"]
    water_no_hole = env_data["water_no_hole"]
    
    all_inspection_points = []
    inspection_records = []
    
    print(f"\n生成巡检采样点...")
    print(f"  内缩距离: {offset_distance} 米")
    print(f"  采样间距: {sample_spacing} 米")
    print(f"  孔洞处理策略: {hole_strategy}")
    
    # 处理无孔洞水域（简单情况）
    print(f"\n处理 {len(water_no_hole)} 个无孔洞水域...")
    for water in water_no_hole:
        points, record = _generate_points_no_hole(
            water, offset_distance, sample_spacing
        )
        all_inspection_points.extend(points)
        inspection_records.append(record)
    
    # 处理有孔洞水域（根据策略）
    print(f"\n处理 {len(water_with_hole)} 个有孔洞水域（策略: {hole_strategy}）...")
    for water in water_with_hole:
        if hole_strategy == 'outer_only':
            points, record = _generate_points_outer_only(
                water, offset_distance, sample_spacing
            )
        elif hole_strategy == 'outer_and_holes':
            points, record = _generate_points_outer_and_holes(
                water, offset_distance, sample_spacing
            )
        elif hole_strategy == 'avoid_holes':
            points, record = _generate_points_avoid_holes(
                water, offset_distance, sample_spacing
            )
        else:
            raise ValueError(f"Unknown hole_strategy: {hole_strategy}")
        
        all_inspection_points.extend(points)
        inspection_records.append(record)
    
    print(f"\n✓ 生成完成！总采样点数: {len(all_inspection_points)}")
    
    return {
        'inspection_points': all_inspection_points,      # 所有采样点列表 [[x, y], ...]
        'inspection_records': inspection_records,        # 每个水域的详细记录
        'total_points': len(all_inspection_points),
        'offset_distance': offset_distance,
        'sample_spacing': sample_spacing,
        'hole_strategy': hole_strategy
    }


def _generate_points_no_hole(water, offset_distance, sample_spacing):
    """
    为无孔洞水域生成采样点
    
    处理流程：
    1. 获取原始水域多边形
    2. 内缩 offset_distance 米
    3. 沿边界均匀采样
    """
    water_idx = water["idx"]
    outer_coords = water["outer"]
    
    # 创建原始多边形
    original_poly = Polygon(outer_coords)
    
    # 内缩
    offset_poly = original_poly.buffer(-offset_distance, join_style=2)
    
    # 检查内缩后是否消失
    if offset_poly.is_empty or offset_poly.area < 1.0:
        print(f"  警告: 水域 P{water_idx} 在内缩 {offset_distance}m 后消失，跳过")
        return [], {
            'water_idx': water_idx,
            'has_holes': False,
            'points': [],
            'point_count': 0,
            'status': 'empty_after_offset'
        }
    
    # 处理可能的 MultiPolygon 情况
    if offset_poly.geom_type == 'MultiPolygon':
        # 取最大的多边形
        offset_poly = max(offset_poly.geoms, key=lambda p: p.area)
    
    # 沿边界采样
    sampling_points = _sample_polygon_boundary(offset_poly, sample_spacing)
    
    print(f"  P{water_idx} (无孔洞): 生成 {len(sampling_points)} 个采样点")
    
    return sampling_points, {
        'water_idx': water_idx,
        'has_holes': False,
        'points': sampling_points,
        'point_count': len(sampling_points),
        'offset_area': offset_poly.area,
        'status': 'success'
    }


def _generate_points_outer_only(water, offset_distance, sample_spacing):
    """
    策略1: 只在外轮廓上采样（忽略孔洞）
    
    适用场景：
    - 孔洞是陆地，不需要巡检
    - 简化处理，只关注水域外边界
    
    优点：简单直接
    缺点：可能遗漏孔洞边界区域
    """
    water_idx = water["idx"]
    outer_coords = water["outer"]
    hole_coords = water["holes"]
    
    # 只处理外轮廓（忽略孔洞）
    original_poly = Polygon(outer_coords)
    offset_poly = original_poly.buffer(-offset_distance, join_style=2)
    
    if offset_poly.is_empty or offset_poly.area < 1.0:
        print(f"  警告: 水域 P{water_idx} 外轮廓在内缩后消失，跳过")
        return [], {
            'water_idx': water_idx,
            'has_holes': True,
            'hole_count': len(hole_coords),
            'points': [],
            'point_count': 0,
            'status': 'empty_after_offset'
        }
    
    if offset_poly.geom_type == 'MultiPolygon':
        offset_poly = max(offset_poly.geoms, key=lambda p: p.area)
    
    sampling_points = _sample_polygon_boundary(offset_poly, sample_spacing)
    
    print(f"  P{water_idx} (有{len(hole_coords)}个孔洞, 策略:仅外轮廓): 生成 {len(sampling_points)} 个采样点")
    
    return sampling_points, {
        'water_idx': water_idx,
        'has_holes': True,
        'hole_count': len(hole_coords),
        'strategy': 'outer_only',
        'points': sampling_points,
        'point_count': len(sampling_points),
        'status': 'success'
    }


def _generate_points_outer_and_holes(water, offset_distance, sample_spacing):
    """
    策略2: 在外轮廓和所有孔洞边界上都采样
    
    适用场景：
    - 孔洞是陆地，需要巡检孔洞边界
    - 需要完整覆盖所有边界
    
    优点：覆盖完整
    缺点：孔洞周围采样点密集
    """
    water_idx = water["idx"]
    outer_coords = water["outer"]
    hole_coords_list = water["holes"]
    
    all_points = []
    
    # 1. 处理外轮廓（向内缩）
    outer_poly = Polygon(outer_coords)
    offset_outer = outer_poly.buffer(-offset_distance, join_style=2)
    
    if not offset_outer.is_empty and offset_outer.area >= 1.0:
        if offset_outer.geom_type == 'MultiPolygon':
            offset_outer = max(offset_outer.geoms, key=lambda p: p.area)
        outer_points = _sample_polygon_boundary(offset_outer, sample_spacing)
        all_points.extend(outer_points)
    
    # 2. 处理每个孔洞（向外扩）
    hole_points_count = 0
    for hole_coords in hole_coords_list:
        hole_poly = Polygon(hole_coords)
        # 孔洞向外扩，使采样点离开孔洞边界
        offset_hole = hole_poly.buffer(offset_distance, join_style=2)
        
        if not offset_hole.is_empty:
            if offset_hole.geom_type == 'MultiPolygon':
                offset_hole = max(offset_hole.geoms, key=lambda p: p.area)
            hole_points = _sample_polygon_boundary(offset_hole, sample_spacing)
            all_points.extend(hole_points)
            hole_points_count += len(hole_points)
    
    print(f"  P{water_idx} (有{len(hole_coords_list)}个孔洞, 策略:外轮廓+孔洞): "
          f"外轮廓 {len(all_points)-hole_points_count} + 孔洞 {hole_points_count} = {len(all_points)} 个采样点")
    
    return all_points, {
        'water_idx': water_idx,
        'has_holes': True,
        'hole_count': len(hole_coords_list),
        'strategy': 'outer_and_holes',
        'points': all_points,
        'point_count': len(all_points),
        'outer_points': len(all_points) - hole_points_count,
        'hole_points': hole_points_count,
        'status': 'success'
    }


def _generate_points_avoid_holes(water, offset_distance, sample_spacing):
    """
    策略3: 在考虑孔洞的情况下内缩并采样
    
    适用场景：
    - 孔洞是陆地，需要避开
    - 希望采样点只在水域内
    
    优点：采样点保证在水域内且远离边界
    缺点：可能产生不连续的区域
    """
    water_idx = water["idx"]
    outer_coords = water["outer"]
    hole_coords_list = water["holes"]
    
    # 创建带孔洞的多边形
    original_poly = Polygon(outer_coords, holes=hole_coords_list)
    
    # 内缩（会自动处理孔洞）
    offset_poly = original_poly.buffer(-offset_distance, join_style=2)
    
    if offset_poly.is_empty or offset_poly.area < 1.0:
        print(f"  警告: 水域 P{water_idx} 在考虑孔洞并内缩后消失，跳过")
        return [], {
            'water_idx': water_idx,
            'has_holes': True,
            'hole_count': len(hole_coords_list),
            'points': [],
            'point_count': 0,
            'status': 'empty_after_offset'
        }
    
    all_points = []
    
    # 处理可能的 MultiPolygon（内缩后可能分裂）
    if offset_poly.geom_type == 'MultiPolygon':
        print(f"  注意: 水域 P{water_idx} 内缩后分裂为 {len(offset_poly.geoms)} 个部分")
        for sub_poly in offset_poly.geoms:
            if sub_poly.area >= 1.0:
                points = _sample_polygon_boundary(sub_poly, sample_spacing)
                all_points.extend(points)
    else:
        all_points = _sample_polygon_boundary(offset_poly, sample_spacing)
    
    print(f"  P{water_idx} (有{len(hole_coords_list)}个孔洞, 策略:避开孔洞): 生成 {len(all_points)} 个采样点")
    
    return all_points, {
        'water_idx': water_idx,
        'has_holes': True,
        'hole_count': len(hole_coords_list),
        'strategy': 'avoid_holes',
        'points': all_points,
        'point_count': len(all_points),
        'status': 'success'
    }


def _sample_polygon_boundary(polygon, spacing):
    """
    沿多边形边界均匀采样点
    
    :param polygon: Shapely Polygon 对象
    :param spacing: 采样点间距（米）
    :return: 采样点列表 [[x, y], ...]
    """
    if polygon.is_empty:
        return []
    
    # 获取外边界
    boundary = polygon.exterior
    
    # 沿边界采样
    length = boundary.length
    num_points = max(int(length / spacing), 3)  # 至少3个点
    
    samples = []
    for i in range(num_points):
        distance = (i / num_points) * length
        point = boundary.interpolate(distance)
        samples.append([point.x, point.y])
    
    return samples


def visualize_inspection_points(env_data, inspection_data, save_path='inspection_points_visualization.png'):
    """
    可视化巡检采样点
    
    :param env_data: 环境数据
    :param inspection_data: 巡检采样点数据
    :param save_path: 保存路径
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPLPolygon
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    water_with_hole = env_data["water_with_hole"]
    water_no_hole = env_data["water_no_hole"]
    boundary_polygon = env_data["boundary_polygon"]
    inspection_points = inspection_data['inspection_points']
    
    # 绘制水域
    for water in water_with_hole + water_no_hole:
        # 外轮廓
        ax.fill(*water["outer"].T, facecolor='#87CEEB', edgecolor='darkblue', 
               alpha=0.3, linewidth=1.5)
        
        # 孔洞
        for hole in water["holes"]:
            ax.fill(*hole.T, facecolor='white', edgecolor='darkblue', 
                   alpha=1.0, linewidth=1.5)
        
        # 水域编号
        center = water["outer"].mean(axis=0)
        ax.text(center[0], center[1], f"P{water['idx']}", 
               ha='center', va='center', fontsize=9, fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                        edgecolor='darkblue', alpha=0.8))
    
    # 绘制边界
    closed_boundary = np.vstack([boundary_polygon, boundary_polygon[0:1, :]])
    ax.plot(closed_boundary[:, 0], closed_boundary[:, 1], 
           'k-', linewidth=2, label='Boundary')
    
    # 绘制巡检采样点
    if inspection_points:
        points_array = np.array(inspection_points)
        ax.scatter(points_array[:, 0], points_array[:, 1], 
                  c='red', s=20, marker='o', alpha=0.8, 
                  edgecolors='darkred', linewidths=0.5,
                  label=f'Inspection Points ({len(inspection_points)})')
    
    # 设置
    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"Inspection Sampling Points\n"
                f"Offset: {inspection_data['offset_distance']}m, "
                f"Spacing: {inspection_data['sample_spacing']}m, "
                f"Strategy: {inspection_data['hole_strategy']}", 
                fontsize=12, fontweight='bold')
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n可视化图片已保存: {save_path}")
    plt.show()


def assign_inspection_clusters(inspection_data, core_points_data, env_data):
    """
    将巡检点绑定到 K-Means 聚类块。
    :param inspection_data: 巡检点生成结果
    :param core_points_data: compute_core_points 返回的数据
    :param env_data: 环境数据
    :return: inspection_data（原地更新，增加 'cluster_labels' 列表）
    """
    inspection_points = inspection_data.get('inspection_points', [])
    if not inspection_points:
        inspection_data['cluster_labels'] = []
        return inspection_data

    clusters_map = core_points_data.get('water_with_hole_clusters') if core_points_data else None
    if not clusters_map:
        inspection_data['cluster_labels'] = [None] * len(inspection_points)
        return inspection_data

    water_lookup = {}
    for water in env_data.get("water_with_hole", []):
        idx = water.get("idx")
        if idx is None:
            continue
        try:
            water_lookup[idx] = Polygon(water["outer"], holes=water["holes"])
        except Exception:
            continue

    cluster_polygons = []
    for water_idx, clusters in clusters_map.items():
        water_polygon = water_lookup.get(water_idx)
        if water_polygon is None or water_polygon.is_empty:
            continue
        for cluster_idx, cluster_info in enumerate(clusters):
            hull_coords = cluster_info.get("hull")
            if not hull_coords or len(hull_coords) < 3:
                continue
            try:
                hull_polygon = Polygon(hull_coords)
            except Exception:
                continue
            if hull_polygon.is_empty or hull_polygon.area <= 0:
                continue
            cluster_polygons.append((water_idx, cluster_idx, hull_polygon, water_polygon))

    cluster_labels = [None] * len(inspection_points)
    if not cluster_polygons:
        inspection_data['cluster_labels'] = cluster_labels
        return inspection_data

    for idx, point in enumerate(inspection_points):
        point_geom = ShapelyPoint(point)
        for water_idx, cluster_idx, hull_poly, water_poly in cluster_polygons:
            if not hull_poly.contains(point_geom):
                continue
            if not water_poly.contains(point_geom):
                continue
            cluster_labels[idx] = {
                "water_idx": water_idx,
                "cluster_idx": cluster_idx
            }
            break

    inspection_data['cluster_labels'] = cluster_labels
    return inspection_data


# ==================== 使用示例 ====================
if __name__ == "__main__":
    from OSM_ENVIRONMENTS import OSM_ENV
    from extract_map import extract_env_cartesian
    
    print("=" * 80)
    print("巡检采样点生成器 - 测试")
    print("=" * 80)
    
    # 1. 加载环境
    env = OSM_ENV(UAV=1)
    
    # 2. 提取地图数据
    env_data = extract_env_cartesian(env, shrink_dist=0.5, sample_dist=10)
    
    # 3. 生成巡检采样点（测试不同策略）
    print("\n" + "=" * 80)
    print("测试策略1: 只在外轮廓采样")
    print("=" * 80)
    inspection_data_1 = generate_inspection_points(
        env_data, 
        offset_distance=5.0, 
        sample_spacing=10.0,
        hole_strategy='outer_only'
    )
    visualize_inspection_points(env_data, inspection_data_1, 
                               'inspection_outer_only.png')
    
    print("\n" + "=" * 80)
    print("测试策略2: 在外轮廓和孔洞都采样")
    print("=" * 80)
    inspection_data_2 = generate_inspection_points(
        env_data, 
        offset_distance=5.0, 
        sample_spacing=10.0,
        hole_strategy='outer_and_holes'
    )
    visualize_inspection_points(env_data, inspection_data_2, 
                               'inspection_outer_and_holes.png')
    
    print("\n" + "=" * 80)
    print("测试策略3: 避开孔洞采样")
    print("=" * 80)
    inspection_data_3 = generate_inspection_points(
        env_data, 
        offset_distance=5.0, 
        sample_spacing=10.0,
        hole_strategy='avoid_holes'
    )
    visualize_inspection_points(env_data, inspection_data_3, 
                               'inspection_avoid_holes.png')
    
    print("\n" + "=" * 80)
    print("所有测试完成！")
    print("=" * 80)






