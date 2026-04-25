##V0.2.2
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree
import latlongcartconv as lc  
import numpy as np
import time
import os
import subprocess

# ───────────────────────────────────────────────
def ll_ring_to_cart(ring_ll, conv):
    return [conv.get_cartesian([lon, lat]) for lat, lon in ring_ll]

def analyze_water_hierarchy(water_terrain, conv, debug=False):
    """
    分析水域层级关系
    :param water_terrain: 水域地形数据
    :param conv: 坐标转换器
    :param debug: 是否打印调试信息
    :return: 层级关系字典
    """
    # 构建所有水域多边形的列表
    all_water_polygons = []
    for idx, poly_data in enumerate(water_terrain):
        # 转换外轮廓为笛卡尔坐标
        outer_cart = ll_ring_to_cart(poly_data[0], conv)
        outer_polygon = Polygon(outer_cart)
        
        all_water_polygons.append({
            'idx': idx,
            'polygon': outer_polygon,
            'data': poly_data
        })
    
    # 创建空间索引以加速检测
    polygon_list = [item['polygon'] for item in all_water_polygons]
    spatial_index = STRtree(polygon_list)
    
    # 分析每个水域的孔洞
    hierarchy_results = {}
    
    for water_idx, water_data in enumerate(water_terrain):
        # 转换外轮廓
        outer_cart = ll_ring_to_cart(water_data[0], conv)
        outer_polygon = Polygon(outer_cart)
        
        # 获取孔洞
        holes = water_data[1:] if len(water_data) > 1 else []
        
        hole_analysis = []
        
        for hole_idx, hole_coords in enumerate(holes):
            # 转换孔洞为笛卡尔坐标
            hole_cart = ll_ring_to_cart(hole_coords, conv)
            hole_polygon = Polygon(hole_cart)
            
            # 检测这个孔洞内是否包含其他水域
            contained_waters = []
            candidates = spatial_index.query(hole_polygon)
            
            # 改进的检测逻辑：直接使用索引匹配
            for i, water_info in enumerate(all_water_polygons):
                if water_info['idx'] != water_idx:  # 排除自己
                    # 检查水域是否在孔洞内部
                    water_poly = water_info['polygon']
                    
                    # 多种检测方法
                    contains_check = hole_polygon.contains(water_poly)
                    within_check = water_poly.within(hole_polygon)
                    centroid_check = hole_polygon.contains(water_poly.centroid)
                    
                    # 如果任一检测通过，认为水域在孔洞内
                    if contains_check or within_check or centroid_check:
                        contained_waters.append(water_info['idx'])
            
            # 去重并排序
            contained_waters = sorted(list(set(contained_waters)))
            
            # 分类孔洞类型
            if contained_waters:
                hole_type = "container"  # 包含水域的孔洞
            else:
                hole_type = "real_hole"  # 真正的孔洞
            
            hole_analysis.append({
                'hole_idx': hole_idx,
                'hole_coords': hole_cart,  # 笛卡尔坐标
                'type': hole_type,
                'contained_waters': contained_waters
            })
            
        
        hierarchy_results[water_idx] = {
            'outer_coords': outer_cart,  # 笛卡尔坐标
            'holes': hole_analysis
        }
    
    # 保存层级结构到文件
    save_hierarchy_to_file(hierarchy_results)
    
    return hierarchy_results

def save_hierarchy_to_file(hierarchy_results, filename="map_structure.txt"):
    """
    将水域层级结构保存到文本文件
    :param hierarchy_results: 层级关系字典
    :param filename: 输出文件名
    """
    import os
    
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("水域层级结构分析结果\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"总水域数量: {len(hierarchy_results)}\n")
        f.write(f"生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        # 统计信息
        total_holes = 0
        real_holes_count = 0
        container_holes_count = 0
        
        for water_idx, water_info in hierarchy_results.items():
            total_holes += len(water_info['holes'])
            for hole in water_info['holes']:
                if hole['type'] == 'real_hole':
                    real_holes_count += 1
                else:
                    container_holes_count += 1
        
        f.write("-" * 80 + "\n")
        f.write("统计摘要\n")
        f.write("-" * 80 + "\n")
        f.write(f"孔洞总数: {total_holes}\n")
        f.write(f"  - 真正孔洞: {real_holes_count}\n")
        f.write(f"  - 容器孔洞: {container_holes_count}\n\n")
        
        # 详细信息
        f.write("=" * 80 + "\n")
        f.write("详细层级关系\n")
        f.write("=" * 80 + "\n\n")
        
        for water_idx in sorted(hierarchy_results.keys()):
            water_info = hierarchy_results[water_idx]
            holes = water_info['holes']
            
            if len(holes) == 0:
                # 没有孔洞的水域
                f.write(f"P{water_idx}: 无孔洞\n")
            else:
                # 有孔洞的水域
                f.write(f"P{water_idx}: 有 {len(holes)} 个孔洞\n")
                
                for hole in holes:
                    hole_idx = hole['hole_idx']
                    hole_type = hole['type']
                    contained_waters = hole['contained_waters']
                    
                    if hole_type == 'real_hole':
                        f.write(f"  ○ P{water_idx}-{hole_idx+1}: 真正孔洞（陆地）\n")
                    else:
                        f.write(f"  ✓ P{water_idx}-{hole_idx+1}: 容器孔洞\n")
                        f.write(f"      包含水域: P{contained_waters}\n")
                
                f.write("\n")
        
        f.write("=" * 80 + "\n")
        f.write("图例说明\n")
        f.write("=" * 80 + "\n")
        f.write("○ 真正孔洞: 孔洞内部是陆地\n")
        f.write("✓ 容器孔洞: 孔洞内部包含其他水域\n")
        f.write("P{数字}: 水域编号\n")
        f.write("P{水域}-{孔洞}: 孔洞编号（如 P14-2 表示 P14 的第2个孔洞）\n")
    
    return filepath

def ll_poly_to_cart_shapely(poly_ll, conv):
    outer = ll_ring_to_cart(poly_ll[0], conv)
    holes = [ll_ring_to_cart(r, conv) for r in poly_ll[1:]] if len(poly_ll) > 1 else []
    return Polygon(outer, holes)

def shapely_to_np_rings(shape):
    polys = [shape] if isinstance(shape, Polygon) else list(shape.geoms)
    rings = []
    for p in polys:
        outer = np.asarray(p.exterior.coords)[:-1]
        holes = [np.asarray(r.coords)[:-1] for r in p.interiors]
        rings.append((outer, holes))
    return rings

def sample_line(line: LineString, step: float):
    samples = []
    d = 0.0
    while d < line.length:
        samples.append(np.array(line.interpolate(d).coords[0]))
        d += step
    return np.array(samples)
# ───────────────────────────────────────────────

def extract_env_cartesian(env_obj, shrink_dist=0.5, sample_dist=10):
    lat0, lon0 = env_obj.starting_position[0]
    conv = lc.LLCCV([lon0, lat0])

    water_with_hole = []
    water_no_hole = []
    sampling_points = []

    # 将边界经纬度转换为笛卡尔坐标
    boundary_polygon = np.array([conv.get_cartesian([lon, lat]) for lat, lon in env_obj.boundary_points])

    for idx, poly_ll in enumerate(env_obj.water_terrain):
        poly_cart = ll_poly_to_cart_shapely(poly_ll, conv)
        outer, holes = shapely_to_np_rings(poly_cart)[0]

        shrunk = poly_cart.buffer(-shrink_dist, join_style=2)
        if shrunk.is_empty:
            print(f"Warning: Water polygon {idx} disappears after shrinking {shrink_dist} meters. Skipped.")
            continue
        shrunk = unary_union(shrunk)
        in_outer, in_holes = shapely_to_np_rings(shrunk)[0]

        outer_line = LineString(in_outer)
        hole_lines = [LineString(h) for h in in_holes]
        outer_samples = sample_line(outer_line, sample_dist)
        hole_samples = [sample_line(h, sample_dist) for h in hole_lines]

        # 添加所有采样点到统一列表
        sampling_points.extend(outer_samples.tolist())
        for hs in hole_samples:
            sampling_points.extend(hs.tolist())

        record = {
            "idx": idx,
            "outer": outer,
            "holes": holes,
            "outer_samples": outer_samples,
            "hole_samples": hole_samples
        }

        if len(holes) > 0:
            water_with_hole.append(record)
        else:
            water_no_hole.append(record)

    # 分析水域层级关系
    water_hierarchy = analyze_water_hierarchy(env_obj.water_terrain, conv)
    
    result = {
        "water_with_hole": water_with_hole,
        "water_no_hole": water_no_hole,
        "sampling_points": sampling_points,
        "boundary_polygon": boundary_polygon,
        "water_hierarchy": water_hierarchy  # 新增：层级关系数据
    }
    return result


# #V0.2.1
# from shapely.geometry import Polygon, LinearRing
# from shapely.validation import explain_validity
# import latlongcartconv as lc  
# import numpy as np

# def shrink_polygon(poly: Polygon, offset: float) -> Polygon:
#     """向内缩放多边形 offset 米，空或非法返回原对象"""
#     return poly.buffer(-offset) if poly.is_valid and poly.area else poly


# def perimeter_points(poly: Polygon, spacing: float) -> list:
#     """沿外边每隔 spacing 米采样点（笛卡尔坐标系）"""
#     if not (poly.is_valid and poly.exterior and poly.exterior.length):
#         return []
#     n = max(int(poly.exterior.length // spacing), 1)
#     dist = np.linspace(0, poly.exterior.length, n, endpoint=False)
#     return [poly.exterior.interpolate(d).coords[0] for d in dist]


# def extract_env_cartesian(env_obj, shrink_offset=5.0, point_spacing=10.0):
#     """
#     返回:
#         boundary_polygon   : shapely.Polygon（笛卡尔）
#         water_polygons     : [shapely.Polygon, ...]（笛卡尔，含孔洞）
#         sampling_points    : [[x, y], ...] （笛卡尔坐标）
#     """
#     b_ll = [[float(lat), float(lon)] for lat, lon in env_obj.boundary_points]
#     holes_ll = env_obj.geo_fencing_holes or []
#     terrain_src = env_obj.water_terrain or []

#     rings_ll = []
#     grouped = []

#     for item in terrain_src:
#         if isinstance(item[0], (int, float)):
#             rings_ll.append([[float(lat), float(lon)] for lat, lon in item])
#         else:
#             grouped.append(
#                 [[ [float(lat), float(lon)] for lat, lon in ring] for ring in item]
#             )

#     all_pts = b_ll + [pt for ring in rings_ll for pt in ring]
#     origin = [min(p[1] for p in all_pts), min(p[0] for p in all_pts)]
#     cvt = lc.LLCCV(origin)

#     ll2xy = lambda lat, lon: cvt.get_cartesian([lon, lat])

#     water_polygons = []

#     for rings in grouped:
#         outer_xy = [ll2xy(*pt) for pt in rings[0]]
#         holes_xy = [[ll2xy(*pt) for pt in hole] for hole in rings[1:]]
#         poly = Polygon(outer_xy, holes=holes_xy)
#         if not poly.is_valid:
#             print("Invalid polygon (grouped) detected. Reason:", explain_validity(poly))
#             poly = poly.buffer(0)
#         water_polygons.append(poly)

#     rings_xy = [(LinearRing([ll2xy(*pt) for pt in ring]), ring) for ring in rings_ll]
#     rings_xy.sort(key=lambda t: abs(Polygon(t[0]).area), reverse=True)
#     used = set()

#     for i, (outer_ring, outer_ll) in enumerate(rings_xy):
#         if i in used:
#             continue
#         holes_xy = []
#         for j, (inner_ring, inner_ll) in enumerate(rings_xy):
#             if j == i or j in used:
#                 continue
#             if Polygon(inner_ring).within(Polygon(outer_ring)):
#                 holes_xy.append(list(inner_ring.coords))
#                 used.add(j)
#         poly = Polygon(list(outer_ring.coords), holes=holes_xy)
#         if not poly.is_valid:
#             print("Invalid polygon (rings) detected. Reason:", explain_validity(poly))
#             poly = poly.buffer(0)
#         water_polygons.append(poly)
#         used.add(i)

#     boundary_xy = [ll2xy(*pt) for pt in b_ll]
#     holes_xy_full = [[ll2xy(*pt) for pt in hole] for hole in holes_ll]
#     boundary_poly = Polygon(boundary_xy, holes=holes_xy_full if holes_ll else None)
#     if not boundary_poly.is_valid:
#         print("Invalid boundary polygon detected. Reason:", explain_validity(boundary_poly))
#         boundary_poly = boundary_poly.buffer(0)

#     sampling_points = []
#     for poly in water_polygons:
#         inner = shrink_polygon(poly, shrink_offset)
#         for xy in perimeter_points(inner, point_spacing):
#             sampling_points.append(xy)  # 返回笛卡尔坐标

#     return {
#         "boundary_polygon": boundary_poly,
#         "water_polygons": water_polygons,
#         "sampling_points": sampling_points
#     }

# # V0.1  无法解决凹图水域
# from shapely.geometry import Polygon, Point, LineString
# from shapely import affinity
# import latlongcartconv as lc
# import numpy as np


# def shrink_polygon(polygon: Polygon, offset: float) -> Polygon:
#     """
#     向内缩放一个多边形 offset 米
#     """
#     if not polygon.is_valid or polygon.area == 0:
#         return polygon
#     return polygon.buffer(-offset)
 

# def generate_perimeter_points(polygon: Polygon, spacing: float = 10.0):
#     """
#     沿着多边形边缘每隔 spacing 米生成一个点
#     """
#     if not polygon.is_valid:
#         return []

#     exterior = polygon.exterior
#     length = exterior.length
#     num_points = int(length // spacing)
#     distances = np.linspace(0, length, num_points, endpoint=False)
#     points = [exterior.interpolate(d).coords[0] for d in distances]
#     return points


# def extract_env_cartesian(env_obj, shrink_offset=5.0, point_spacing=10.0):
#     """
#     提取环境中的边界、水域及其岸边采样点（基于内缩多边形 + spacing）
#     :param env_obj: 环境对象（如 Benning_EXP）
#     :param shrink_offset: 缩小内多边形时的距离（米）
#     :param point_spacing: 沿边采样点间距（米）
#     :return: dict 包含 boundary_polygon, water_polygons, sampling_points
#     """
#     boundary = env_obj.boundary_points
#     water_areas = env_obj.water_terrain or []
#     holes = env_obj.geo_fencing_holes if env_obj.geo_fencing_holes else []

#     all_points = boundary + [p for poly in water_areas for p in poly]
#     origin = [min([p[1] for p in all_points]), min([p[0] for p in all_points])]
#     converter = lc.LLCCV(origin)

#     boundary_xy = [converter.get_cartesian([lng, lat]) for lat, lng in boundary]
#     hole_xy = [[converter.get_cartesian([lng, lat]) for lat, lng in hole] for hole in holes]
#     water_xy = [[converter.get_cartesian([lng, lat]) for lat, lng in water] for water in water_areas]

#     boundary_poly = Polygon(boundary_xy, holes=hole_xy if hole_xy else None)
#     water_polygons = [Polygon(poly) for poly in water_xy]

#     sampling_points = []
#     for poly in water_polygons:
#         inner_poly = shrink_polygon(poly, offset=shrink_offset)
#         if inner_poly.is_empty:
#             continue
#         pts = generate_perimeter_points(inner_poly, spacing=point_spacing)
#         sampling_points.extend(pts)

#     return {
#         "boundary_polygon": boundary_poly,
#         "water_polygons": water_polygons,
#         "sampling_points": sampling_points
#     }

def generate_map_structure_file(env_obj=None, output_filename="map_structure.txt"):
    """
    独立函数：生成地图结构文件
    可以直接调用此函数来生成水域层级结构文件
    
    :param env_obj: 环境对象，如果为None则自动加载OSM_ENV
    :param output_filename: 输出文件名
    :return: 保存的文件路径
    """
    print("=" * 80)
    print("开始生成水域层级结构文件")
    print("=" * 80)
    
    # 如果没有提供环境对象，自动加载
    if env_obj is None:
        print("\n加载环境对象...")
        from OSM_ENVIRONMENTS import OSM_ENV
        env_obj = OSM_ENV(UAV=1)
    
    # 创建坐标转换器
    lat0, lon0 = env_obj.starting_position[0]
    conv = lc.LLCCV([lon0, lat0])
    
    # 分析水域层级关系
    print("\n分析水域层级关系...")
    water_hierarchy = analyze_water_hierarchy(env_obj.water_terrain, conv, debug=False)
    
    # 保存到文件
    print("\n保存结果到文件...")
    filepath = save_hierarchy_to_file(water_hierarchy, filename=output_filename)
    
    print("\n" + "=" * 80)
    print(f"✓ 完成！文件已保存到: {filepath}")
    print("=" * 80)
    
    return filepath

if __name__ == "__main__":
    # 方式1: 直接生成地图结构文件
    generate_map_structure_file()
    
    # 方式2: 使用自定义环境对象
    # from OSM_ENVIRONMENTS import OSM_ENV
    # env = OSM_ENV(UAV=1)
    # generate_map_structure_file(env, "my_custom_map.txt")
    
    # 方式3: 完整的环境提取
    # from Distance_calculate import write_tsp_and_par
    # env_obj = ... # 你的环境对象
    # result = extract_env_cartesian(env_obj)
    # write_tsp_and_par(result["sampling_points"], save_dir=r".\data")
