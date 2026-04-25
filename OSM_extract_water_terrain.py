##V0.2.1
import osmnx as ox
from osmnx import features
import geopandas as gpd
from shapely.geometry import Polygon, MultiPolygon, LineString
from paths import BASE_DIR

# === 1. 定义检索窗口 (lat, lon) ➜ (lon, lat) ===30.2811082
# polygon_coords = [
#     [30.2811082, 120.0526917],
#     [30.2766514, 120.0526316],
#     [30.2768089, 120.0579339],
#     [30.2812915, 120.0579141]
# ]
polygon_coords = [
    [30.2812915, 120.0526917],
    [30.2766514, 120.0526917],
    [30.2768089, 120.0579339],
    [30.2812915, 120.0579339]
]
region_polygon = Polygon([[lon, lat] for lat, lon in polygon_coords])

# === 2. 定义 OSM 查询标签（扩大到池塘/蓄水池等） ===
tags = {
    "natural":      ["water", "wetland"],        # 包括 marsh、bog 之类
    "water":        ["pond", "lake", "basin", "reservoir"],
    "waterway":     True,                        # 河渠
    "landuse":      ["reservoir"],
    "leisure":      ["pond", "swimming_pool"],   # 装饰池、泳池
    "amenity":      ["fishing", "boating"]       # （按需）
}

# === 3. 下载并过滤 ===
gdf = features.features_from_polygon(region_polygon, tags=tags)
gdf = gdf[gdf.geometry.type.isin(["Polygon", "MultiPolygon", "LineString"])]

# === 4. 设置 CRS & 裁剪 ===
if gdf.crs is None:
    gdf = gdf.set_crs("EPSG:4326")  # 明确 WGS84
gdf = gpd.clip(gdf, region_polygon)

# === 5. 组装 water_terrain（外环+孔洞） ===
def latlon_ring(r):
    """(lon,lat) ➜ [lat, lon]"""
    return [[lat, lon] for lon, lat in r.coords]

water_terrain = []
for geom in gdf.geometry:
    if isinstance(geom, Polygon):
        entry = [latlon_ring(geom.exterior)]
        entry += [latlon_ring(h) for h in geom.interiors]
        water_terrain.append(entry)
    elif isinstance(geom, MultiPolygon):
        for part in geom.geoms:
            entry = [latlon_ring(part.exterior)]
            entry += [latlon_ring(h) for h in part.interiors]
            water_terrain.append(entry)
    elif isinstance(geom, LineString):
        # 把 LineString 也按同样结构包装成一个“多边形”条目
        coords = latlon_ring(geom)
        water_terrain.append([coords])

# === 6. 美化输出 ===
def format_water_terrain(wt: list) -> str:
    lines = ["["]
    for i, poly in enumerate(wt):
        lines.append(f"        [  # Polygon {i}")
        for j, ring in enumerate(poly):
            tag = "# exterior" if j == 0 else f"# hole {j}"
            lines.append(f"            [  {tag}")
            for lat, lon in ring:
                lines.append(f"                [{lat}, {lon}],")
            lines.append("            ],")
        lines.append("        ],")
    lines.append("    ]")
    return "\n".join(lines)

water_terrain_str = format_water_terrain(water_terrain)

# === 7. 写出环境类文件 ===
class_name = "OSM_ENV"
starting_position = [[30.2791, 120.0564]]
boundary_points   = [[lat, lon] for lat, lon in polygon_coords]

env_code = f"""class {class_name}:
    def __init__(self, UAV):
        self.starting_position = {starting_position}
        self.boundary_points   = {boundary_points}
        self.geo_fencing_holes = None
        self.water_terrain     = {water_terrain_str}

        self.save_path = "{class_name}/"
        if UAV == 0:
            self.robot_FOV = 105
            self.robot_operating_height = 12
            self.robot_velocity = 10
            self.water_velocity = self.robot_velocity / 10
        if UAV == 1:
            self.robot_FOV = 60
            self.robot_operating_height = 8
            self.robot_velocity = 10
            self.water_velocity = self.robot_velocity / 4
        if UAV == 2:
            self.robot_FOV = 94
            self.robot_operating_height = 10
            self.robot_velocity = 15
            self.water_velocity = self.robot_velocity / 4
"""

out_path = str(BASE_DIR / "OSM_ENVIRONMENTS.py")
with open(out_path, "w", encoding="utf-8") as f:
    f.write(env_code)

print("Extract finish! Save file:", out_path)



# V0.2
# import osmnx as ox
# import geopandas as gpd
# from shapely.geometry import Polygon, MultiPolygon, LineString
# import pprint 

# # === 1. 定义检索窗口 (lat,lon) 转为 (lon,lat) ===
# polygon_coords = [
#     [30.2810115, 120.0523679],
#     [30.2764037, 120.0524527],
#     [30.2766892, 120.0577022],
#     [30.2811838, 120.0575735]
# ]
# region_polygon = Polygon([[lon, lat] for lat, lon in polygon_coords])

# # === 2. 定义 OSM 查询标签 ===
# tags = {"natural": "water", "waterway": True}

# # === 3. 查询并过滤 ===
# gdf = ox.features_from_polygon(region_polygon, tags=tags)
# gdf = gdf[gdf.geometry.type.isin(["Polygon", "MultiPolygon", "LineString"])]

# # === 4. 裁剪窗口 ===
# gdf = gpd.clip(gdf, region_polygon)

# # === 5. 构造 water_terrain 结构 ===
# def latlon_ring(r):
#     """(lon,lat) ring → [[lat,lon], ...]"""
#     return [[lat, lon] for lon, lat in r.coords]

# water_terrain = []

# for geom in gdf.geometry:
#     if isinstance(geom, Polygon):
#         poly_entry = [latlon_ring(geom.exterior)]
#         poly_entry += [latlon_ring(hole) for hole in geom.interiors]
#         water_terrain.append(poly_entry)

#     elif isinstance(geom, MultiPolygon):
#         for part in geom.geoms:
#             poly_entry = [latlon_ring(part.exterior)]
#             poly_entry += [latlon_ring(hole) for hole in part.interiors]
#             water_terrain.append(poly_entry)

#     elif isinstance(geom, LineString):
#         water_terrain.append([[ [lat, lon] for lon, lat in geom.coords ]])

# # === 6. 写出环境类文件 ===
# class_name = "OSM_ENV"
# starting_position = [[30.2791, 120.0564]]
# boundary_points   = [[lat, lon] for lat, lon in polygon_coords]

# # ① 先把 water_terrain 转成易读字符串
# water_terrain_str = pprint.pformat(
#     water_terrain,
#     indent=8,      # 每一级多缩进 8 个空格，和类属性对齐
#     width=100      # 超过 100 列就自动换行，可按需要调整
# )

# # ② 再拼装 env_code，把 {water_terrain_str} 放进去
# env_code = f"""class {class_name}:
#     def __init__(self, UAV):
#         self.starting_position = {starting_position}
#         self.boundary_points   = {boundary_points}
#         self.geo_fencing_holes = None
#         self.water_terrain     = {water_terrain_str}

#         self.save_path = "{class_name}/"
#         if UAV == 0:
#             self.robot_FOV = 105
#             self.robot_operating_height = 12
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 10
#         if UAV == 1:
#             self.robot_FOV = 60
#             self.robot_operating_height = 8
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 4
#         if UAV == 2:
#             self.robot_FOV = 94
#             self.robot_operating_height = 10
#             self.robot_velocity = 15
#             self.water_velocity = self.robot_velocity / 4
# """

# out_path = str(BASE_DIR / "OSM_ENVIRONMENTS.py")
# with open(out_path, "w", encoding="utf-8") as f:
#     f.write(env_code)

# print("Extract finish! Save file:", out_path)


# V0.1
# import osmnx as ox
# import geopandas as gpd
# from shapely.geometry import Polygon, MultiPolygon, LineString

# # === 1. 待提取多边形区域经纬度 (注意是 (lon, lat) 顺序) ===
# polygon_coords = [
#     [30.2785698, 120.0526039],  # 纬度在前，经度在后
#     [30.2766107, 120.0526304],
#     [30.2767687, 120.0577588],
#     [30.2787122, 120.0577011]
# ]

# # 构造 Polygon 时交换为 (lon, lat)（OSMnx 要求）
# region_polygon = Polygon([[lon, lat] for lat, lon in polygon_coords])

# # === 2. 查询水体相关标签 ===
# tags = {
#     "natural": "water",     # 湖泊、池塘
#     "waterway": True        # 河流、渠道等
# }

# # === 3. 提取数据 ===
# gdf = ox.features_from_polygon(region_polygon, tags=tags)
# gdf = gdf[gdf.geometry.type.isin(["Polygon", "MultiPolygon", "LineString"])]

# # === 4. 裁剪数据，只保留窗口内部部分 ===
# gdf_clipped = gpd.clip(gdf, region_polygon)

# # === 5. 提取水域几何，保留外边界和内边界 ===
# water_terrain = []
# for geom in gdf_clipped.geometry:
#     if isinstance(geom, Polygon):
#         # 提取外边界
#         outer_coords = list(geom.exterior.coords)
#         water_terrain.append([[lat, lon] for lon, lat in outer_coords])

#         # 提取内环（内边界），如果存在
#         for interior in geom.interiors:
#             inner_coords = list(interior.coords)
#             water_terrain.append([[lat, lon] for lon, lat in inner_coords])
#     elif isinstance(geom, MultiPolygon):
#         for part in geom.geoms:
#             # 提取外边界
#             outer_coords = list(part.exterior.coords)
#             water_terrain.append([[lat, lon] for lon, lat in outer_coords])

#             # 提取内环（内边界），如果存在
#             for interior in part.interiors:
#                 inner_coords = list(interior.coords)
#                 water_terrain.append([[lat, lon] for lon, lat in inner_coords])
#     elif isinstance(geom, LineString):
#         coords = list(geom.coords)
#         water_terrain.append([[lat, lon] for lon, lat in coords])

# # === 6. 构造 Python 环境类字符串 ===
# class_name = "OSM_ENV"
# starting_position = [[30.2791, 120.0564]]
# boundary_points = [[lat, lon] for lat, lon in polygon_coords]

# env_code = f"""class {class_name}:
#     def __init__(self, UAV):
#         self.starting_position = {starting_position}
#         self.boundary_points = {boundary_points}
#         self.geo_fencing_holes = None
#         self.water_terrain = {water_terrain}

#         self.save_path = "{class_name}/"
#         if UAV == 0:
#             self.robot_FOV = 105
#             self.robot_operating_height = 12
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 10
#         if UAV == 1:
#             self.robot_FOV = 60
#             self.robot_operating_height = 8
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 4
#         if UAV == 2:
#             self.robot_FOV = 94
#             self.robot_operating_height = 10
#             self.robot_velocity = 15
#             self.water_velocity = self.robot_velocity / 4
# """

# # === 7. 保存为 Python 文件 ===
# output_path = str(BASE_DIR / "OSM_ENVIRONMENTS.py")
# with open(output_path, "w", encoding="utf-8") as f:
#     f.write(env_code)

# print(f"Extract finish! Save file name：{output_path}")



# import osmnx as ox
# import geopandas as gpd
# from shapely.geometry import Polygon, MultiPolygon, LineString

# # === 1. 待提取多边形区域经纬度 (注意是 (lon, lat) 顺序) ===
# polygon_coords = [
#     [30.2810115, 120.0523679],  # 纬度在前，经度在后
#     [30.2764037, 120.0524527],
#     [30.2766892, 120.0577022],
#     [30.2811838, 120.0575735]
# ]

# # 构造 Polygon 时交换为 (lon, lat)（OSMnx 要求）
# region_polygon = Polygon([[lon, lat] for lat, lon in polygon_coords])

# # === 2. 查询水体相关标签 ===
# tags = {
#     "natural": "water",     # 湖泊、池塘
#     "waterway": True        # 河流、渠道等
# }

# # === 3. 提取数据 ===
# gdf = ox.features_from_polygon(region_polygon, tags=tags)
# gdf = gdf[gdf.geometry.type.isin(["Polygon", "MultiPolygon", "LineString"])]

# # === 4. 裁剪数据，只保留窗口内部部分 ===
# gdf_clipped = gpd.clip(gdf, region_polygon)

# # === 5. 提取边界经纬度 ===
# water_terrain = []
# for geom in gdf_clipped.geometry:
#     if isinstance(geom, Polygon):
#         coords = list(geom.exterior.coords)
#         water_terrain.append([[lat, lon] for lon, lat in coords])
#     elif isinstance(geom, MultiPolygon):
#         for part in geom.geoms:
#             coords = list(part.exterior.coords)
#             water_terrain.append([[lat, lon] for lon, lat in coords])
#     elif isinstance(geom, LineString):
#         coords = list(geom.coords)
#         water_terrain.append([[lat, lon] for lon, lat in coords])

# # === 6. 构造 Python 环境类字符串 ===
# class_name = "OSM_ENV"
# starting_position = [[30.2791, 120.0564]]
# boundary_points = [[lat, lon] for lat, lon in polygon_coords]

# env_code = f"""class {class_name}:
#     def __init__(self, UAV):
#         self.starting_position = {starting_position}
#         self.boundary_points = {boundary_points}
#         self.geo_fencing_holes = None
#         self.water_terrain = {water_terrain}

#         self.save_path = "{class_name}/"
#         if UAV == 0:
#             self.robot_FOV = 105
#             self.robot_operating_height = 12
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 10
#         if UAV == 1:
#             self.robot_FOV = 60
#             self.robot_operating_height = 8
#             self.robot_velocity = 10
#             self.water_velocity = self.robot_velocity / 4
#         if UAV == 2:
#             self.robot_FOV = 94
#             self.robot_operating_height = 10
#             self.robot_velocity = 15
#             self.water_velocity = self.robot_velocity / 4
# """

# # === 7. 保存为 Python 文件 ===
# output_path = str(BASE_DIR / "OSM_ENVIRONMENTS.py")
# with open(output_path, "w", encoding="utf-8") as f:
#     f.write(env_code)

# print(f"Extract finish! Save file name：{output_path}")

# V0.0
# import osmnx as ox
# import geopandas as gpd
# from shapely.geometry import Polygon

# # 设置多边形区域
# polygon_coords = [
#     (120.056027, 30.279638),  # 注意顺序是 (lon, lat)
#     (120.055844, 30.278429),
#     (120.056976, 30.278649),
#     (120.056850, 30.279756),
# ]
# region_polygon = Polygon(polygon_coords)

# # 查询标签（可扩展）
# tags = {
#     "natural": "water",     # 湖泊、池塘
#     "waterway": True        # 河流等
# }

# # 使用新版API
# gdf = ox.features_from_polygon(region_polygon, tags=tags)

# # 只保留需要的几何类型
# gdf = gdf[gdf.geometry.type.isin(["Polygon", "MultiPolygon", "LineString"])]

# # 保存为 GeoJSON 文件
# output_path = str(BASE_DIR / "extracted_water_bodies.geojson")
# gdf.to_file(output_path, driver="GeoJSON", encoding="utf-8")

# print("Extract finish!sava file name: extracted_water_bodies.geojson")
