"""核心点计算模块 (V0.1)

本模块基于步骤3生成的栅格数据与步骤2提取的环境数据，
为水域生成核心参考点集合：

- 无孔洞水域：返回几何质心
- 有孔洞水域：基于水域栅格点执行 K-Means 聚类，得到若干核心点

返回结构示例：

```
{
    'water_with_hole_core_points': {
        14: [[x1, y1], [x2, y2], ...],
        ...
    },
    'water_no_hole_core_points': {
        1: [[x1, y1]],
        ...
    }
}
```

后续若新增其他核心点生成策略，可保持一致的输入输出接口。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Tuple, Optional

import numpy as np
from shapely.geometry import Polygon, Point, MultiPoint
from shapely.prepared import prep

RNG_SEED = 42


@dataclass
class CorePointsResult:
    """封装核心点计算结果，便于后续扩展。"""

    water_with_hole_core_points: Dict[int, List[List[float]]]
    water_no_hole_core_points: Dict[int, List[List[float]]]
    water_with_hole_clusters: Optional[Dict[int, List[Dict[str, object]]]] = None

    def to_dict(self) -> Dict[str, Dict[int, List[List[float]]]]:
        result = {
            "water_with_hole_core_points": self.water_with_hole_core_points,
            "water_no_hole_core_points": self.water_no_hole_core_points,
        }
        if self.water_with_hole_clusters is not None:
            result["water_with_hole_clusters"] = self.water_with_hole_clusters
        return result


def compute_core_points(env_data: dict, grid_data: dict) -> Dict[str, Dict[int, List[List[float]]]]:
    """基于环境数据与栅格数据计算水域核心点。

    Args:
        env_data: extract_env_cartesian 返回的环境数据
        grid_data: GridGenerator.generate_grid 返回的栅格数据

    Returns:
        dict: 结构同 CorePointsResult.to_dict()
    """

    water_with_hole = env_data.get("water_with_hole", [])
    water_no_hole = env_data.get("water_no_hole", [])

    no_hole_core_points: Dict[int, List[List[float]]] = {}
    with_hole_core_points: Dict[int, List[List[float]]] = {}
    with_hole_clusters: Dict[int, List[Dict[str, object]]] = {}

    # ------------------------------------------------------------------
    # 1. 计算无孔洞水域的平均面积 (用于估算聚类数量)
    # ------------------------------------------------------------------
    no_hole_areas: List[float] = []
    for water in water_no_hole:
        polygon = Polygon(water["outer"])
        if polygon.is_empty:
            continue
        no_hole_areas.append(polygon.area)

    if no_hole_areas:
        avg_no_hole_area = sum(no_hole_areas) / len(no_hole_areas)
    else:
        # 没有无孔洞水域时，退化为使用带孔洞水域面积的平均值
        with_hole_areas = [Polygon(w["outer"], holes=w["holes"]).area for w in water_with_hole]
        avg_no_hole_area = sum(with_hole_areas) / len(with_hole_areas) if with_hole_areas else 1.0

    # 避免后续除零
    if avg_no_hole_area <= 0:
        avg_no_hole_area = 1.0

    # ------------------------------------------------------------------
    # 2. 无孔洞水域：直接取几何中心
    # ------------------------------------------------------------------
    for water in water_no_hole:
        idx = water.get("idx")
        polygon = Polygon(water["outer"])
        if polygon.is_empty:
            continue

        centroid = polygon.centroid
        no_hole_core_points[idx] = [[centroid.x, centroid.y]]

    # ------------------------------------------------------------------
    # 3. 带孔洞水域：使用栅格水域点进行 K-Means 聚类
    # ------------------------------------------------------------------
    water_mask = grid_data.get("is_water")
    coordinates = grid_data.get("coordinates")

    if water_mask is None or coordinates is None:
        raise ValueError("grid_data 中缺少 is_water 或 coordinates 字段")

    water_points = coordinates[water_mask]

    if water_points.size == 0:
        # 没有任何水域栅格，直接返回已有结果
        return CorePointsResult(with_hole_core_points, no_hole_core_points, with_hole_clusters).to_dict()

    for water in water_with_hole:
        idx = water.get("idx")
        polygon = Polygon(water["outer"], holes=water["holes"])
        if polygon.is_empty or polygon.area <= 0:
            continue

        cluster_count = max(1, math.ceil(polygon.area / avg_no_hole_area))

        # 过滤落在当前水域的栅格点
        prepared_polygon = prep(polygon)
        candidate_mask = np.array([prepared_polygon.contains(Point(pt)) for pt in water_points])
        candidate_points = water_points[candidate_mask]

        # 若候选点不足，退化为随机采样
        if candidate_points.shape[0] < cluster_count:
            candidate_points = _sample_points_within_polygon(
                polygon, max(cluster_count * 5, 50), seed=RNG_SEED
            )

        if candidate_points.shape[0] == 0:
            continue

        unique_point_count = max(1, np.unique(candidate_points, axis=0).shape[0])
        adjusted_cluster_count = min(cluster_count, unique_point_count)
        centroids, assignments = _kmeans(candidate_points, adjusted_cluster_count, seed=RNG_SEED)
        centroid_list: List[List[float]] = centroids.tolist()

        # 若聚类点不在水域内，将其坐标移动到距离最近的水域栅格中心
        for cluster_idx in range(adjusted_cluster_count):
            c = centroid_list[cluster_idx]
            if not polygon.contains(Point(c[0], c[1])):
                # 在 candidate_points（当前水域内的水域栅格中心）中找最近点
                dists = np.linalg.norm(candidate_points - np.array(c, dtype=float), axis=1)
                nearest_idx = int(np.argmin(dists))
                centroid_list[cluster_idx] = candidate_points[nearest_idx].tolist()

        with_hole_core_points[idx] = centroid_list

        cluster_details: List[Dict[str, object]] = []
        for cluster_idx in range(adjusted_cluster_count):
            cluster_points = candidate_points[assignments == cluster_idx]
            if cluster_points.size == 0:
                continue

            hull_coords: Optional[List[List[float]]] = None
            if cluster_points.shape[0] >= 3:
                hull = MultiPoint(cluster_points).convex_hull
                if hull.geom_type == "Polygon" and hull.area > 0:
                    hull_coords = np.asarray(hull.exterior.coords[:-1], dtype=float).tolist()

            cluster_details.append(
                {
                    "centroid": centroid_list[cluster_idx],
                    "points": cluster_points.tolist(),
                    "hull": hull_coords,
                }
            )

        if cluster_details:
            with_hole_clusters[idx] = cluster_details

    return CorePointsResult(with_hole_core_points, no_hole_core_points, with_hole_clusters).to_dict()


def _sample_points_within_polygon(
    polygon: Polygon,
    sample_size: int,
    seed: int = RNG_SEED,
) -> np.ndarray:
    """在多边形内部采样指定数量的点。"""

    if polygon.is_empty:
        return np.empty((0, 2))

    rng = np.random.default_rng(seed)
    minx, miny, maxx, maxy = polygon.bounds

    samples: List[Tuple[float, float]] = []
    max_attempts = sample_size * 10
    attempts = 0

    while len(samples) < sample_size and attempts < max_attempts:
        attempts += 1
        x = rng.uniform(minx, maxx)
        y = rng.uniform(miny, maxy)
        point = Point(x, y)
        if polygon.contains(point):
            samples.append((x, y))

    return np.array(samples, dtype=float)


def _kmeans(
    points: np.ndarray,
    cluster_count: int,
    max_iter: int = 100,
    tol: float = 1e-3,
    seed: int = RNG_SEED,
) -> Tuple[np.ndarray, np.ndarray]:
    """简易 K-Means 聚类实现，满足当前核心点计算需求。"""

    if cluster_count <= 0:
        raise ValueError("cluster_count 必须为正数")

    if points.shape[0] <= cluster_count:
        unique_points = np.unique(points, axis=0)
        limited = unique_points[:cluster_count]
        assignments = np.zeros(points.shape[0], dtype=int)
        if limited.shape[0] > 0 and points.shape[0] > 0:
            assignments = np.arange(points.shape[0]) % limited.shape[0]
        return limited, assignments

    rng = np.random.default_rng(seed)
    centroids = points[rng.choice(points.shape[0], cluster_count, replace=False)]
    assignments = np.zeros(points.shape[0], dtype=int)

    for _ in range(max_iter):
        # 分配样本到最近的质心
        distances = np.linalg.norm(points[:, None, :] - centroids[None, :, :], axis=2)
        assignments = np.argmin(distances, axis=1)

        new_centroids = np.zeros_like(centroids)
        for k in range(cluster_count):
            cluster_points = points[assignments == k]
            if cluster_points.size == 0:
                # 空簇：随机重置为任意样本
                new_centroids[k] = points[rng.integers(0, points.shape[0])]
            else:
                new_centroids[k] = cluster_points.mean(axis=0)

        shift = np.linalg.norm(new_centroids - centroids)
        centroids = new_centroids

        if shift <= tol:
            break

    return centroids, assignments


__all__ = ["compute_core_points"]





