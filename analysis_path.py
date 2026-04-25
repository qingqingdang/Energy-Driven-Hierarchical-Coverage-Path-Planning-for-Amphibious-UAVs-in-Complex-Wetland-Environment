"""
路径评估模块（独立运行）

读取 combine_path 生成的路径或其他方法生成的路径文件（每行 x y），
计算：覆盖率、转弯次数、路径长度、能量消耗，并输出评估报告。

指标定义：
- 覆盖率 = (水域路径长度 × 4) / 所有水域面积（可视化蓝色部分）
- 转弯次数 = 路径方向变化超过阈值的次数（默认 30°）
- 路径长度 = 总长度（米）；水域内长度、非水域内长度分开统计
- 能量 = 水域内长度×1 + 非水域内长度×10 + 起飞次数×10 + 降落次数×5
  （水域→非水域视为起飞，非水域→水域视为降落）
"""

import os
import numpy as np
from typing import List, Dict, Optional, Tuple
from shapely.geometry import Polygon, Point
from datetime import datetime

# 默认路径（相对本脚本所在目录）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH_FILE = os.path.join(SCRIPT_DIR, "data", "combined_path.txt")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "data")
REPORT_FILENAME = "path_analysis_report.txt"

# 覆盖率线宽（与采样间距一致，米）
COVERAGE_LINE_WIDTH = 4.0
# 能量系数
ENERGY_WATER_PER_M = 1.0
ENERGY_NON_WATER_PER_M = 10.0
ENERGY_TAKEOFF = 10.0
ENERGY_LANDING = 5.0
# 转弯角度阈值（度）
TURN_ANGLE_THRESHOLD_DEG = 30.0


def load_path_from_file(file_path: str) -> List[List[float]]:
    """
    从文件加载路径点序列。支持 # 开头的注释行，每行有效内容为 "x y"。

    :param file_path: 路径文件路径
    :return: [[x, y], ...]，失败返回 []
    """
    if not os.path.isfile(file_path):
        return []
    points = []
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 2:
                    x, y = float(parts[0]), float(parts[1])
                    points.append([x, y])
    except Exception:
        return []
    return points


def load_water_polygons_from_env() -> Tuple[List[Polygon], float]:
    """
    从当前项目环境（OSM_ENV + extract_map）加载所有水域多边形并计算总面积。

    :return: (water_polygons, total_water_area)。若加载失败返回 ([], 0.0)
    """
    try:
        from OSM_ENVIRONMENTS import OSM_ENV
        from extract_map import extract_env_cartesian
    except ImportError:
        return [], 0.0
    try:
        env = OSM_ENV(UAV=1)
        env_data = extract_env_cartesian(env, shrink_dist=0.5, sample_dist=10)
    except Exception:
        return [], 0.0
    water_no_hole = env_data.get("water_no_hole") or []
    water_with_hole = env_data.get("water_with_hole") or []
    polygons = []
    for w in water_no_hole:
        outer = w.get("outer")
        if outer is not None and len(outer) >= 3:
            try:
                poly = Polygon(outer)
                if not poly.is_empty:
                    polygons.append(poly)
            except Exception:
                pass
    for w in water_with_hole:
        outer = w.get("outer")
        holes = w.get("holes") or []
        if outer is not None and len(outer) >= 3:
            try:
                poly = Polygon(outer, holes=holes)
                if not poly.is_empty:
                    polygons.append(poly)
            except Exception:
                pass
    total_area = sum(p.area for p in polygons)
    return polygons, total_area


def _point_in_water(x: float, y: float, water_polygons: List[Polygon]) -> bool:
    pt = Point(x, y)
    for poly in water_polygons:
        try:
            if poly.contains(pt) or pt.within(poly):
                return True
        except Exception:
            continue
    return False


def analyze_path(
    points: List[List[float]],
    water_polygons: Optional[List[Polygon]] = None,
    total_water_area: Optional[float] = None,
    line_width: float = COVERAGE_LINE_WIDTH,
    turn_threshold_deg: float = TURN_ANGLE_THRESHOLD_DEG,
) -> Dict:
    """
    对路径点序列做评估：路径长度、水域/非水域分段、起飞降落、转弯次数、覆盖率、能量。

    :param points: 路径点序列 [[x,y], ...]
    :param water_polygons: 水域多边形列表（Shapely），None 则仅做长度与转弯
    :param total_water_area: 水域总面积（平方米），None 时从 water_polygons 计算
    :param line_width: 覆盖率公式中的线宽（米）
    :param turn_threshold_deg: 转弯角度阈值（度）
    :return: 指标字典
    """
    n = len(points)
    if n < 2:
        return {
            "total_length": 0.0,
            "water_length": 0.0,
            "non_water_length": 0.0,
            "takeoff_count": 0,
            "landing_count": 0,
            "turn_count": 0,
            "coverage_ratio": 0.0,
            "total_water_area": 0.0,
            "energy": 0.0,
            "point_count": n,
        }
    if total_water_area is None and water_polygons:
        total_water_area = sum(p.area for p in water_polygons)
    total_water_area = total_water_area or 0.0
    in_water = []
    if water_polygons:
        in_water = [_point_in_water(p[0], p[1], water_polygons) for p in points]
    else:
        in_water = [False] * n

    total_length = 0.0
    water_length = 0.0
    non_water_length = 0.0
    takeoff_count = 0
    landing_count = 0

    for i in range(n - 1):
        a, b = points[i], points[i + 1]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        seg_len = np.sqrt(dx * dx + dy * dy)
        total_length += seg_len
        in_a, in_b = in_water[i], in_water[i + 1]
        if in_a and in_b:
            water_length += seg_len
        elif not in_a and not in_b:
            non_water_length += seg_len
        else:
            if in_a and not in_b:
                takeoff_count += 1
            else:
                landing_count += 1
            non_water_length += seg_len

    turn_count = 0
    thresh_rad = np.radians(turn_threshold_deg)
    for i in range(1, n - 1):
        p1 = np.array(points[i - 1])
        p2 = np.array(points[i])
        p3 = np.array(points[i + 1])
        v1 = p2 - p1
        v2 = p3 - p2
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 > 1e-10 and n2 > 1e-10:
            cos_a = np.dot(v1, v2) / (n1 * n2)
            cos_a = np.clip(cos_a, -1.0, 1.0)
            angle = np.arccos(cos_a)
            if angle > thresh_rad:
                turn_count += 1

    coverage_ratio = (water_length * line_width) / total_water_area if total_water_area > 0 else 0.0
    energy = (
        water_length * ENERGY_WATER_PER_M
        + non_water_length * ENERGY_NON_WATER_PER_M
        + takeoff_count * ENERGY_TAKEOFF
        + landing_count * ENERGY_LANDING
    )

    return {
        "total_length": total_length,
        "water_length": water_length,
        "non_water_length": non_water_length,
        "takeoff_count": takeoff_count,
        "landing_count": landing_count,
        "turn_count": turn_count,
        "coverage_ratio": coverage_ratio,
        "total_water_area": total_water_area,
        "energy": energy,
        "point_count": n,
        "line_width": line_width,
        "turn_threshold_deg": turn_threshold_deg,
    }


def generate_report(metrics: Dict, path_file: str, output_path: str) -> None:
    """将评估结果写入文本报告。"""
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("路径评估报告\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"路径文件: {path_file}\n")
        f.write(f"点数: {metrics['point_count']}\n\n")
        f.write("路径长度 (m)\n")
        f.write("-" * 40 + "\n")
        f.write(f"  总长度:     {metrics['total_length']:.2f}\n")
        f.write(f"  水域内:     {metrics['water_length']:.2f}\n")
        f.write(f"  非水域内:   {metrics['non_water_length']:.2f}\n\n")
        f.write("起降与能量\n")
        f.write("-" * 40 + "\n")
        f.write(f"  起飞次数:   {metrics['takeoff_count']}\n")
        f.write(f"  降落次数:   {metrics['landing_count']}\n")
        f.write(f"  能量消耗:   {metrics['energy']:.2f}\n")
        f.write(f"    (水域×1 + 非水域×10 + 起飞×10 + 降落×5)\n\n")
        f.write("其他指标\n")
        f.write("-" * 40 + "\n")
        f.write(f"  转弯次数 (>{metrics['turn_threshold_deg']}°): {metrics['turn_count']}\n")
        f.write(f"  水域总面积 (m²): {metrics['total_water_area']:.2f}\n")
        f.write(f"  覆盖率 (水域长×{metrics['line_width']}/水域面积): {metrics['coverage_ratio']:.4f}\n")
    print(f"  报告已保存: {output_path}")


def run_analysis(
    path_file: str = DEFAULT_PATH_FILE,
    output_dir: str = OUTPUT_DIR,
    report_filename: str = REPORT_FILENAME,
    load_water: bool = True,
) -> Dict:
    """
    独立运行：加载路径 → 可选加载水域 → 分析 → 写报告。

    :param path_file: 路径文件路径
    :param output_dir: 报告输出目录
    :param report_filename: 报告文件名
    :param load_water: 是否从环境加载水域（用于覆盖率与能量中的水域/非水域划分）
    :return: 评估指标字典
    """
    print("=" * 60)
    print("路径评估（analysis_path）")
    print("=" * 60)
    print(f"路径文件: {path_file}")
    points = load_path_from_file(path_file)
    if not points:
        print("  未读取到有效路径点，请检查文件路径与格式（每行 x y）。")
        return {}
    print(f"  已加载 {len(points)} 个点")
    water_polygons = []
    total_water_area = 0.0
    if load_water:
        water_polygons, total_water_area = load_water_polygons_from_env()
        if water_polygons:
            print(f"  已加载 {len(water_polygons)} 个水域多边形，总面积 {total_water_area:.2f} m²")
        else:
            print("  未加载到水域数据，覆盖率与水域/非水域能量将不可用")
    metrics = analyze_path(
        points,
        water_polygons=water_polygons if water_polygons else None,
        total_water_area=total_water_area if total_water_area > 0 else None,
        line_width=COVERAGE_LINE_WIDTH,
        turn_threshold_deg=TURN_ANGLE_THRESHOLD_DEG,
    )
    print("\n评估结果:")
    print(f"  总路径长度:   {metrics['total_length']:.2f} m")
    print(f"  水域内长度:   {metrics['water_length']:.2f} m")
    print(f"  非水域内长度: {metrics['non_water_length']:.2f} m")
    print(f"  起飞次数:     {metrics['takeoff_count']}")
    print(f"  降落次数:     {metrics['landing_count']}")
    print(f"  能量消耗:     {metrics['energy']:.2f}")
    print(f"  转弯次数:     {metrics['turn_count']}")
    print(f"  覆盖率:       {metrics['coverage_ratio']:.4f}")
    report_path = os.path.join(output_dir, report_filename)
    generate_report(metrics, path_file, report_path)
    print("=" * 60)
    return metrics


def main():
    """主函数：使用默认或命令行指定的路径文件运行评估。"""
    import sys
    path_file = DEFAULT_PATH_FILE
    if len(sys.argv) >= 2:
        path_file = sys.argv[1]
    run_analysis(
        path_file=path_file,
        output_dir=OUTPUT_DIR,
        report_filename=REPORT_FILENAME,
        load_water=True,
    )


if __name__ == "__main__":
    main()
