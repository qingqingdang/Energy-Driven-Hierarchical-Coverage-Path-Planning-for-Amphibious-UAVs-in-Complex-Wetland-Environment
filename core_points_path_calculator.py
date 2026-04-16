"""
核心点路径计算模块 (V0.3)

本模块提供核心点之间的路径计算功能：
- 合并五角星点和菱形点
- 计算所有点对之间的最优路径和成本
- 保存成本矩阵和详细路径信息
- 调用LKH求解器进行TSP求解
- 从文件读取路径信息用于可视化
"""

import time
import os
import re
import ast
import numpy as np
from AStar.grid_based_a_star import GridBasedAStar
from paths import DATA_DIR


def merge_core_points(no_hole_points, with_hole_points):
    """
    合并五角星点和菱形点，保留元信息
    
    :param no_hole_points: 无孔洞水域核心点字典 {water_idx: [[x, y], ...]}
    :param with_hole_points: 有孔洞水域核心点字典 {water_idx: [[x, y], ...]}
    :return: 合并后的点列表，每个元素包含坐标和元信息
    """
    merged_points = []
    
    # 添加无孔洞水域的核心点
    for water_idx, point_list in no_hole_points.items():
        for point_idx, point in enumerate(point_list):
            merged_points.append({
                'point': point,  # [x, y]
                'type': 'no_hole',
                'water_idx': water_idx,
                'point_idx': point_idx
            })
    
    # 添加有孔洞水域的核心点
    for water_idx, point_list in with_hole_points.items():
        for cluster_idx, point in enumerate(point_list):
            merged_points.append({
                'point': point,  # [x, y]
                'type': 'with_hole',
                'water_idx': water_idx,
                'cluster_idx': cluster_idx
            })
    
    return merged_points


def calculate_all_pairs_paths(merged_points, grid_data, cost_calculator, save_path='core_points_all_pairs_paths.txt'):
    """
    计算所有点对之间的最优路径和成本，并保存到文件
    
    :param merged_points: 合并后的核心点列表
    :param grid_data: 栅格数据
    :param cost_calculator: 成本计算器
    :param save_path: 保存文件路径
    :return: 成本矩阵和路径字典
    """
    n = len(merged_points)
    if n == 0:
        print("  错误: 没有核心点可计算")
        return None, None
    
    print(f"  总核心点数: {n}")
    print(f"  需要计算的点对数量: {n * (n - 1) // 2}")
    
    # 初始化A*规划器
    a_star_planner = GridBasedAStar(grid_data, cost_calculator)
    
    # 初始化成本矩阵（对称矩阵）
    cost_matrix = np.full((n, n), np.inf)
    distance_matrix = np.full((n, n), np.inf)
    
    # 存储所有路径（可选，如果内存允许）
    paths_dict = {}
    failed_pairs = []
    
    # 计算所有点对（i < j）
    total_pairs = n * (n - 1) // 2
    calculated = 0
    start_time = time.time()
    
    print(f"  开始计算所有点对路径...")
    
    for i in range(n):
        for j in range(i + 1, n):
            point_i = merged_points[i]['point']
            point_j = merged_points[j]['point']
            
            # 计算路径
            path_result = a_star_planner.plan(
                point_i[0], point_i[1],
                point_j[0], point_j[1]
            )
            
            if path_result['success']:
                cost = path_result['cost']
                distance = path_result['distance']
                
                # 填充对称矩阵
                cost_matrix[i, j] = cost
                cost_matrix[j, i] = cost
                distance_matrix[i, j] = distance
                distance_matrix[j, i] = distance
                
                # 保存路径信息
                paths_dict[f"{i}->{j}"] = {
                    'cost': cost,
                    'distance': distance,
                    'path': path_result['path'],
                    'path_length': len(path_result['path']),
                    'from_info': merged_points[i],
                    'to_info': merged_points[j]
                }
            else:
                # 路径规划失败
                failed_pairs.append((i, j, path_result.get('error', '未知错误')))
            
            # 更新进度
            calculated += 1
            if calculated % max(1, total_pairs // 20) == 0 or calculated == total_pairs:
                elapsed = time.time() - start_time
                progress = calculated / total_pairs * 100
                if calculated > 0:
                    avg_time = elapsed / calculated
                    remaining = (total_pairs - calculated) * avg_time
                    print(f"    进度: {calculated}/{total_pairs} ({progress:.1f}%) | "
                          f"已用时间: {elapsed:.1f}s | 预计剩余: {remaining:.1f}s")
    
    total_time = time.time() - start_time
    print(f"  ✓ 计算完成！总耗时: {total_time:.2f}秒")
    
    if failed_pairs:
        print(f"  ⚠ 警告: {len(failed_pairs)} 对点无法找到路径")
        for i, j, error in failed_pairs[:5]:  # 只显示前5个
            print(f"    点对 ({i}, {j}): {error}")
        if len(failed_pairs) > 5:
            print(f"    ... 还有 {len(failed_pairs) - 5} 对失败的点对")
    
    # 保存到文件
    try:
        save_all_pairs_results(merged_points, cost_matrix, distance_matrix, paths_dict, save_path)
        print(f"  ✓ 文件保存成功！")
    except Exception as e:
        print(f"  ✗ 文件保存失败: {e}")
        import traceback
        traceback.print_exc()
    
    return cost_matrix, paths_dict


def save_tsp_file(merged_points, cost_matrix, data_dir_full, tsp_name="core_points.tsp"):
    """
    保存成本矩阵为TSP格式文件
    
    :param merged_points: 合并后的核心点列表
    :param cost_matrix: 成本矩阵
    :param data_dir_full: 保存目录（绝对路径）
    :param tsp_name: TSP文件名
    """
    n = len(merged_points)
    tsp_path = os.path.join(data_dir_full, tsp_name)
    
    print(f"  保存TSP文件到: {tsp_path}")
    
    # 计算最大可达成本（用于处理不可达点对）
    max_reachable_cost = 0.0
    for i in range(n):
        for j in range(n):
            if np.isfinite(cost_matrix[i, j]) and cost_matrix[i, j] > max_reachable_cost:
                max_reachable_cost = cost_matrix[i, j]
    
    # 不可达点对使用最大可达成本 × 100
    unreachable_cost = int(round(max_reachable_cost * 100))
    if unreachable_cost == 0:
        unreachable_cost = 999999  # 如果所有点对都不可达，使用默认大值
    
    with open(tsp_path, 'w', encoding='utf-8') as f:
        f.write(f"NAME: {tsp_name.replace('.tsp', '')}\n")
        f.write(f"TYPE: TSP\n")
        f.write(f"COMMENT: core_points problem with explicit distance matrix\n")
        f.write(f"DIMENSION: {n}\n")
        f.write(f"EDGE_WEIGHT_TYPE: EXPLICIT\n")
        f.write(f"EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
        f.write(f"EDGE_WEIGHT_SECTION\n")
        
        # 写入成本矩阵（转换为整数，四舍五入）
        for i in range(n):
            row = []
            for j in range(n):
                if i == j:
                    # 点到自己本身的距离为0
                    cost_int = 0
                elif np.isfinite(cost_matrix[i, j]):
                    # 四舍五入转换为整数
                    cost_int = int(round(cost_matrix[i, j]))
                else:
                    # 不可达点对使用最大可达成本 × 100
                    cost_int = unreachable_cost
                row.append(str(cost_int))
            f.write(" ".join(row) + "\n")
        
        f.write("EOF\n")
    
    print(f"  ✓ TSP文件保存完成！")
    print(f"    不可达点对成本: {unreachable_cost}")
    print(f"    最大可达成本: {max_reachable_cost:.2f}")


def save_par_file(data_dir_full, n_points, tsp_name="core_points.tsp", 
                  par_name="core_points.par", solution_name="core_points_solution.txt"):
    """
    保存LKH求解器的PAR参数文件
    
    :param data_dir_full: 保存目录（绝对路径）
    :param n_points: 点数
    :param tsp_name: TSP文件名
    :param par_name: PAR文件名
    :param solution_name: 解决方案文件名
    """
    par_path = os.path.join(data_dir_full, par_name)
    
    # 计算点对数量
    total_pairs = n_points * (n_points - 1) // 2
    
    print(f"  保存PAR文件到: {par_path}")
    print(f"    点数: {n_points}, 点对数量: {total_pairs}")
    
    with open(par_path, 'w', encoding='utf-8') as f:
        f.write(f"PROBLEM_FILE = {tsp_name}\n")
        f.write(f"OUTPUT_TOUR_FILE = {solution_name}\n")
        f.write(f"RUNS = 1\n")
        f.write(f"MAX_TRIALS = {total_pairs}\n")
        f.write(f"MOVE_TYPE = 5\n")
        f.write(f"MAX_CANDIDATES = 50 SYMMETRIC\n")
        f.write(f"TRACE_LEVEL = 1\n")
    
    print(f"  ✓ PAR文件保存完成！")


def save_paths_file(merged_points, paths_dict, data_dir_full, paths_name="core_points_path.txt"):
    """
    保存所有点对的路径信息（详细格式）
    
    :param merged_points: 合并后的核心点列表
    :param paths_dict: 路径字典
    :param data_dir_full: 保存目录（绝对路径）
    :param paths_name: 路径文件名
    """
    paths_path = os.path.join(data_dir_full, paths_name)
    n = len(merged_points)
    
    print(f"  保存路径文件到: {paths_path}")
    
    with open(paths_path, 'w', encoding='utf-8') as f:
        f.write("# 核心点路径详细信息\n")
        f.write(f"# 总点数: {n}\n")
        f.write(f"# 总路径数: {len(paths_dict)}\n\n")
        
        # 写入点信息
        f.write("# 点索引信息:\n")
        for idx, point_info in enumerate(merged_points):
            point = point_info['point']
            if point_info['type'] == 'no_hole':
                f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                       f"无孔洞水域 P{point_info['water_idx']}\n")
            else:
                f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                       f"有孔洞水域 P{point_info['water_idx']}, 聚类 {point_info['cluster_idx']}\n")
        f.write("\n")
        
        # 写入路径信息（详细格式）
        f.write("# 路径详细信息 (格式: from_idx->to_idx)\n")
        f.write("# 每个路径包含: Cost, Distance, Path Length, From, To, Path Points\n\n")
        
        for key, path_info in sorted(paths_dict.items()):
            from_idx, to_idx = map(int, key.split('->'))
            f.write(f"{key}:\n")
            f.write(f"  Cost: {path_info['cost']:.6f}\n")
            f.write(f"  Distance: {path_info['distance']:.6f} m\n")
            f.write(f"  Path Length: {path_info['path_length']} points\n")
            f.write(f"  From: {path_info['from_info']}\n")
            f.write(f"  To: {path_info['to_info']}\n")
            f.write(f"  Path Points: {path_info['path']}\n")
            f.write("\n")
    
    print(f"  ✓ 路径文件保存完成！")


def save_all_pairs_results(merged_points, cost_matrix, distance_matrix, paths_dict, save_path):
    """
    保存所有点对的路径结果到文件
    
    :param merged_points: 合并后的核心点列表
    :param cost_matrix: 成本矩阵
    :param distance_matrix: 距离矩阵
    :param paths_dict: 路径字典
    :param save_path: 保存文件路径
    """
    try:
        data_dir_full = str(DATA_DIR)
        print(f"  目标保存目录: {data_dir_full}")
        
        # 确保 data 文件夹存在
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        
        base_path = save_path.replace('.txt', '')
        n = len(merged_points)
        
        # 1. 保存TSP格式文件
        save_tsp_file(merged_points, cost_matrix, data_dir_full, tsp_name="core_points.tsp")
        
        # 2. 保存PAR格式文件
        save_par_file(data_dir_full, n_points=n, 
                     tsp_name="core_points.tsp",
                     par_name="core_points.par",
                     solution_name="core_points_solution.txt")
        
        # 3. 保存路径文件
        save_paths_file(merged_points, paths_dict, data_dir_full, paths_name="core_points_path.txt")
        
        # 4. 保留原有的成本矩阵和详细路径文件（向后兼容）
        cost_matrix_path = os.path.join(data_dir_full, f"{base_path}_cost_matrix.txt")
        detail_path = os.path.join(data_dir_full, f"{base_path}_detail.txt")
        
        # 保存成本矩阵
        print(f"  保存成本矩阵到: {cost_matrix_path}")
        with open(cost_matrix_path, 'w', encoding='utf-8') as f:
            f.write("# 核心点成本矩阵\n")
            f.write(f"# 总点数: {n}\n")
            f.write("# 格式: 对称矩阵，cost_matrix[i][j] 表示从点i到点j的成本\n")
            f.write("# 不可达路径用 inf 表示\n\n")
            
            # 写入点信息
            f.write("# 点索引信息:\n")
            for idx, point_info in enumerate(merged_points):
                point = point_info['point']
                if point_info['type'] == 'no_hole':
                    f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                           f"无孔洞水域 P{point_info['water_idx']}\n")
                else:
                    f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                           f"有孔洞水域 P{point_info['water_idx']}, 聚类 {point_info['cluster_idx']}\n")
            f.write("\n")
            
            # 写入成本矩阵
            f.write("# 成本矩阵:\n")
            for i in range(n):
                row_str = " ".join([f"{cost_matrix[i, j]:.6f}" if np.isfinite(cost_matrix[i, j]) else "inf" 
                                   for j in range(n)])
                f.write(f"{row_str}\n")
        
        # 保存详细路径信息
        print(f"  保存详细路径信息到: {detail_path}")
        with open(detail_path, 'w', encoding='utf-8') as f:
            f.write("# 核心点路径详细信息\n")
            f.write(f"# 总点数: {n}\n")
            f.write(f"# 总路径数: {len(paths_dict)}\n\n")
            
            # 写入点信息
            f.write("# 点索引信息:\n")
            for idx, point_info in enumerate(merged_points):
                point = point_info['point']
                if point_info['type'] == 'no_hole':
                    f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                           f"无孔洞水域 P{point_info['water_idx']}\n")
                else:
                    f.write(f"# {idx}: ({point[0]:.2f}, {point[1]:.2f}) - "
                           f"有孔洞水域 P{point_info['water_idx']}, 聚类 {point_info['cluster_idx']}\n")
            f.write("\n")
            
            # 写入路径信息
            f.write("# 路径详细信息 (格式: from_idx->to_idx: cost, distance, path_length, path_points)\n")
            for key, path_info in sorted(paths_dict.items()):
                from_idx, to_idx = map(int, key.split('->'))
                f.write(f"\n{key}:\n")
                f.write(f"  Cost: {path_info['cost']:.6f}\n")
                f.write(f"  Distance: {path_info['distance']:.6f} m\n")
                f.write(f"  Path Length: {path_info['path_length']} points\n")
                f.write(f"  From: {path_info['from_info']}\n")
                f.write(f"  To: {path_info['to_info']}\n")
                f.write(f"  Path Points: {path_info['path']}\n")
        
        print(f"\n  ✓ 所有文件保存完成！")
        print(f"    TSP文件: {os.path.join(data_dir_full, 'core_points.tsp')}")
        print(f"    PAR文件: {os.path.join(data_dir_full, 'core_points.par')}")
        print(f"    路径文件: {os.path.join(data_dir_full, 'core_points_path.txt')}")
        print(f"    成本矩阵文件: {cost_matrix_path}")
        print(f"    详细路径文件: {detail_path}")
        
    except Exception as e:
        print(f"  ✗ 保存文件时发生错误: {e}")
        import traceback
        traceback.print_exc()
        raise


def load_paths_from_file(paths_file_path):
    """
    从文件读取所有点对的路径信息
    
    :param paths_file_path: 路径文件路径（如 core_points_path.txt）
    :return: 路径字典，格式: {f"{i}->{j}": {'cost': float, 'distance': float, 'path': list, ...}}
    """
    if not os.path.exists(paths_file_path):
        print(f"  ✗ 路径文件不存在: {paths_file_path}")
        return None
    
    print(f"  读取路径文件: {paths_file_path}")
    paths_dict = {}
    
    try:
        with open(paths_file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            # 检查是否是路径键（格式: from_idx->to_idx:）
            if re.match(r'^\d+->\d+:$', line):
                key = line[:-1]  # 去掉末尾的冒号
                from_idx, to_idx = map(int, key.split('->'))
                
                # 读取路径信息
                cost = None
                distance = None
                path_points = None
                
                i += 1
                while i < len(lines):
                    line = lines[i].strip()
                    
                    # 如果遇到下一个路径键或空行，停止
                    if re.match(r'^\d+->\d+:$', line) or (not line and cost is not None):
                        break
                    
                    # 解析Cost
                    if line.startswith('Cost:'):
                        cost = float(line.split(':', 1)[1].strip())
                    
                    # 解析Distance
                    elif line.startswith('Distance:'):
                        distance_str = line.split(':', 1)[1].strip()
                        distance = float(distance_str.split()[0])  # 提取数字部分
                    
                    # 解析Path Points
                    elif line.startswith('Path Points:'):
                        # 提取路径点字符串（去掉"Path Points: "前缀）
                        path_str = line.split(':', 1)[1].strip()
                        
                        # 替换np.float64(...)为普通数字
                        # 使用正则表达式匹配 np.float64(数字) 并替换为数字
                        path_str = re.sub(r'np\.float64\(([^)]+)\)', r'\1', path_str)
                        
                        try:
                            # 使用ast.literal_eval安全地解析列表
                            path_points = ast.literal_eval(path_str)
                            # 转换为numpy数组格式（列表的列表）
                            if path_points:
                                path_points = [[float(p[0]), float(p[1])] for p in path_points]
                        except Exception as e:
                            print(f"    警告: 解析路径点失败 ({key}): {e}")
                            path_points = None
                    
                    i += 1
                
                # 如果成功解析了所有信息，保存到字典
                if cost is not None and distance is not None and path_points is not None:
                    paths_dict[key] = {
                        'cost': cost,
                        'distance': distance,
                        'path': path_points,
                        'path_length': len(path_points)
                    }
                else:
                    print(f"    警告: 路径信息不完整 ({key})")
                
                # 回退一行，因为外层循环会继续
                i -= 1
            else:
                i += 1
        
        print(f"  ✓ 成功读取 {len(paths_dict)} 条路径")
        return paths_dict
        
    except Exception as e:
        print(f"  ✗ 读取路径文件时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None


def load_tsp_tour_from_file(tsp_path_file):
    """
    从文件读取LKH求解的TSP访问顺序
    
    :param tsp_path_file: TSP路径文件路径（如 core_points_tsp_path.txt）
    :return: 访问顺序列表（点索引，0-based），如果失败返回None
    """
    if not os.path.exists(tsp_path_file):
        print(f"  ✗ TSP路径文件不存在: {tsp_path_file}")
        return None
    
    print(f"  读取TSP访问顺序: {tsp_path_file}")
    
    try:
        with open(tsp_path_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        tour_order = []
        in_tour_section = False
        
        # 先尝试读取FULL_TOUR部分（更完整）
        for i, line in enumerate(lines):
            line_stripped = line.strip()
            
            # 如果找到FULL_TOUR部分，读取下一行
            if line_stripped == "FULL_TOUR:":
                # 读取下一行（包含完整路径）
                if i + 1 < len(lines):
                    next_line = lines[i + 1].strip()
                    if next_line:
                        # 解析格式: "0 -> 27 -> 51 -> ... -> 0"
                        parts = [p.strip() for p in next_line.split('->')]
                        tour_order = [int(p) for p in parts if p.isdigit()]
                        if tour_order:
                            # 如果最后返回起点，去掉最后一个重复的起点
                            if len(tour_order) > 1 and tour_order[-1] == tour_order[0]:
                                tour_order = tour_order[:-1]
                            print(f"  ✓ 从FULL_TOUR读取到 {len(tour_order)} 个点的访问顺序")
                            return tour_order
                break
        
        # 如果没有FULL_TOUR，尝试读取TOUR_ORDER部分
        for line in lines:
            line_stripped = line.strip()
            
            # 跳过注释和空行
            if not line_stripped or line_stripped.startswith('#'):
                continue
            
            # 检查是否进入TOUR_ORDER部分
            if line_stripped == "TOUR_ORDER:":
                in_tour_section = True
                continue
            
            # 如果在TOUR_ORDER部分，读取点索引
            if in_tour_section:
                # 格式可能是 "0 -> " 或 "0"
                parts = [p.strip() for p in line_stripped.split('->')]
                for part in parts:
                    if part.isdigit():
                        tour_order.append(int(part))
        
        if tour_order:
            print(f"  ✓ 从TOUR_ORDER读取到 {len(tour_order)} 个点的访问顺序")
            return tour_order
        else:
            print(f"  ✗ 未找到有效的访问顺序")
            return None
            
    except Exception as e:
        print(f"  ✗ 读取TSP访问顺序时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None

