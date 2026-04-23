##V0.5 - 集成巡检采样点生成（支持所有点对路径计算）
import os
import time
import numpy as np
from OSM_ENVIRONMENTS import OSM_ENV
from extract_map import extract_env_cartesian
from paths import DATA_DIR

from visualize import draw_main_maps
from grid_processing import GridGenerator, CostCalculator
from core_points_1 import compute_core_points
from AStar.grid_based_a_star import GridBasedAStar
from core_points_path_calculator import (
    merge_core_points,
    calculate_all_pairs_paths,
    save_all_pairs_results,
    load_paths_from_file,
    load_tsp_tour_from_file
)
from lkh_solver import solve_tsp_with_lkh
from shapely.geometry import Polygon, LineString, Point
from water_coverage_processor import (
    process_water_block_coverage,
    load_coverage_paths_from_files,
    COVERAGE_PATHS_SUBDIR,
)
from water_coverage_statistics import calculate_water_coverage_statistics
from water_entry_exit_calculator import compute_all_water_entry_exit
from combine_path import build_combined_path, save_combined_path


if __name__ == "__main__":
    # =============================================================================
    # 配置参数
    # =============================================================================
    SHOW_WATER_LABELS = False  # 是否显示水域和孔洞序号标记
    SHOW_WATER_BLOCK_NUMBERS = False  # 可视化中是否显示各水域块的骨架访问序号（小数字，第几块）
    PRINT_ENTRY_EXIT_DEBUG = False  # True: 打印骨架路径与水域交点及每个水域的起点、终点
    
    # 路径规划模式配置
    CALCULATE_ALL_PAIRS = False   # True: 计算所有点对并保存, False: 计算两个点进行可视化
    SOLVE_TSP_WITH_LKH = True  # True: 调用LKH求解器求解骨架路径问题TSP, False: 跳过TSP求解
    VISUALIZE_TSP_TOUR = True              # 是否可视化LKH求解的TSP访问顺序路径
    
    # =============================================================================
    # 可视化图生成开关
    # =============================================================================
    SHOW_GRID_SKELETON_MAP = False      # 是否显示栅格化-骨架路径图（显示栅格，不显示采样点）
    SHOW_COVERAGE_SKELETON_MAP = True # 是否显示覆盖路径-骨架路径图（不显示栅格，不显示采样点与覆盖路径，保持简洁）
    SHOW_COVERAGE_PATH_MAP = False      # 是否显示覆盖路径生成可视化地图（基于骨架图，叠加各水域采样点与覆盖路径；后续新增也在此图展示）
    
    # =============================================================================
    # 水域块内路径生成开关 + 块数限制
    # =============================================================================
    # 开关1：水域块内覆盖路径是否重算
    #   False = 关闭：不重新计算成本矩阵、不调用 LKH，直接从 data/coverage_paths/ 加载已有结果用于可视化
    #   True  = 开启：重新计算采样点、成本矩阵，调用 LKH，计算水域块内路径并写入 data/coverage_paths/
    RECOMPUTE_COVERAGE_PATHS = True
    MAX_WATER_BLOCKS_TO_COVER = 54    # 仅当 RECOMPUTE_COVERAGE_PATHS=True 时有效：为前 N 块生成覆盖路径（1=只第1块；设为很大或 None 表示全部,当前地图水域块有54个）
    
    # 采样点生成参数（水域块内覆盖路径用）：间距、X/Y 变形系数
    FILL_SPACING = 4.0        # 采样点基距（与地图坐标同单位，通常为米）
    X_SCALE_FACTOR = 0.85    # X 方向缩放（垂直于起点→终点）
    Y_SCALE_FACTOR = 1.0     # Y 方向缩放（沿起点→终点）
    # LKH 水域块内 TSP 求解参数：RUNS 越大解越优、耗时越长；越小求解越快、解可能略差
    LKH_RUNS = 1            # PAR 文件中的 RUNS（默认 2：速度优先；可改为 10 等提高质量）
    t_start = time.perf_counter()
    
    # 1. 加载环境对象
    print("1. 加载环境对象...")
    env = OSM_ENV(UAV=1)

    # 2. 提取边界、水域、采样点（使用 shrink_offset 和 spacing 控制）
    print("2. 提取地图数据...")
    env_data = extract_env_cartesian(env, shrink_dist=0.5, sample_dist=10)
    sampling_points = env_data["sampling_points"]
    water_with_hole = env_data["water_with_hole"]
    water_no_hole = env_data["water_no_hole"]
    boundary_polygon = env_data["boundary_polygon"]
    water_hierarchy = env_data["water_hierarchy"]  # 新增：层级关系数据
    
    # =============================================================================
    # 步骤2.5：生成巡检采样点（已注释 - 不再使用沿边巡检，改为覆盖巡检）
    # =============================================================================
    inspection_data = None
    
    # 3. 生成栅格地图
    print("\n3. 生成栅格地图...")
    grid_generator = GridGenerator(grid_size=4.0, boundary_padding=50.0) 

    grid_data = grid_generator.generate_grid(env_data)

    # 3.5. 计算核心点
    print("\n3.5. 计算水域核心点...")
    core_points_data = compute_core_points(env_data, grid_data)
    total_core_points = sum(
        len(points)
        for category in core_points_data.values()
        for points in category.values()
    )
    print(f"  ✓ 核心点生成完成！总核心点数: {total_core_points}")
    
    # 4. 初始化成本计算器
    print("\n4. 初始化成本计算器...")
    cost_calculator = CostCalculator(base_water_cost=1.0, land_cost=10.0, outside_boundary_cost=100.0)
    
    # 4.5. 使用A*算法计算核心点之间的路径
    path_result = None
    
    # 提取核心点
    no_hole_points = core_points_data.get('water_no_hole_core_points', {})
    with_hole_points = core_points_data.get('water_with_hole_core_points', {})
    
    if CALCULATE_ALL_PAIRS:
        # 模式1: 计算所有点对之间的路径并保存
        print("\n4.5. 计算所有核心点对之间的A*路径...")
        
        # 合并核心点
        merged_points = merge_core_points(no_hole_points, with_hole_points)
        
        if len(merged_points) < 2:
            print("  - 核心点数量不足（至少需要2个点）")
        else:
            # 计算所有点对
            cost_matrix, paths_dict = calculate_all_pairs_paths(
                merged_points, 
                grid_data, 
                cost_calculator,
                save_path='core_points_all_pairs_paths.txt'
            )
            
            if cost_matrix is not None:
                print(f"  ✓ 所有点对路径计算完成！")
                print(f"    成本矩阵大小: {cost_matrix.shape}")
                print(f"    成功计算的路径数: {len(paths_dict)}")
                
                # 4.6. TSP求解
                if SOLVE_TSP_WITH_LKH:
                    print("\n4.6. 调用LKH-3.exe求解TSP问题...")
                    data_dir_full = str(DATA_DIR)
                    tour_order = solve_tsp_with_lkh(
                        data_dir_full,
                        par_name="core_points.par",
                        solution_name="core_points_solution.txt",
                        output_name="core_points_tsp_path.txt"
                    )
                    
                    if tour_order is not None:
                        print(f"  ✓ TSP求解完成！")
                        print(f"    最优访问顺序包含 {len(tour_order)} 个点")
                        print(f"    访问顺序: {' -> '.join(map(str, tour_order[:10]))}" + 
                              (f" -> ... -> {tour_order[-1]}" if len(tour_order) > 10 else ""))
                    else:
                        print(f"  ✗ TSP求解失败")
                else:
                    print("\n4.6. TSP求解（已跳过）")
                    print("  提示: 设置 SOLVE_TSP_WITH_LKH = True 以启用TSP求解")
    
    # 4.6. TSP求解（独立于步骤4.5，可以使用之前生成的文件）
    if SOLVE_TSP_WITH_LKH:
        print("\n4.6. 调用LKH-3.exe求解TSP问题...")
        data_dir_full = str(DATA_DIR)
        tour_order = solve_tsp_with_lkh(
            data_dir_full,
            par_name="core_points.par",
            solution_name="core_points_solution.txt",
            output_name="core_points_tsp_path.txt"
        )
        
        if tour_order is not None:
            print(f"  ✓ TSP求解完成！")
            print(f"    最优访问顺序包含 {len(tour_order)} 个点")
            print(f"    访问顺序: {' -> '.join(map(str, tour_order[:10]))}" + 
                  (f" -> ... -> {tour_order[-1]}" if len(tour_order) > 10 else ""))
        else:
            print(f"  ✗ TSP求解失败")
    elif CALCULATE_ALL_PAIRS:
        # 只有在CALCULATE_ALL_PAIRS模式下才显示跳过提示
        print("\n4.6. TSP求解（已跳过）")
        print("  提示: 设置 SOLVE_TSP_WITH_LKH = True 以启用TSP求解")
    
    # 5. 读取TSP路径数据（如果启用可视化）
    tsp_tour_result = None
    if VISUALIZE_TSP_TOUR:
        print("\n5. 读取TSP路径数据用于可视化...")
        
        data_dir_full = str(DATA_DIR)
        
        # 读取TSP访问顺序
        tsp_path_file = os.path.join(data_dir_full, "core_points_tsp_path.txt")
        tour_order = load_tsp_tour_from_file(tsp_path_file)
        
        if tour_order:
            # 读取路径文件
            paths_file = os.path.join(data_dir_full, "core_points_path.txt")
            paths_dict = load_paths_from_file(paths_file)
            
            if paths_dict:
                # 合并核心点（用于获取点坐标）
                merged_points = merge_core_points(no_hole_points, with_hole_points)
                
                if len(merged_points) > 0:
                    tsp_tour_result = {
                        'tour_order': tour_order,
                        'paths_dict': paths_dict,
                        'merged_points': merged_points
                    }
                    print(f"  ✓ TSP路径数据准备完成")
                    print(f"    访问顺序: {len(tour_order)} 个点")
                    print(f"    可用路径: {len(paths_dict)} 条")
                else:
                    print("  ✗ 无法合并核心点")
            else:
                print("  ✗ 无法读取路径文件")
        else:
            print("  ✗ 无法读取TSP访问顺序")
            print("    提示: 请确保已运行TSP求解（设置 SOLVE_TSP_WITH_LKH = True）")
    
    # =============================================================================
    # 5. 所有水域块起点/终点（供可视化和覆盖路径生成）
    # =============================================================================
    coverage_result = None
    tour_order = (tsp_tour_result or {}).get("tour_order", [])
    merged_points = (tsp_tour_result or {}).get("merged_points", [])
    all_water_entry_exit = compute_all_water_entry_exit(
        tsp_tour_result,
        water_no_hole,
        water_with_hole,
        core_points_data=core_points_data,
        debug_print=PRINT_ENTRY_EXIT_DEBUG,
    )

    # =============================================================================
    # 5.4. 逐块覆盖路径：重算前 N 块并写文件，或从 data/coverage_paths 加载
    # =============================================================================
    all_blocks_coverage_paths = {}
    block1_entry = all_water_entry_exit.get(1, {})
    first_block_coverage_start = block1_entry.get("start_point")
    first_block_sampling_points = None
    first_block_coverage_tour = None
    first_block_exit_point = block1_entry.get("exit_point")

    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    coverage_paths_dir = os.path.join(data_dir, COVERAGE_PATHS_SUBDIR)

    if RECOMPUTE_COVERAGE_PATHS and tsp_tour_result and len(tour_order) >= 2:
        N = (min(MAX_WATER_BLOCKS_TO_COVER, len(tour_order)) if MAX_WATER_BLOCKS_TO_COVER is not None
             else len(tour_order))
        # 「起点/终点太近」阈值：以水域块50的起退点距离为准则；若无块50则用 1.0 m
        block50_entry = all_water_entry_exit.get(50, {})
        s50 = block50_entry.get("start_point")
        e50 = block50_entry.get("exit_point")
        if s50 is not None and e50 is not None:
            degenerate_threshold = float(np.sqrt((e50[0] - s50[0]) ** 2 + (e50[1] - s50[1]) ** 2))
        else:
            degenerate_threshold = 1.0
        print(f"\n5.4. 重新计算前 {N} 块覆盖路径，写入 {coverage_paths_dir} ...")
        print(f"  [阈值] 起点/终点视为同一点的距离阈值 = {degenerate_threshold:.4f} m（以块50起退点距离为准则）")
        for block_no in range(1, N + 1):
            i = block_no - 1
            point_idx = tour_order[i]
            if point_idx >= len(merged_points):
                continue
            point_info = merged_points[point_idx]
            water_idx = point_info.get("water_idx")
            water_type = point_info.get("type")
            water_info = None
            if water_type == "no_hole":
                for w in water_no_hole:
                    if w.get("idx") == water_idx:
                        water_info = w
                        break
            elif water_type == "with_hole":
                for w in water_with_hole:
                    if w.get("idx") == water_idx:
                        water_info = w
                        break
            if not water_info:
                continue
            water_poly = Polygon(water_info["outer"], holes=water_info.get("holes", []))
            # 聚类水域块：采样范围应为「聚类凸包∩水域」（红虚线区域），不是整片水域
            coverage_poly = water_poly
            if water_type == "with_hole" and core_points_data:
                clusters_map = core_points_data.get("water_with_hole_clusters", {})
                if water_idx in clusters_map:
                    current_pt = merged_points[point_idx].get("point")
                    if current_pt is not None:
                        for cluster in clusters_map[water_idx]:
                            hull_coords = cluster.get("hull")
                            if hull_coords and len(hull_coords) >= 3:
                                try:
                                    hull_poly = Polygon(hull_coords)
                                    if not hull_poly.is_empty and hull_poly.contains(Point(current_pt[0], current_pt[1])):
                                        inter = hull_poly.intersection(water_poly)
                                        if not inter.is_empty:
                                            coverage_poly = inter.geoms[0] if inter.geom_type == "MultiPolygon" and len(inter.geoms) > 0 else inter
                                            if coverage_poly.geom_type != "Polygon":
                                                coverage_poly = water_poly
                                        break
                                except Exception:
                                    continue
            entry_exit = all_water_entry_exit.get(block_no, {})
            start_pt = entry_exit.get("start_point")
            exit_pt = entry_exit.get("exit_point")
            if start_pt is None or exit_pt is None:
                print(f"  [跳过] 第{block_no}块：起点或终点缺失")
                continue
            _, sp, tour, _ = process_water_block_coverage(
                block_no=block_no,
                water_info=water_info,
                water_poly=coverage_poly,
                start_point=start_pt,
                exit_point=exit_pt,
                grid_data=grid_data,
                env_data=env_data,
                coverage_paths_dir=coverage_paths_dir,
                skip_sampling_and_path=False,
                fill_spacing=FILL_SPACING,
                x_scale_factor=X_SCALE_FACTOR,
                y_scale_factor=Y_SCALE_FACTOR,
                degenerate_threshold=degenerate_threshold,
                lkh_runs=LKH_RUNS,
            )
            # 只要有采样点就加入，便于可视化；LKH 失败时 tour 为 None，仍可画采样点
            if sp is not None:
                all_blocks_coverage_paths[block_no] = {"sampling_points": sp, "tour_order": tour}
        if 1 in all_blocks_coverage_paths:
            first_block_sampling_points = all_blocks_coverage_paths[1].get("sampling_points")
            first_block_coverage_tour = all_blocks_coverage_paths[1].get("tour_order")
    else:
        print(f"\n5.4. 不重算覆盖路径，从 {coverage_paths_dir} 加载已有结果...")
        all_blocks_coverage_paths = load_coverage_paths_from_files(coverage_paths_dir)
        if 1 in all_blocks_coverage_paths:
            first_block_sampling_points = all_blocks_coverage_paths[1].get("sampling_points")
            first_block_coverage_tour = all_blocks_coverage_paths[1].get("tour_order")
        print(f"  已加载 {len(all_blocks_coverage_paths)} 块覆盖路径")
    
    # =============================================================================
    # 5.5. 合并整体路径（覆盖路径 + 骨架出口→入口段）并保存点坐标序列
    # =============================================================================
    if all_blocks_coverage_paths and tsp_tour_result and all_water_entry_exit:
        full_path, combine_info = build_combined_path(
            all_blocks_coverage_paths,
            tsp_tour_result,
            all_water_entry_exit,
            max_blocks=MAX_WATER_BLOCKS_TO_COVER if RECOMPUTE_COVERAGE_PATHS else None,
        )
        if full_path:
            data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
            out_path = save_combined_path(full_path, data_dir, filename="combined_path.txt")
            print(f"\n5.5. 整体路径已合并并保存")
            print(f"  总点数: {len(full_path)}  输出: {out_path}")
        else:
            print(f"\n5.5. 未生成整体路径（{combine_info.get('error', '无有效块')}）")
    
    # =============================================================================
    # 6. 可视化栅格地图
    # =============================================================================
    print("\n6. 可视化栅格地图...")
    print("  注意：栅格成本数字显示功能已暂时注释")
    if SHOW_WATER_LABELS:
        print("  水域和孔洞序号标记功能：已启用")
    else:
        print("  水域和孔洞序号标记功能：已禁用")
    
    # =============================================================================
    # 6.1. 按开关生成栅格-骨架图、覆盖-骨架图、覆盖路径图
    # =============================================================================
    draw_main_maps(
        grid_data,
        env_data,
        cost_calculator=cost_calculator,
        tsp_tour_result=tsp_tour_result,
        core_points_data=core_points_data,
        path_result=path_result,
        all_water_entry_exit=all_water_entry_exit,
        first_block_coverage_start=first_block_coverage_start,
        first_block_sampling_points=first_block_sampling_points,
        first_block_coverage_tour=first_block_coverage_tour,
        first_block_exit_point=first_block_exit_point,
        all_blocks_coverage_paths=all_blocks_coverage_paths,
        show_grid_skeleton_map=SHOW_GRID_SKELETON_MAP,
        show_coverage_skeleton_map=SHOW_COVERAGE_SKELETON_MAP,
        show_coverage_path_map=SHOW_COVERAGE_PATH_MAP,
        show_water_labels=SHOW_WATER_LABELS,
        show_water_block_numbers=SHOW_WATER_BLOCK_NUMBERS,
    )

    # =============================================================================
    # 统计第一个水域块覆盖路径信息
    # =============================================================================
    if first_block_sampling_points is not None and first_block_coverage_tour is not None:
        calculate_water_coverage_statistics(
            first_block_sampling_points,
            first_block_coverage_tour,
            tsp_tour_result,
            water_no_hole,
            water_with_hole,
            coverage_width=4.0
        )
    
    # 输出总结
    print("\n" + "="*80)
    print("运行完成！")
    print("="*80)
    
    if total_core_points > 0:
        print(f"核心点: {total_core_points} 个")
        no_hole_count = sum(len(points) for points in core_points_data.get('water_no_hole_core_points', {}).values())
        with_hole_count = sum(len(points) for points in core_points_data.get('water_with_hole_core_points', {}).values())
        print(f"  无孔洞水域核心点: {no_hole_count} 个（五角星）")
        print(f"  有孔洞水域核心点: {with_hole_count} 个（菱形）")
    
    if tsp_tour_result:
        tour_order = tsp_tour_result.get('tour_order', [])
        paths_dict = tsp_tour_result.get('paths_dict', {})
        if tour_order:
            print(f"\nTSP求解结果:")
            print(f"  访问顺序: {len(tour_order)} 个点")
            print(f"  最优访问顺序: {' -> '.join(map(str, tour_order[:10]))}" + 
                  (f" -> ... -> {tour_order[-1]}" if len(tour_order) > 10 else ""))
            print(f"  可用路径数: {len(paths_dict)} 条")
    
    
    print(f"\n栅格地图: {grid_data['grid_type'].shape[0]} x {grid_data['grid_type'].shape[1]}")
    print(f"\n生成文件:")

    point_overlay_desc = []

    if total_core_points > 0:
        point_overlay_desc.append("核心点")
    if tsp_tour_result:
        point_overlay_desc.append("TSP骨架路径")
    overlay_suffix = " + ".join(point_overlay_desc)
    print(f"  - grid_map_visualization.png （栅格地图{(' + ' + overlay_suffix) if overlay_suffix else ''}）")
    print(f"  - map_structure.txt （水域层级结构）")
    if tsp_tour_result:
        print(f"  - core_points_tsp_path.txt （TSP访问顺序）")
        print(f"  - core_points_path.txt （核心点路径详情）")
    if SHOW_COVERAGE_SKELETON_MAP:
        print(f"  - coverage_skeleton_map.png （覆盖路径-骨架路径图）")
    if SHOW_COVERAGE_PATH_MAP:
        print(f"  - coverage_path_map.png （覆盖路径生成可视化地图）")
    elapsed = time.perf_counter() - t_start
    if elapsed >= 3600:
        h, r = divmod(elapsed, 3600)
        m, s = divmod(r, 60)
        duration_str = f"{int(h)} 小时 {int(m)} 分 {s:.1f} 秒"
    elif elapsed >= 60:
        m, s = divmod(elapsed, 60)
        duration_str = f"{int(m)} 分 {s:.1f} 秒"
    else:
        duration_str = f"{elapsed:.1f} 秒"
    print(f"\n本次运行时长: {duration_str}")
    print("="*80)
    

