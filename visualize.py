##V0.6 - Multi-stage visualization support
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, PathPatch, Rectangle
from matplotlib.path import Path
from matplotlib.markers import MarkerStyle
import matplotlib.font_manager as fm
from shapely.geometry import Polygon as ShapelyPolygon
from typing import Optional

# Set font support
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def visualize_grid_map(
    grid_data,
    env_data,
    cost_calculator=None,
    save_path=None,
    show_title=True,
    show_labels=True,
    inspection_points=None,
    show_point_numbers=False,
    core_points=None,
    path_result=None,
    tsp_tour_result=None,
    coverage_result=None,
    first_block_coverage_start=None,
    first_block_sampling_points=None,
    first_block_coverage_tour=None,
    first_block_exit_point=None,
    all_blocks_coverage_paths=None,
    second_water_start=None,
    second_water_exit=None,
    second_water_idx=None,
    all_clustered_water_entry_exit_points=None,
    all_single_water_entry_exit_points=None,
    show_grid=True,
    show_sampling_points=True,
    show_water_block_numbers=False,
):
    """
    Visualize grid map
    :param grid_data: Grid data
    :param env_data: Environment data
    :param cost_calculator: Cost calculator (optional)
    :param save_path: Save path (optional)
    :param show_title: Whether to show title
    :param show_labels: Whether to show water body and hole labels
    :param inspection_points: Inspection sampling points to overlay (optional)
    :param show_point_numbers: Whether to show point numbers on inspection points
    :param core_points: Core points data (optional)
    :param path_result: Single A* path result (optional)
    :param tsp_tour_result: TSP tour result dictionary containing tour_order, paths_dict, merged_points (optional)
    :param coverage_result: Coverage paths result dictionary (optional)
    """
    # 创建新图形，避免与之前的图冲突
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Draw grid and get actual drawn statistics (if enabled)
    drawn_stats = None
    if show_grid:
        drawn_stats = _draw_grid_cells(ax, grid_data)
    else:
        # 覆盖路径图：不显示栅格，仅将范围外区域填成红色禁飞区（纯色、无栅格线）
        _draw_no_fly_zone(ax, grid_data)
    
    # Draw grid cost numbers (temporarily commented)
    # if cost_calculator is not None:
    #     _draw_grid_costs(ax, grid_data, cost_calculator)
    
    # Draw original water body boundaries
    _draw_water_bodies(ax, env_data, show_labels)
    
    # Draw boundary
    _draw_boundary(ax, env_data)
    
    # Draw inspection points if provided
    if inspection_points is not None and len(inspection_points) > 0:
        _draw_inspection_points(ax, inspection_points, show_point_numbers)
    
    # Draw core points if provided
    if core_points:
        _draw_core_points(ax, core_points)
        _draw_cluster_hulls(ax, env_data, core_points)
    
    # Draw A* path if provided (single path) - 已禁用，避免与TSP路径混淆
    # if path_result and path_result.get('success'):
    #     _draw_a_star_path(ax, path_result)
    
    # Draw TSP tour if provided
    if tsp_tour_result:
        _draw_tsp_tour(ax, tsp_tour_result)
    
    # Draw first water coverage start point if provided
    if first_block_coverage_start is not None:
        _draw_block1_coverage_start(ax, first_block_coverage_start)
    
    # Draw first block coverage exit point if provided
    if first_block_exit_point is not None:
        _draw_block1_coverage_exit(ax, first_block_exit_point)
    
    # Draw sampling points: 若传入所有块覆盖路径则绘制各块采样点，否则仅绘制第一块（若提供）
    if show_sampling_points:
        if all_blocks_coverage_paths and len(all_blocks_coverage_paths) > 0:
            _draw_all_blocks_sampling_points(ax, all_blocks_coverage_paths)
        elif first_block_sampling_points is not None:
            _draw_block1_sampling_points(ax, first_block_sampling_points)
    
    # Draw coverage path(s): 若传入所有块则绘制各块路径，否则仅绘制第一块
    if all_blocks_coverage_paths and len(all_blocks_coverage_paths) > 0:
        _draw_all_blocks_coverage_paths(ax, all_blocks_coverage_paths)
    elif first_block_coverage_tour is not None and first_block_sampling_points is not None:
        _draw_block1_coverage_path(ax, first_block_sampling_points, first_block_coverage_tour)
    
    # 所有非第一块水域的起点/终点统一由此绘制（含手动矫正后的值），避免第二块被画两遍
    _all_water_entry_exit = {**(all_single_water_entry_exit_points or {}), **(all_clustered_water_entry_exit_points or {})}
    if _all_water_entry_exit:
        _draw_water_entry_exit_points(ax, _all_water_entry_exit)
    # 仅当没有合并字典时才单独画第二块（向后兼容）
    elif second_water_start is not None or second_water_exit is not None:
        _draw_second_water_entry_exit_points(ax, second_water_start, second_water_exit)
    # 是否在水域块旁标出骨架路径访问序号（小数字）
    if show_water_block_numbers:
        _draw_water_block_number_labels(ax, _all_water_entry_exit, first_block_coverage_start)
    
    # Draw second water body label if provided (已注释 - 暂时不使用)
    # if second_water_idx is not None:
    #     _draw_second_water_label(ax, env_data, second_water_idx)
    
    # =============================================================================
    # 绘制覆盖路径（已注释 - 暂时不使用）
    # =============================================================================
    # if coverage_result:
    #     _draw_coverage_paths(ax, coverage_result)

    # Set axis limits to show only inside the boundary
    boundary_polygon = env_data["boundary_polygon"]
    x_min, x_max = boundary_polygon[:, 0].min(), boundary_polygon[:, 0].max()
    y_min, y_max = boundary_polygon[:, 1].min(), boundary_polygon[:, 1].max()
    margin = 10  # 边距（米）
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    
    # Set figure properties
    ax.set_aspect('equal', adjustable='box')
    if show_title:
        title_text = show_title if isinstance(show_title, str) else "Grid Map Visualization"
        # 使用Times New Roman字体（英文标题）
        ax.set_title(title_text, fontsize=16, fontweight='bold', 
                    pad=10, family='Times New Roman')
    ax.set_xlabel("X (m)", fontsize=12, family='Times New Roman', fontweight='bold')
    ax.set_ylabel("Y (m)", fontsize=12, family='Times New Roman', fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Set axis tick font size
    ax.tick_params(axis='both', which='major', labelsize=12)
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontfamily('Times New Roman')
        label.set_fontweight('bold')
    
    # Add legend
    _add_grid_legend(ax, coverage_result=None)  # 已注释：暂时不使用覆盖路径
    
    # Add statistics based on actually drawn grids (only if grid is shown)
    if drawn_stats is not None:
        _add_grid_statistics(ax, drawn_stats)
    
    # Adjust layout to leave space for top elements
    plt.subplots_adjust(top=0.85)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Grid map saved to: {save_path}")
    
    # 不自动显示，由调用者控制
    # plt.show()  # 注释掉，避免阻塞，让两张图都能生成


def draw_main_maps(
    grid_data,
    env_data,
    cost_calculator=None,
    tsp_tour_result=None,
    core_points_data=None,
    path_result=None,
    all_water_entry_exit=None,
    first_block_coverage_start=None,
    first_block_sampling_points=None,
    first_block_coverage_tour=None,
    first_block_exit_point=None,
    all_blocks_coverage_paths=None,
    show_grid_skeleton_map=False,
    show_coverage_skeleton_map=False,
    show_coverage_path_map=False,
    show_water_labels=False,
    show_water_block_numbers=False,
):
    """
    按开关生成栅格-骨架图、覆盖-骨架图、覆盖路径图，并在有图时 plt.show()。
    供 main 调用，保持 main 简洁。
    """
    inspection_points = None
    any_shown = False
    if show_grid_skeleton_map:
        print("\n  生成栅格化-骨架路径图...")
        visualize_grid_map(
            grid_data, env_data, cost_calculator,
            save_path="grid_skeleton_map.png",
            show_title="Grid-Skeleton Path Map",
            show_labels=show_water_labels,
            inspection_points=inspection_points,
            show_point_numbers=False,
            core_points=core_points_data,
            path_result=path_result,
            tsp_tour_result=tsp_tour_result,
            coverage_result=None,
            first_block_coverage_start=first_block_coverage_start,
            first_block_sampling_points=None,
            first_block_coverage_tour=None,
            first_block_exit_point=first_block_exit_point,
            all_blocks_coverage_paths=None,
            second_water_start=None,
            second_water_exit=None,
            second_water_idx=None,
            all_single_water_entry_exit_points=all_water_entry_exit,
            all_clustered_water_entry_exit_points={},
            show_grid=True,
            show_sampling_points=False,
            show_water_block_numbers=show_water_block_numbers,
        )
        print("  ✓ 栅格化-骨架路径图已保存: grid_skeleton_map.png")
        any_shown = True
    if show_coverage_skeleton_map:
        print("\n  生成覆盖路径-骨架路径图...")
        visualize_grid_map(
            grid_data, env_data, cost_calculator,
            save_path="coverage_skeleton_map.png",
            show_title="Coverage-Skeleton Path Map",
            show_labels=show_water_labels,
            inspection_points=inspection_points,
            show_point_numbers=False,
            core_points=core_points_data,
            path_result=path_result,
            tsp_tour_result=tsp_tour_result,
            coverage_result=None,
            first_block_coverage_start=None,
            first_block_sampling_points=None,
            first_block_coverage_tour=None,
            first_block_exit_point=None,
            all_blocks_coverage_paths=None,
            second_water_start=None,
            second_water_exit=None,
            second_water_idx=None,
            all_single_water_entry_exit_points=all_water_entry_exit,
            all_clustered_water_entry_exit_points={},
            show_grid=False,
            show_sampling_points=False,
            show_water_block_numbers=show_water_block_numbers,
        )
        print("  ✓ 覆盖路径-骨架路径图已保存: coverage_skeleton_map.png")
        any_shown = True
    if show_coverage_path_map:
        print("\n  生成覆盖路径生成可视化地图...")
        visualize_grid_map(
            grid_data, env_data, cost_calculator,
            save_path="coverage_path_map.png",
            show_title="Coverage Path Generation Map",
            show_labels=show_water_labels,
            inspection_points=inspection_points,
            show_point_numbers=False,
            core_points=core_points_data,
            path_result=path_result,
            tsp_tour_result=tsp_tour_result,
            coverage_result=None,
            first_block_coverage_start=first_block_coverage_start,
            first_block_sampling_points=first_block_sampling_points,
            first_block_coverage_tour=first_block_coverage_tour,
            first_block_exit_point=first_block_exit_point,
            all_blocks_coverage_paths=all_blocks_coverage_paths,
            second_water_start=None,
            second_water_exit=None,
            second_water_idx=None,
            all_single_water_entry_exit_points=all_water_entry_exit,
            all_clustered_water_entry_exit_points={},
            show_grid=False,
            show_sampling_points=True,
            show_water_block_numbers=show_water_block_numbers,
        )
        print("  ✓ 覆盖路径生成可视化地图已保存: coverage_path_map.png")
        any_shown = True
    if any_shown:
        plt.show()


def _draw_grid_cells(ax, grid_data):
    """Draw grid cells and return actual drawn grid statistics"""
    coordinates = grid_data['coordinates']
    grid_type = grid_data['grid_type']
    grid_size = grid_data['grid_size']
    
    rows, cols = coordinates.shape[:2]
    
    # Define color mapping
    colors = {
        -1: '#FF0000',  # Red: outside boundary
        0: '#FFFFFF',   # White: land
        1: '#87CEEB',   # Blue: water
    }
    
    # Statistics for actually drawn grids
    drawn_stats = {
        'water_grids': 0,
        'land_grids': 0,
        'boundary_grids': 0
    }
    
    # Draw each grid
    for i in range(rows):
        for j in range(cols):
            # Calculate grid bottom-left corner coordinates
            x = coordinates[i, j, 0] - grid_size / 2
            y = coordinates[i, j, 1] - grid_size / 2
            
            # Create rectangle
            rect = Rectangle(
                (x, y), grid_size, grid_size,
                facecolor=colors[grid_type[i, j]],
                edgecolor='black',
                linewidth=0.5,
                alpha=0.7
            )
            ax.add_patch(rect)
            
            # Count the drawn grids by color
            if grid_type[i, j] == 1:  # Blue: water
                drawn_stats['water_grids'] += 1
            elif grid_type[i, j] == 0:  # White: land
                drawn_stats['land_grids'] += 1
            elif grid_type[i, j] == -1:  # Red: outside boundary
                drawn_stats['boundary_grids'] += 1
    
    return drawn_stats


def _draw_no_fly_zone(ax, grid_data):
    """仅将范围外栅格填成红色禁飞区（纯色、无栅格线），用于 show_grid=False 的覆盖路径-骨架路径图。
    使用 grid_type==-1 的栅格（与 GridGenerator 判定一致），边缘设为 none 避免出现栅格线。"""
    coordinates = grid_data["coordinates"]
    grid_type = grid_data["grid_type"]
    grid_size = grid_data["grid_size"]
    rows, cols = coordinates.shape[:2]
    for i in range(rows):
        for j in range(cols):
            if grid_type[i, j] != -1:  # -1: 边界外
                continue
            x = coordinates[i, j, 0] - grid_size / 2
            y = coordinates[i, j, 1] - grid_size / 2
            rect = Rectangle(
                (x, y), grid_size, grid_size,
                facecolor="#FF0000",
                edgecolor="none",  # 无栅格线，纯红
                alpha=0.7,
            )
            ax.add_patch(rect)


def _draw_grid_costs(ax, grid_data, cost_calculator, max_grids_to_show=100):
    """Draw grid cost numbers"""
    coordinates = grid_data['coordinates']
    grid_type = grid_data['grid_type']
    grid_size = grid_data['grid_size']
    
    rows, cols = coordinates.shape[:2]
    total_grids = rows * cols
    
    # Calculate font size (based on grid size)
    font_size = max(6, min(12, grid_size * 0.3))
    
    # If too many grids, only show part
    if total_grids > max_grids_to_show:
        step = max(1, total_grids // max_grids_to_show)
    else:
        step = 1
    
    # Iterate through each grid
    for i in range(0, rows, step):
        for j in range(0, cols, step):
            # Skip grids outside boundary
            if grid_type[i, j] == -1:
                continue
            
            # Calculate grid center coordinates
            center_x = coordinates[i, j, 0]
            center_y = coordinates[i, j, 1]
            
            # Calculate cost
            cost = cost_calculator.calculate_grid_cost(grid_data, i, j)
            
            # If cost is infinite, show "∞"
            if cost == float('inf'):
                cost_text = "∞"
            else:
                # Format cost number (keep 1 decimal place)
                cost_text = f"{cost:.1f}"
            
            # Add cost text
            ax.text(center_x, center_y, cost_text, 
                   ha='center', va='center',
                   fontsize=font_size,
                   fontfamily='Times New Roman',
                   color='black',
                   fontweight='bold',
                   bbox=dict(boxstyle='round,pad=0.1', 
                           facecolor='white', 
                           edgecolor='none', 
                           alpha=0.8))

def _draw_water_bodies(ax, env_data, show_labels=True):
    """Draw original water body boundaries, considering hierarchy relationships"""
    water_with_hole = env_data["water_with_hole"]
    water_no_hole = env_data["water_no_hole"]
    water_hierarchy = env_data.get("water_hierarchy", {})  # Get hierarchy data
    
    # Collect all water body IDs inside holes
    contained_water_ids = set()
    for water_idx, water_info in water_hierarchy.items():
        for hole in water_info['holes']:
            if hole['type'] == 'container':
                contained_water_ids.update(hole['contained_waters'])
    
    
    def draw_single_water(w, alpha=0.3, fill_holes=True):
        """
        Draw single water body
        :param w: Water body data
        :param alpha: Transparency
        :param fill_holes: Whether to fill holes (True=holes are white, False=holes are transparent)
        """
        from matplotlib.path import Path
        from matplotlib.patches import PathPatch
        
        water_idx = w.get("idx", 0)
        
        # Check if this water body has holes and needs to be hollowed out
        if len(w["holes"]) > 0 and not fill_holes:
            # Use Path to create polygon with holes (hole parts are transparent)
            vertices = []
            codes = []
            
            # Outer contour
            outer = w["outer"]
            vertices.extend(outer)
            codes.append(Path.MOVETO)
            codes.extend([Path.LINETO] * (len(outer) - 1))
            codes.append(Path.CLOSEPOLY)
            vertices.append(outer[0])  # Close
            
            # Holes (as inner contours, will be hollowed out)
            for hole in w["holes"]:
                vertices.extend(hole)
                codes.append(Path.MOVETO)
                codes.extend([Path.LINETO] * (len(hole) - 1))
                codes.append(Path.CLOSEPOLY)
                vertices.append(hole[0])  # Close
            
            # Create Path and Patch
            path = Path(vertices, codes)
            patch = PathPatch(path, facecolor='#87CEEB', edgecolor='darkblue', 
                            alpha=alpha, linewidth=1, zorder=20)  # 线宽缩小为原来的一半，显示在顶层
            ax.add_patch(patch)
            
            # Add legend
            if 'Water Body' not in ax.get_legend_handles_labels()[1]:
                ax.plot([], [], color='#87CEEB', linewidth=10, label='Water Body')
        else:
            # No holes or want to fill holes, draw outer contour directly
            ax.fill(*w["outer"].T, facecolor='#87CEEB', 
                   edgecolor='darkblue', alpha=alpha, linewidth=1, zorder=20,  # 线宽缩小为原来的一半，显示在顶层
                   label='Water Body' if 'Water Body' not in ax.get_legend_handles_labels()[1] else '')
        
        # Add polygon number label
        if show_labels:
            polygon_id = w.get("idx", "?")
            center_x, center_y = w["outer"].mean(axis=0)
            
            # 个别水域标签偏移，避免遮挡
            offset_x, offset_y = 0, 0
            if polygon_id == 20:
                offset_y = 8    # P20 向上
            elif polygon_id == 15:
                offset_y = -8   # P15 向下
            elif polygon_id == 10:
                offset_x = 8    # P10 向右
            
            ax.text(center_x + offset_x, center_y + offset_y, f"P{polygon_id}", 
                   ha='center', va='center', fontsize=8, fontweight='bold',
                   color='darkblue', bbox=dict(boxstyle='round,pad=0.3', 
                   facecolor='white', edgecolor='darkblue', alpha=0.8))
    
    # ========================================
    # Stage 1: Draw all water bodies not inside holes
    # ========================================
    for w in water_with_hole + water_no_hole:
        water_idx = w.get("idx", 0)
        if water_idx not in contained_water_ids:
            # fill_holes=False means hole parts are not filled (remain transparent)
            draw_single_water(w, fill_holes=False)
    
    # ========================================
    # Stage 2: Draw all holes (real holes and container boundaries)
    # ========================================
    for w in water_with_hole:
        water_idx = w.get("idx", 0)
        if water_idx in water_hierarchy:
            holes_info = water_hierarchy[water_idx]['holes']
            
            for hole_info in holes_info:
                hole_coords = hole_info['hole_coords']
                
                if hole_info['type'] == 'real_hole':
                    # Real hole, draw white fill
                    ax.fill(*np.array(hole_coords).T, facecolor='white', 
                           edgecolor='darkblue', alpha=1.0, linewidth=0.5, zorder=20,  # 线宽缩小为原来的一半，显示在顶层
                           label='Land' if 'Land' not in ax.get_legend_handles_labels()[1] else '')
                    
                    # Add hole number label
                    if show_labels:
                        hole_center_x, hole_center_y = np.array(hole_coords).mean(axis=0)
                        ax.text(hole_center_x, hole_center_y, f"P{water_idx}-{hole_info['hole_idx']+1}", 
                               ha='center', va='center', fontsize=6, fontweight='bold',
                               color='red', bbox=dict(boxstyle='round,pad=0.2', 
                               facecolor='yellow', edgecolor='red', alpha=0.8))
                
                elif hole_info['type'] == 'container':
                    # Container hole, draw dashed boundary (不添加 container 标签)
                    ax.plot(*np.array(hole_coords).T, color='darkblue', 
                           linewidth=0.5, alpha=0.8, linestyle='--', zorder=20)
        else:
            # No hierarchy info, use original logic
            for hole_idx, h in enumerate(w["holes"]):
                ax.fill(*h.T, facecolor='white', 
                       edgecolor='darkblue', alpha=1.0, linewidth=0.5, zorder=20,  # 线宽缩小为原来的一半，显示在顶层
                       label='Land' if 'Land' not in ax.get_legend_handles_labels()[1] else '')
                
                if show_labels:
                    polygon_id = w.get("idx", "?")
                    hole_center_x, hole_center_y = h.mean(axis=0)
                    ax.text(hole_center_x, hole_center_y, f"P{polygon_id}-{hole_idx+1}", 
                           ha='center', va='center', fontsize=6, fontweight='bold',
                           color='red', bbox=dict(boxstyle='round,pad=0.2', 
                           facecolor='yellow', edgecolor='red', alpha=0.8))
    
    # ========================================
    # Stage 3: Draw water bodies inside holes
    # ========================================
    for w in water_with_hole + water_no_hole:
        water_idx = w.get("idx", 0)
        if water_idx in contained_water_ids:
            # Water bodies inside holes usually have no holes themselves, but set fill_holes=False for consistency
            draw_single_water(w, fill_holes=False)

def _draw_boundary(ax, env_data):
    """Draw boundary"""
    boundary_polygon = env_data["boundary_polygon"]
    # 闭合边界：将第一个点追加到末尾
    closed_boundary = np.vstack([boundary_polygon, boundary_polygon[0:1, :]])
    ax.plot(closed_boundary[:, 0], closed_boundary[:, 1], 
           'k-', linewidth=3, label='Boundary')

def _draw_inspection_points(ax, inspection_points, show_numbers=False):
    """
    Draw inspection sampling points on the grid map
    :param ax: Matplotlib axis
    :param inspection_points: List of inspection points [[x, y], ...]
    :param show_numbers: Whether to show point numbers
    """
    points_array = np.array(inspection_points)
    
    # Draw inspection points as red circles (smaller size: 15 instead of 30)
    ax.scatter(points_array[:, 0], points_array[:, 1], 
              c='red', s=15, marker='o', alpha=0.8, 
              edgecolors='darkred', linewidths=0.5,
              label=f'Inspection Points ({len(inspection_points)})',
              zorder=10)  # Higher zorder to draw on top
    
    # Optionally show point numbers
    if show_numbers and len(inspection_points) <= 200:  # Only show numbers if not too many
        for idx, point in enumerate(inspection_points):
            ax.text(point[0], point[1], str(idx), 
                   ha='center', va='center', fontsize=6, 
                   color='white', fontweight='bold',
                   bbox=dict(boxstyle='circle,pad=0.1', 
                            facecolor='red', edgecolor='darkred', alpha=0.8),
                   zorder=11)


def _draw_core_points(ax, core_points):
    """Draw water body core points."""

    with_hole = core_points.get('water_with_hole_core_points', {})
    no_hole = core_points.get('water_no_hole_core_points', {})

    if no_hole:
        no_hole_arrays = [np.asarray(points, dtype=float) for points in no_hole.values() if len(points) > 0]
        if no_hole_arrays:
            no_hole_points = np.concatenate(no_hole_arrays, axis=0)
            ax.scatter(
                no_hole_points[:, 0],
                no_hole_points[:, 1],
                c='gold',
                marker='*',
                s=80,
                edgecolors='darkorange',
                linewidths=0.8,
                alpha=0.9,
                label='Core Points (No Holes)',
                zorder=12,
            )

    if with_hole:
        with_hole_arrays = [np.asarray(points, dtype=float) for points in with_hole.values() if len(points) > 0]
        if with_hole_arrays:
            with_hole_points = np.concatenate(with_hole_arrays, axis=0)
            ax.scatter(
                with_hole_points[:, 0],
                with_hole_points[:, 1],
                c='purple',
                marker='D',
                s=25,
                edgecolors='mediumpurple',
                linewidths=0.5,
                alpha=1.0,
                label='Core Points (With Holes)',
                zorder=12,
            )


def _draw_cluster_hulls(ax, env_data, core_points, color="#FF0000"):
    """Draw cluster boundaries as intersection of cluster hull and water polygon."""
    if not core_points:
        return

    clusters_map = core_points.get('water_with_hole_clusters', {})
    if not clusters_map:
        return

    water_lookup = {water.get("idx"): water for water in env_data.get("water_with_hole", [])}
    label_added = False

    for idx, clusters in clusters_map.items():
        water = water_lookup.get(idx)
        if not water:
            continue

        # 创建水域多边形（包括外边界和孔洞）
        try:
            water_poly = ShapelyPolygon(water["outer"], holes=water.get("holes", []))
        except Exception:
            continue
        
        if water_poly.is_empty or water_poly.area <= 1e-6:
            continue

        for cluster in clusters:
            hull_coords = cluster.get("hull")
            if not hull_coords or len(hull_coords) < 3:
                continue

            try:
                hull_poly = ShapelyPolygon(hull_coords)
            except Exception:
                continue
                
            if hull_poly.is_empty or hull_poly.area <= 1e-6:
                continue

            # 计算聚类凸包与水域的交集
            try:
                intersection = hull_poly.intersection(water_poly)
            except Exception:
                continue
            
            # 处理不同的几何类型
            if intersection.is_empty:
                continue
            
            # 提取交集的边界坐标
            boundary_coords = None
            if intersection.geom_type == 'Polygon':
                # 单个多边形：提取外边界
                exterior_coords = list(intersection.exterior.coords)
                if len(exterior_coords) >= 3:
                    boundary_coords = exterior_coords[:-1]  # 去掉最后一个重复点
            elif intersection.geom_type == 'MultiPolygon':
                # 多个多边形：绘制每个多边形的边界
                for poly in intersection.geoms:
                    if poly.geom_type == 'Polygon':
                        exterior_coords = list(poly.exterior.coords)
                        if len(exterior_coords) >= 3:
                            boundary_coords = exterior_coords[:-1]
                            # 绘制这个多边形的边界
                            boundary_array = np.asarray(boundary_coords, dtype=float)
                            boundary_array = np.vstack([boundary_array, boundary_array[0]])
                            ax.plot(
                                boundary_array[:, 0],
                                boundary_array[:, 1],
                                linestyle="--",
                                color=color,
                                linewidth=1.5,
                                label="Cluster Boundary" if not label_added else "",
                                zorder=11,
                            )
                            label_added = True
                continue  # MultiPolygon 已经处理完，跳过后续单个边界绘制
            
            # 绘制单个多边形的边界
            if boundary_coords and len(boundary_coords) >= 3:
                boundary_array = np.asarray(boundary_coords, dtype=float)
                boundary_array = np.vstack([boundary_array, boundary_array[0]])
                ax.plot(
                    boundary_array[:, 0],
                    boundary_array[:, 1],
                    linestyle="--",
                    color=color,
                    linewidth=1.5,
                    label="Cluster Boundary" if not label_added else "",
                    zorder=11,
                )
                label_added = True


def _is_same_boundary(hull_poly: ShapelyPolygon, outer_poly: Optional[ShapelyPolygon], hole_polys, tolerance: float = 1.0) -> bool:
    """Check whether the hull polygon coincides with existing water boundaries."""
    if outer_poly and outer_poly.symmetric_difference(hull_poly).area <= tolerance:
        return True

    for hole_poly in hole_polys:
        if hole_poly.symmetric_difference(hull_poly).area <= tolerance:
            return True

    return False

def _draw_a_star_path(ax, path_result):
    """
    绘制A*算法计算出的路径
    
    :param ax: Matplotlib axis
    :param path_result: A*路径结果字典
    """
    if not path_result.get('success') or not path_result.get('path'):
        return
    
    path = path_result['path']
    path_array = np.array(path)
    
    # 绘制路径线
    ax.plot(
        path_array[:, 0],
        path_array[:, 1],
        color='lime',
        linewidth=2.5,
        linestyle='-',
        alpha=0.8,
        label='A* Path',
        zorder=13
    )
    
    # 绘制起点和终点标记
    start = path_result.get('start', path[0])
    goal = path_result.get('goal', path[-1])
    
    # 起点：绿色圆圈
    ax.scatter(
        start[0], start[1],
        c='green',
        marker='o',
        s=100,
        edgecolors='darkgreen',
        linewidths=2,
        alpha=0.9,
        label='Start',
        zorder=14
    )
    
    # 终点：红色方块
    ax.scatter(
        goal[0], goal[1],
        c='red',
        marker='s',
        s=100,
        edgecolors='darkred',
        linewidths=2,
        alpha=0.9,
        label='Goal',
        zorder=14
    )
    
    # 显示路径信息文本
    cost = path_result.get('cost', 0)
    distance = path_result.get('distance', 0)
    info_text = f"Cost: {cost:.2f}\nDistance: {distance:.2f}m"
    
    # 在路径中点位置显示信息
    if len(path) > 0:
        mid_idx = len(path) // 2
        mid_point = path[mid_idx]
        ax.text(
            mid_point[0], mid_point[1],
            info_text,
            ha='center',
            va='center',
            fontsize=9,
            fontweight='bold',
            color='black',
            bbox=dict(
                boxstyle='round,pad=0.5',
                facecolor='yellow',
                edgecolor='black',
                alpha=0.8
            ),
            zorder=15
        )


def _draw_tsp_tour(ax, tsp_tour_result):
    """
    绘制TSP访问顺序路径
    
    :param ax: Matplotlib axis
    :param tsp_tour_result: TSP路径结果字典，包含:
        - tour_order: 访问顺序列表（点索引）
        - paths_dict: 路径字典 {f"{i}->{j}": {'cost': float, 'distance': float, 'path': list}}
        - merged_points: 合并后的核心点列表
    """
    import random
    
    tour_order = tsp_tour_result.get('tour_order')
    paths_dict = tsp_tour_result.get('paths_dict')
    merged_points = tsp_tour_result.get('merged_points')
    
    if not tour_order or not paths_dict or not merged_points:
        print("  ⚠ TSP路径数据不完整，跳过可视化")
        return
    
    if len(tour_order) < 2:
        print("  ⚠ TSP访问顺序点数不足，跳过可视化")
        return
    
    print(f"  绘制TSP路径，包含 {len(tour_order)} 个点")
    
    # 计算总成本和总距离
    total_cost = 0.0
    total_distance = 0.0
    segment_count = 0
    valid_segments = []  # 存储有效的路径段信息，用于随机选择显示
    
    # 收集所有有效的路径段
    for seg_idx in range(len(tour_order)):
        from_idx = tour_order[seg_idx]
        to_idx = tour_order[(seg_idx + 1) % len(tour_order)]  # 最后一段返回起点
        
        # 获取路径键（确保顺序正确）
        path_key = f"{from_idx}->{to_idx}"
        if path_key not in paths_dict:
            # 尝试反向
            path_key = f"{to_idx}->{from_idx}"
            if path_key not in paths_dict:
                print(f"    ⚠ 警告: 未找到路径 {from_idx}->{to_idx}，跳过该段")
                continue
        
        path_info = paths_dict[path_key]
        path_points = path_info.get('path', [])
        cost = path_info.get('cost', 0)
        distance = path_info.get('distance', 0)
        
        if not path_points:
            print(f"    ⚠ 警告: 路径 {path_key} 没有路径点")
            continue
        
        # 保存有效段信息
        valid_segments.append({
            'seg_idx': seg_idx,
            'path_points': path_points,
            'cost': cost,
            'distance': distance,
            'from_idx': from_idx,
            'to_idx': to_idx
        })
        
        total_cost += cost
        total_distance += distance
        segment_count += 1
    
    # 随机选择5段显示标注（如果总段数大于5）
    segments_to_label = []
    if len(valid_segments) > 5:
        segments_to_label = random.sample(valid_segments, 5)
    else:
        segments_to_label = valid_segments
    
    # 绘制所有路径段（黄色）
    for seg_info in valid_segments:
        path_points = seg_info['path_points']
        path_array = np.array(path_points)
        
        # 绘制路径线段（深黄色，更不透明）
        ax.plot(
            path_array[:, 0],
            path_array[:, 1],
            color='#FFA500',  # 深橙色/深黄色
            linewidth=2.5,
            linestyle='-',
            alpha=0.95,  # 提高不透明度
            label='TSP Tour' if seg_info['seg_idx'] == 0 else '',
            zorder=13
        )
        # 绘制路径方向箭头（红色），从起点核心点指向终点核心点
        from_idx = seg_info['from_idx']
        to_idx = seg_info['to_idx']
        if from_idx < len(merged_points) and to_idx < len(merged_points):
            # 获取起点和终点的核心点坐标（几何中心）
            start_center = merged_points[from_idx]['point']
            end_center = merged_points[to_idx]['point']
            
            # 计算箭头位置（在路径中点附近，但方向从起点指向终点）
            # 使用路径中点作为箭头位置，但方向基于核心点
            if len(path_points) > 0:
                mid_idx = len(path_points) // 2
                arrow_pos = path_points[mid_idx]
                
                # 计算从起点核心点到终点核心点的方向向量
                dx = end_center[0] - start_center[0]
                dy = end_center[1] - start_center[1]
                length = np.sqrt(dx**2 + dy**2)
                
                if length > 0:
                    # 归一化方向向量
                    dx_norm = dx / length
                    dy_norm = dy / length
                    
                    # 计算路径的实际长度（用于确定箭头大小）
                    path_length = seg_info.get('distance', length)
                    
                    # 箭头长度（基于路径实际长度，但限制在合理范围内）
                    # 使用路径长度的8-15%，但最小8米，最大25米
                    arrow_length = min(max(path_length * 0.12, 8.0), 25.0)
                    
                    # 计算箭头的起点和终点（在路径中点位置，方向从起点核心点指向终点核心点）
                    arrow_tail = [
                        arrow_pos[0] - dx_norm * arrow_length * 0.5,
                        arrow_pos[1] - dy_norm * arrow_length * 0.5
                    ]
                    arrow_head = [
                        arrow_pos[0] + dx_norm * arrow_length * 0.5,
                        arrow_pos[1] + dy_norm * arrow_length * 0.5
                    ]
                    
                    # 绘制箭头（红色，从起点核心点指向终点核心点）
                    ax.annotate(
                        "",
                        xy=(arrow_head[0], arrow_head[1]),
                        xytext=(arrow_tail[0], arrow_tail[1]),
                        arrowprops=dict(
                            arrowstyle="->",
                            color='red',
                            lw=2.5,
                            shrinkA=0,
                            shrinkB=0,
                            alpha=0.9
                        ),
                        zorder=14
                    )
        
        # 只对选中的段显示标注（已注释 - 不再显示Cost/Dist标签）
        # if seg_info in segments_to_label:
        #     if len(path_points) > 0:
        #         mid_idx = len(path_points) // 2
        #         mid_point = path_points[mid_idx]
        #         segment_text = f"Cost: {seg_info['cost']:.1f}\nDist: {seg_info['distance']:.1f}m"
        #         
        #         ax.text(
        #             mid_point[0], mid_point[1],
        #             segment_text,
        #             ha='center',
        #             va='center',
        #             fontsize=8,
        #             fontweight='bold',
        #             color='black',
        #             bbox=dict(
        #                 boxstyle='round,pad=0.3',
        #                 facecolor='lightyellow',
        #                 edgecolor='orange',
        #                 alpha=0.8
        #             ),
        #             zorder=15
        #         )
    
    # 只绘制起点标记（绿色圆圈，缩小为原来的1/2）
    start_idx = tour_order[0]
    if start_idx < len(merged_points):
        start_point = merged_points[start_idx]['point']
        ax.scatter(
            start_point[0], start_point[1],
            c='green',
            marker='o',
            s=75,  # 缩小为原来的1/2 (150 -> 75)
            edgecolors='darkgreen',
            linewidths=2,
            alpha=0.9,
            label='TSP Start',
            zorder=16
        )
        # 不显示起点编号（已移除标签）
    
    # 在标题区域显示总统计信息
    total_text = f"TSP Tour Statistics:\nTotal Cost: {total_cost:.2f}\nTotal Distance: {total_distance:.2f} m\nSegments: {segment_count}"
    
    # 在图的右上角添加文本
    ax.text(
        0.98, 0.98,
        total_text,
        transform=ax.transAxes,
        ha='right',
        va='top',
        fontsize=10,
        fontweight='bold',
        color='black',
        bbox=dict(
            boxstyle='round,pad=0.5',
            facecolor='yellow',
            edgecolor='black',
            alpha=0.9
        ),
        zorder=20,
        family='Times New Roman'
    )
    
    print(f"  ✓ TSP路径绘制完成")
    print(f"    总成本: {total_cost:.2f}")
    print(f"    总距离: {total_distance:.2f} m")
    print(f"    路径段数: {segment_count}")
    print(f"    显示标注的段数: {len(segments_to_label)}")


def _draw_coverage_paths(ax, coverage_result):
    """
    绘制覆盖巡检路径
    
    :param ax: Matplotlib axis
    :param coverage_result: 覆盖路径结果字典，包含:
        - coverage_paths: 覆盖路径列表
        - total_points: 总覆盖点数
        - scan_mode: 扫描模式
        - water_count: 水域块数
    """
    coverage_paths = coverage_result.get('coverage_paths', [])
    scan_mode = coverage_result.get('scan_mode', 'zigzag')
    
    if not coverage_paths:
        print("  ⚠ 没有覆盖路径可绘制")
        return
    
    print(f"  绘制覆盖路径，包含 {len(coverage_paths)} 个水域块")
    print(f"    扫描模式: {scan_mode}")
    
    # 定义颜色（为不同水域块使用不同颜色）
    colors = plt.cm.tab20(np.linspace(0, 1, len(coverage_paths)))
    
    total_coverage_points = 0
    
    for idx, path_info in enumerate(coverage_paths):
        coverage_path = path_info.get('coverage_path', [])
        water_idx = path_info.get('water_idx', idx)
        start_point = path_info.get('start_point')
        end_point = path_info.get('end_point')
        
        if not coverage_path or len(coverage_path) < 2:
            continue
        
        total_coverage_points += len(coverage_path)
        
        # 转换为numpy数组
        path_array = np.array(coverage_path)
        
        # 绘制覆盖路径（使用不同颜色区分不同水域块）
        color = colors[idx % len(colors)]
        ax.plot(
            path_array[:, 0],
            path_array[:, 1],
            color=color,
            linewidth=1.5,
            linestyle='-',
            alpha=0.8,
            label=f'Coverage Path P{water_idx}' if idx == 0 else '',
            zorder=12
        )
        
        # 绘制起点标记（绿色小圆点）
        if start_point:
            ax.scatter(
                start_point[0], start_point[1],
                c='green',
                marker='o',
                s=50,
                edgecolors='darkgreen',
                linewidths=1.5,
                alpha=0.9,
                zorder=15,
                label='Coverage Start' if idx == 0 else ''
            )
        
        # 绘制终点标记（红色小方块）
        if end_point:
            ax.scatter(
                end_point[0], end_point[1],
                c='red',
                marker='s',
                s=50,
                edgecolors='darkred',
                linewidths=1.5,
                alpha=0.9,
                zorder=15,
                label='Coverage End' if idx == 0 else ''
            )
    
    print(f"  ✓ 覆盖路径绘制完成！")
    print(f"    总覆盖点数: {total_coverage_points}")
    print(f"    水域块数: {len(coverage_paths)}")


def _add_grid_legend(ax, coverage_result=None):
    """Add grid legend"""
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    
    legend_elements = [
        Patch(facecolor='#87CEEB', edgecolor='black', label='Water Grid'),
        Patch(facecolor='#FFFFFF', edgecolor='black', label='Land Grid'),
        Patch(facecolor='#FF0000', edgecolor='black', label='No-Fly Zones')
    ]
    
    # 如果有覆盖路径，添加覆盖路径图例项
    if coverage_result:
        scan_mode = coverage_result.get('scan_mode', 'zigzag')
        coverage_paths = coverage_result.get('coverage_paths', [])
        if coverage_paths:
            # 使用第一个覆盖路径的颜色作为示例
            colors = plt.cm.tab20(np.linspace(0, 1, len(coverage_paths)))
            legend_elements.append(
                Line2D([0], [0], color=colors[0], linewidth=1.5, 
                      label=f'Coverage Path ({scan_mode})')
            )
            legend_elements.append(
                Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
                      markersize=8, label='Coverage Start', linestyle='None')
            )
            legend_elements.append(
                Line2D([0], [0], marker='s', color='w', markerfacecolor='red',
                      markersize=8, label='Coverage End', linestyle='None')
            )
    
    # Move legend to top, left-aligned with image left border, vertical arrangement
    ax.legend(handles=legend_elements, loc='upper left', fontsize=10, 
              bbox_to_anchor=(0, 1.1), ncol=1, frameon=True, 
              prop={'family': 'Times New Roman', 'weight': 'bold'})

def _add_grid_statistics(ax, drawn_stats):
    """Add grid statistics based on actually drawn grids"""
    if drawn_stats is None:
        return  # 如果没有栅格统计数据，直接返回
    
    water_grids = drawn_stats['water_grids']
    land_grids = drawn_stats['land_grids']
    boundary_grids = drawn_stats['boundary_grids']
    total_grids = water_grids + land_grids + boundary_grids
    valid_grids = water_grids + land_grids
    
    water_ratio = water_grids / valid_grids if valid_grids > 0 else 0
    land_ratio = land_grids / valid_grids if valid_grids > 0 else 0
    
    stats_text = f"""Grid Statistics:
Total Grids: {total_grids}
Water Grids: {water_grids} ({water_ratio:.1%})
Land Grids: {land_grids} ({land_ratio:.1%})
No-Fly Zones: {boundary_grids}"""
    
    # Move statistics to top, right-aligned with image right border
    ax.text(1.0, 1.02, stats_text, transform=ax.transAxes, 
           verticalalignment='bottom', horizontalalignment='right',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
           fontsize=10, family='Times New Roman', fontweight='bold')

def plot_path(sampling_points, water_with_hole, water_no_hole, boundary_polygon):
    """
    Visualize water bodies, sampling points and boundaries.
    Parameters:
        sampling_points: All sampling points (list)
        water_with_hole: Water body information with holes
        water_no_hole: Water body information without holes
        boundary_polygon: Region boundary, (N,2) ndarray
    """
    plt.figure(figsize=(10, 10))
    ax = plt.gca()

    def plot_water(water_list):
        for w in water_list:
            # Outer contour
            ax.fill(*w["outer"].T, facecolor='#87CEEB', edgecolor='k', alpha=0.6,
                    label=('Water Body' if 'Water Body' not in ax.get_legend_handles_labels()[1] else ''))

            # Holes
            for h in w["holes"]:
                ax.fill(*h.T, facecolor='white', edgecolor='k', alpha=1.0,
                        label=('Land' if 'Land' not in ax.get_legend_handles_labels()[1] else ''))

            # Outer contour sampling points (visible)
            ax.plot(w["outer_samples"][:, 0], w["outer_samples"][:, 1], 'ro', markersize=3, alpha=1.0,
                    label=('Sample Point' if 'Sample Point' not in ax.get_legend_handles_labels()[1] else ''))

            # Hole sampling points (visible)
            for s in w["hole_samples"]:
                ax.plot(s[:, 0], s[:, 1], 'go', markersize=2, alpha=1.0,
                        label=('Hole Sample' if 'Hole Sample' not in ax.get_legend_handles_labels()[1] else ''))

            # Center number
            cx, cy = w["outer"].mean(axis=0)
            ax.text(cx, cy, str(w["idx"]), ha='center', va='center',
                    fontsize=9, fontweight='bold')

    # Draw water bodies
    plot_water(water_with_hole)
    plot_water(water_no_hole)

    # Draw boundary
    ax.plot(boundary_polygon[:, 0], boundary_polygon[:, 1], 'k-', linewidth=1, label='Boundary')

    # 创建水域的Path对象用于碰撞检测
    water_polygons = []
    for water in water_with_hole + water_no_hole:
        # 创建主水体多边形
        main_poly = Polygon(water["outer"], closed=True)
        # 创建孔洞多边形
        holes = [Polygon(hole, closed=True) for hole in water["holes"]]
        water_polygons.append((main_poly, holes))

    # 找到适合的轨迹（只在蓝色水域内）
    def is_in_water(x, y):
        point = np.array([[x, y]])
        for i, (main_poly, holes) in enumerate(water_polygons):
            if main_poly.contains_points(point)[0]:
                # 检查是否在任何孔洞内
                in_hole = False
                for hole in holes:
                    if hole.contains_points(point)[0]:
                        in_hole = True
                        break
                if not in_hole:
                    return True
        return False

    # =============================================================================
    # TSP结果显示部分 - 暂时注释掉
    # =============================================================================
    
    # # 读取LKH求解结果并可视化路径
    # try:
    #     solution_file = r".\data\sampling_points_solution.txt"
    #     with open(solution_file, "r") as f:
    #         lines = f.readlines()
    #     
    #     # 解析LKH输出格式，找到TOUR_SECTION
    #     tour_section_found = False
    #     tour_indices = []
    #     
    #     for line in lines:
    #         line = line.strip()
    #         if line == "TOUR_SECTION":
    #             tour_section_found = True
    #             print("Found TOUR_SECTION")
    #             continue
    #         elif line == "-1" or line == "EOF":
    #             print(f"End of tour section, found {len(tour_indices)} indices")
    #             break
    #         elif tour_section_found and line.isdigit():
    #             # LKH从1开始，转换为0开始，并确保索引有效
    #             index = int(line) - 1
    #             if 0 <= index < len(sampling_points):
    #                 tour_indices.append(index)
    #             else:
    #                 print(f"Warning: Invalid index {line} -> {index}, max valid index is {len(sampling_points)-1}")
    #     
    #     print(f"Parsed tour indices: {tour_indices[:10]}...")  # 显示前10个索引
    #     print(f"Total valid indices: {len(tour_indices)}")
    #     print(f"Sampling points count: {len(sampling_points)}")
    #     
    #     # 调试：检查前几个采样点是否在水域内
    #     print("Debug: Checking first 5 sampling points:")
    #     for i in range(min(5, len(sampling_points))):
    #         pt = sampling_points[i]
    #         in_water = is_in_water(pt[0], pt[1])
    #         print(f"  Point {i}: ({pt[0]:.1f}, {pt[1]:.1f}) -> {'IN WATER' if in_water else 'NOT IN WATER'}")
    #     
    #     if tour_indices:
    #         # 按求解结果顺序连接采样点
    #         ordered_points = [sampling_points[i] for i in tour_indices]
    #         ordered_points.append(ordered_points[0])  # 闭合路径
    #         
    #         # 绘制最优路径，检测线段是否经过水域
    #         def is_line_in_water(p1, p2, num_samples=10):
    #             """检测线段是否有80%以上的区域在水域内"""
    #             water_count = 0
    #             for i in range(num_samples + 1):
    #                 t = i / num_samples
    #                 x = p1[0] + t * (p2[0] - p1[0])
    #                 y = p1[1] + t * (p2[1] - p1[1])
    #                 if is_in_water(x, y):
    #                     water_count += 1
    #             
    #             # 如果80%以上的点在水域内，则认为是水域路径
    #             water_ratio = water_count / (num_samples + 1)
    #             return water_ratio >= 0.8  # 如果80%以上在水域内，则认为是水域路径（即20%以上在非水域就变红）
    #         
    #         # 分段绘制路径，根据是否在水域内选择颜色
    #         water_segments = 0
    #         non_water_segments = 0
    #         for i in range(len(ordered_points) - 1):
    #             p1 = ordered_points[i]
    #             p2 = ordered_points[i + 1]
    #             
    #             if is_line_in_water(p1, p2):
    #                 # 线段在水域内，用蓝色
    #                 ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', linewidth=2, 
    #                        label='LKH Optimal Path (Water)' if i == 0 else "")
    #                 water_segments += 1
    #             else:
    #                 # 线段经过非水域区域，用红色
    #                 ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-', linewidth=2, 
    #                        label='LKH Path (Non-water)' if i == 0 else "")
    #                 non_water_segments += 1
    #                 # 调试：输出红色线段的信息
    #                 if non_water_segments <= 5:  # 只输出前5个红色线段
    #                     print(f"  Red segment {non_water_segments}: ({p1[0]:.1f}, {p1[1]:.1f}) -> ({p2[0]:.1f}, {p2[1]:.1f})")
    #         
    #         print(f"Path statistics: {water_segments} blue segments, {non_water_segments} red segments")
    #         
    #         # 在路径起点标记
    #         ax.plot(ordered_points[0][0], ordered_points[0][1], 'go', markersize=8, label='Start/End Point')
    #         
    #         print(f"LKH solution loaded: {len(tour_indices)} points in optimal tour")
    #     else:
    #         print("No valid tour indices found")
    # except FileNotFoundError:
    #     print(f"sampling_points_solution.txt not found at {solution_file}, skipping LKH path visualization")
    # except Exception as e:
    #     print(f"Error reading LKH solution: {e}")
    #     import traceback
    #     traceback.print_exc()

    # =============================================================================
    # 轨迹生成部分 - 暂时注释掉（TSP相关功能）
    # =============================================================================
    
    # # 生成轨迹点（确保在水域内）
    # trajectory_points = []
    # x_min, x_max = boundary_polygon[:, 0].min(), boundary_polygon[:, 0].max()
    # y_min, y_max = boundary_polygon[:, 1].min(), boundary_polygon[:, 1].max()
    # 
    # # 尝试多条可能的线段，找到一条完全在水域内的
    # for attempt in range(10):
    #     # 随机生成起点和终点（在水域内）
    #     while True:
    #         start_x = np.random.uniform(x_min, x_max)
    #         start_y = np.random.uniform(y_min, y_max)
    #         if is_in_water(start_x, start_y):
    #             break
    #             
    #     while True:
    #         end_x = np.random.uniform(x_min, x_max)
    #         end_y = np.random.uniform(y_min, y_max)
    #         if is_in_water(end_x, end_y):
    #             break
    #     
    #     # 生成线段上的点
    #     temp_traj_x = np.linspace(start_x, end_x, 50)
    #     temp_traj_y = np.linspace(start_y, end_y, 50)
    #     
    #     # 检查所有点是否在水域内
    #     valid = True
    #     for x, y in zip(temp_traj_x, temp_traj_y):
    #         if not is_in_water(x, y):
    #             valid = False
    #             break
    #             
    #     if valid:
    #         trajectory_points = list(zip(temp_traj_x, temp_traj_y))
    #         break
    # 
    # if not trajectory_points:
    #     # 如果找不到完全在水域内的直线，使用折线
    #     mid_x = (x_min + x_max) / 2
    #     mid_y = (y_min + y_max) / 2
    #     waypoints = []
    #     
    #     # 生成多个路径点（确保在水域内）
    #     for _ in range(3):
    #         while True:
    #             x = np.random.uniform(x_min, x_max)
    #             y = np.random.uniform(y_min, y_max)
    #             if is_in_water(x, y):
    #                 waypoints.append((x, y))
    #                 break
    #     
    #     # 在路径点之间插值
    #     trajectory_points = []
    #     for i in range(len(waypoints)-1):
    #         seg_x = np.linspace(waypoints[i][0], waypoints[i+1][0], 20)
    #         seg_y = np.linspace(waypoints[i][1], waypoints[i+1][1], 20)
    #         for x, y in zip(seg_x, seg_y):
    #             if is_in_water(x, y):
    #                 trajectory_points.append((x, y))
    # 
    # if trajectory_points:
    #     traj_x, traj_y = zip(*trajectory_points)
    #     # 绘制深蓝色虚线轨迹
    #     ax.plot(traj_x, traj_y, '--', color='darkblue', linewidth=1.5, label='Boat Trajectory')
    #     
    #     # 在轨迹上添加船形标记（红色三角形）
    #     boat_pos = len(traj_x) // 2
    #     boat_marker = MarkerStyle('^')
    #     ax.plot(traj_x[boat_pos], traj_y[boat_pos], marker=boat_marker, 
    #             color='red', markersize=10, label='Boat')

    # Set axis limits to show only inside the boundary
    x_min, x_max = boundary_polygon[:, 0].min(), boundary_polygon[:, 0].max()
    y_min, y_max = boundary_polygon[:, 1].min(), boundary_polygon[:, 1].max()
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Water Terrain & Sampling Points with Boundary")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)
    
    # 获取并显示图例
    handles, labels = ax.get_legend_handles_labels()
    unique_labels = []
    unique_handles = []
    for handle, label in zip(handles, labels):
        if label not in unique_labels:
            unique_labels.append(label)
            unique_handles.append(handle)
    ax.legend(unique_handles, unique_labels)

    plt.show()
# ##V0.2
# import matplotlib.pyplot as plt
# import numpy as np

# def plot_path(sampling_points, water_with_hole, water_no_hole, boundary_polygon):
#     """
#     可视化水体、采样点与边界。
#     参数:
#         sampling_points: 所有采样点（列表）
#         water_with_hole: 含孔洞水体信息
#         water_no_hole: 无孔洞水体信息
#         boundary_polygon: 区域边界，(N,2) ndarray
#     """
#     plt.figure(figsize=(10, 10))
#     ax = plt.gca()

#     def plot_water(water_list):
#         for w in water_list:
#             # 外轮廓
#             ax.fill(*w["outer"].T, facecolor='#87CEEB', edgecolor='k', alpha=0.6,
#                     label=('Water Body' if 'Water Body' not in ax.get_legend_handles_labels()[1] else ''))

#             # 孔洞
#             for h in w["holes"]:
#                 ax.fill(*h.T, facecolor='white', edgecolor='k', alpha=1.0,
#                         label=('Land' if 'Land' not in ax.get_legend_handles_labels()[1] else ''))

#             # 外轮廓采样点（设为透明）
#             ax.plot(w["outer_samples"][:, 0], w["outer_samples"][:, 1], 'ro', markersize=3, alpha=0,
#                     label=('Sample Point' if 'Sample Point' not in ax.get_legend_handles_labels()[1] else ''))

#             # 孔洞采样点（设为透明）
#             for s in w["hole_samples"]:
#                 ax.plot(s[:, 0], s[:, 1], 'go', markersize=2, alpha=0,
#                         label=('Hole Sample' if 'Hole Sample' not in ax.get_legend_handles_labels()[1] else ''))

#             # 中心编号（保持可见）
#             cx, cy = w["outer"].mean(axis=0)
#             ax.text(cx, cy, str(w["idx"]), ha='center', va='center',
#                     fontsize=9, fontweight='bold')

#     # 绘制水体
#     plot_water(water_with_hole)
#     plot_water(water_no_hole)

#     ax.set_aspect('equal', adjustable='box')
#     ax.set_title("Water Terrain & Sampling Points with Boundary")
#     ax.set_xlabel("X (m)")
#     ax.set_ylabel("Y (m)")
#     ax.grid(True)
#     handles, labels = ax.get_legend_handles_labels()
#     ax.legend(handles, labels)

#     plt.show()

# # V0.1

# import matplotlib.pyplot as plt
# import numpy as np

# def plot_water_map(water_with_hole, water_no_hole, sample_dist=10.0):
#     """
#     可视化水体信息和采样点分布。
#     参数:
#         water_with_hole: 含孔洞水体记录列表，每项为 dict
#         water_no_hole: 无孔洞水体记录列表，每项为 dict
#         sample_dist: 采样点间距（用于标题显示）
#     """

#     plt.figure(figsize=(10, 10))
#     ax = plt.gca()

#     def plot_water(water_list):
#         for w in water_list:
#             # 原始水体外轮廓
#             ax.fill(*w["outer"].T, facecolor='#87CEEB', edgecolor='k', alpha=0.6,
#                     label=('Water Body' if 'Water Body' not in ax.get_legend_handles_labels()[1] else ''))

#             # 孔洞
#             for h in w["holes"]:
#                 ax.fill(*h.T, facecolor='white', edgecolor='k', alpha=1.0,
#                         label=('Land' if 'Land' not in ax.get_legend_handles_labels()[1] else ''))

#             # 外部采样点
#             ax.plot(w["outer_samples"][:, 0], w["outer_samples"][:, 1], 'ro', markersize=3,
#                     label=('Sample Point' if 'Sample Point' not in ax.get_legend_handles_labels()[1] else ''))

#             # 孔洞采样点
#             for s in w["hole_samples"]:
#                 ax.plot(s[:, 0], s[:, 1], 'go', markersize=2,
#                         label=('Hole Sample' if 'Hole Sample' not in ax.get_legend_handles_labels()[1] else ''))

#             # 中心编号
#             cx, cy = w["outer"].mean(axis=0)
#             ax.text(cx, cy, str(w["idx"]), ha='center', va='center',
#                     fontsize=9, fontweight='bold')

#     # 先画带孔洞水体
#     plot_water(water_with_hole)

#     # 后画无孔洞水体
#     plot_water(water_no_hole)

#     # 设置图形属性
#     ax.set_aspect('equal', adjustable='box')
#     ax.set_title(f"Water Terrain & Sample Points (Interval = {sample_dist} m)")
#     ax.set_xlabel("X (m)")
#     ax.set_ylabel("Y (m)")
#     ax.grid(True)
#     handles, labels = ax.get_legend_handles_labels()
#     ax.legend(handles, labels)

#     plt.show()


def _draw_block1_coverage_start(ax, start_point):
    """
    绘制第一个水域块覆盖路径的起点（绿色点，在水域边界上，样式与其他水域块起点一致）
    
    :param ax: matplotlib axes对象
    :param start_point: 起点坐标 [x, y]
    """
    if start_point is None or len(start_point) < 2:
        return
    
    ax.scatter(
        start_point[0], start_point[1],
        c='green',
        marker='o',
        s=10,
        edgecolors='darkgreen',
        linewidths=2.0,
        alpha=1.0,
        label='Water Entry Point (First)',
        zorder=30
    )


def _draw_block1_coverage_exit(ax, exit_point):
    """
    绘制第一个水域块覆盖路径的终点（红色圆点，在水域边界上，样式与其他水域块终点一致）
    
    :param ax: matplotlib axes对象
    :param exit_point: 终点坐标 [x, y]
    """
    if exit_point is None or len(exit_point) < 2:
        return
    
    ax.scatter(
        exit_point[0], exit_point[1],
        c='red',
        marker='o',  # 与其他水域块终点一致
        s=10,
        edgecolors='darkred',
        linewidths=2.0,
        alpha=1.0,
        label='Coverage Exit',
        zorder=30
    )


def _draw_block1_coverage_path(ax, sampling_points, tour_order):
    """
    绘制第一个水域块的覆盖路径
    
    :param ax: Matplotlib axis
    :param sampling_points: 采样点列表 [[x, y], ...]，起点在索引0，退出点在索引1
    :param tour_order: TSP访问顺序列表（点索引，Python 0-based）
    """
    if not tour_order or len(tour_order) < 2:
        print("  ⚠ 第一个水域块覆盖路径数据不完整，跳过可视化")
        return
    
    if len(sampling_points) == 0:
        print("  ⚠ 采样点列表为空，跳过可视化")
        return
    
    print(f"  绘制第一个水域块覆盖路径，包含 {len(tour_order)} 个点")
    
    # 绘制路径线段
    for i in range(len(tour_order) - 1):
        from_idx = tour_order[i]
        to_idx = tour_order[i + 1]
        
        # 检查索引有效性
        if from_idx >= len(sampling_points) or to_idx >= len(sampling_points):
            continue
        
        point_from = sampling_points[from_idx]
        point_to = sampling_points[to_idx]
        
        # 绘制路径线段（红色，线宽3）
        ax.plot(
            [point_from[0], point_to[0]],
            [point_from[1], point_to[1]],
            color='red',
            linewidth=3,
            linestyle='-',
            alpha=0.9,
            label='First Water Coverage Path' if i == 0 else '',
            zorder=15
        )
    
    print(f"  ✓ 第一个水域块覆盖路径绘制完成")


def _draw_all_blocks_coverage_paths(ax, all_blocks_coverage_paths):
    """
    绘制所有水域块的覆盖路径（按 tour_order 连线的路径）
    :param ax: Matplotlib axis
    :param all_blocks_coverage_paths: 字典 { block_no: {'sampling_points': [[x,y],...], 'tour_order': [...]} }
    """
    if not all_blocks_coverage_paths:
        return
    import numpy as np
    colors = plt.cm.tab20(np.linspace(0, 1, max(len(all_blocks_coverage_paths), 1)))
    for ki, block_no in enumerate(sorted(all_blocks_coverage_paths.keys())):
        info = all_blocks_coverage_paths[block_no]
        sp = info.get("sampling_points") if isinstance(info, dict) else None
        tour = info.get("tour_order") if isinstance(info, dict) else None
        if not tour or len(tour) < 2 or not sp or len(sp) == 0:
            if sp and len(sp) > 0 and (not tour or len(tour) < 2):
                print(f"    [跳过] 水域块 {block_no}：无有效访问顺序 (tour 为空或点数<2)")
            continue
        c = colors[ki % len(colors)]
        for i in range(len(tour) - 1):
            from_idx = tour[i]
            to_idx = tour[i + 1]
            if from_idx >= len(sp) or to_idx >= len(sp):
                continue
            point_from = sp[from_idx]
            point_to = sp[to_idx]
            ax.plot(
                [point_from[0], point_to[0]],
                [point_from[1], point_to[1]],
                color=c,
                linewidth=3,
                linestyle='-',
                alpha=0.9,
                label=f'Block {block_no} Coverage' if i == 0 else '',
                zorder=15
            )
    print(f"  绘制各块覆盖路径，共 {len(all_blocks_coverage_paths)} 块")


def _draw_block1_sampling_points(ax, sampling_points):
    """
    绘制第一个水域块的采样点（黑色小点）
    
    :param ax: matplotlib axes对象
    :param sampling_points: 采样点列表 [[x, y], ...]
    """
    if sampling_points is None or len(sampling_points) == 0:
        return
    
    # 提取x和y坐标
    x_coords = [point[0] for point in sampling_points]
    y_coords = [point[1] for point in sampling_points]
    
    ax.scatter(
        x_coords, y_coords,
        c='black',
        marker='o',
        s=4,  # 采样点尺寸
        edgecolors='black',
        linewidths=0.3,
        alpha=0.8,  # 保持较高透明度
        label='Sampling Points',
        zorder=25  # 置于水域/骨架路径之上，便于看见采样点
    )


def _draw_all_blocks_sampling_points(ax, all_blocks_coverage_paths):
    """
    绘制所有水域块的采样点（每块一色，便于区分）
    
    :param ax: matplotlib axes 对象
    :param all_blocks_coverage_paths: 字典 { block_no: {'sampling_points': [[x,y],...], 'tour_order': [...]} }
    """
    if not all_blocks_coverage_paths:
        return
    import numpy as np
    colors = plt.cm.tab20(np.linspace(0, 1, max(len(all_blocks_coverage_paths), 1)))
    for ki, block_no in enumerate(sorted(all_blocks_coverage_paths.keys())):
        info = all_blocks_coverage_paths[block_no]
        sp = info.get("sampling_points") if isinstance(info, dict) else None
        if not sp or len(sp) == 0:
            continue
        x_coords = [p[0] for p in sp]
        y_coords = [p[1] for p in sp]
        c = colors[ki % len(colors)]
        ax.scatter(
            x_coords, y_coords,
            c=[c],
            marker='o',
            s=4,  # 采样点尺寸
            edgecolors='none',
            alpha=0.85,
            label='Sampling Points' if ki == 0 else None,
            zorder=25,  # 置于水域/骨架路径之上，便于看见采样点
        )


def _draw_second_water_entry_exit_points(ax, start_point, exit_point):
    """
    绘制第二个水域块的起点和终点
    
    :param ax: matplotlib axes对象
    :param start_point: 起点坐标 [x, y]（绿色，大小3）
    :param exit_point: 终点坐标 [x, y]（红色，大小3）
    """
    if start_point is not None:
        print(f"  绘制第二个水域块起点: ({start_point[0]:.2f}, {start_point[1]:.2f})")
        ax.scatter(
            start_point[0], start_point[1],
            c='green',
            marker='o',
            s=10,  # 点大小改为10
            edgecolors='darkgreen',  # 使用更深的绿色边框，更容易看到
            linewidths=2.0,  # 增加边框宽度
            alpha=1.0,
            label='Second Water Start' if exit_point is None else '',
            zorder=30  # 提高zorder，确保显示在最顶层
        )
        print(f"    ✓ 绿色起点已绘制: 坐标=({start_point[0]:.2f}, {start_point[1]:.2f}), zorder=30")
    else:
        print(f"    ⚠ 第二个水域块起点为None，无法绘制")
    
    if exit_point is not None:
        print(f"  绘制第二个水域块终点: ({exit_point[0]:.2f}, {exit_point[1]:.2f})")
        ax.scatter(
            exit_point[0], exit_point[1],
            c='red',
            marker='o',
            s=10,  # 点大小改为10
            edgecolors='darkred',  # 使用更深的红色边框，更容易看到
            linewidths=2.0,  # 增加边框宽度
            alpha=1.0,
            label='Second Water Exit' if start_point is None else '',
            zorder=30  # 提高zorder，确保显示在最顶层
        )
        print(f"    ✓ 红色终点已绘制: 坐标=({exit_point[0]:.2f}, {exit_point[1]:.2f}), zorder=30")
    else:
        print(f"    ⚠ 第二个水域块终点为None，无法绘制")


def _draw_all_water_entry_exit_points(ax, all_water_entry_exit_points):
    """
    绘制所有中间水域块的起点和终点（包括最后一个水域块的起点）
    
    :param ax: matplotlib axes对象
    :param all_water_entry_exit_points: 字典 {water_idx: {'start_point': [x, y], 'exit_point': [x, y] or None, 'tour_idx': int}}
    """
    if not all_water_entry_exit_points:
        return
    
    # 为每个水域块绘制起点和终点
    first_key = list(all_water_entry_exit_points.keys())[0]
    for water_idx, points_info in all_water_entry_exit_points.items():
        start_point = points_info.get('start_point')
        exit_point = points_info.get('exit_point')
        
        # 绘制起点（绿色）
        if start_point is not None:
            ax.scatter(
                start_point[0], start_point[1],
                c='green',
                marker='o',
                s=10,
                edgecolors='darkgreen',
                linewidths=2.0,
                alpha=1.0,
                zorder=30,
                label='Water Entry Point' if water_idx == first_key else ''
            )
        
        # 绘制终点（红色）- 注意：最后一个水域块可能没有exit_point
        if exit_point is not None:
            ax.scatter(
                exit_point[0], exit_point[1],
                c='red',
                marker='o',
                s=10,
                edgecolors='darkred',
                linewidths=2.0,
                alpha=1.0,
                zorder=30,
                label='Water Exit Point' if water_idx == first_key else ''
            )


def _draw_water_block_number_labels(ax, all_water_entry_exit_points, first_block_coverage_start=None):
    """
    在各水域块旁绘制小数字，表示骨架路径访问的第几块。
    第1块用 first_block_coverage_start 位置；第2块及以后用 all_water_entry_exit_points 的 key 与 start_point。
    """
    if ax is None:
        return
    # 小号字体，黑色，略偏上避免压住起点圆点
    fontsize = 6
    color = 'black'
    dy = 3.0  # 竖直偏移（米）
    zorder = 31
    # 第1块
    if first_block_coverage_start is not None:
        x, y = first_block_coverage_start[0], first_block_coverage_start[1]
        ax.text(x, y + dy, '1', fontsize=fontsize, color=color, ha='center', va='bottom', zorder=zorder, fontweight='bold')
    if not all_water_entry_exit_points:
        return
    for block_no, points_info in all_water_entry_exit_points.items():
        start_point = points_info.get('start_point')
        exit_point = points_info.get('exit_point')
        # 取起点；无起点则取终点
        pt = start_point if start_point is not None else exit_point
        if pt is None:
            continue
        x, y = pt[0], pt[1]
        ax.text(x, y + dy, str(block_no), fontsize=fontsize, color=color, ha='center', va='bottom', zorder=zorder, fontweight='bold')


def _draw_water_entry_exit_points(ax, all_water_entry_exit_points):
    """
    绘制所有水域块（单连通+聚类）的起点和终点，统一风格：绿色起点、红色终点。
    
    规则：
    - 若 exit_point 为 None（最后一点），只画绿色起点
    - 若起点与终点重合（距离<1m），只画红色点
    - 否则画绿色起点与红色终点
    
    :param ax: matplotlib axes
    :param all_water_entry_exit_points: 合并字典，key 为 water_idx（单连通）或 (water_idx, point_idx)（聚类），value 含 start_point、exit_point
    """
    if not all_water_entry_exit_points:
        return
    
    import numpy as np
    
    COINCIDENCE_THRESHOLD = 1.0  # 米
    first_key = list(all_water_entry_exit_points.keys())[0]

    for key, points_info in all_water_entry_exit_points.items():
        start_point = points_info.get('start_point')
        exit_point = points_info.get('exit_point')
        if start_point is None:
            continue

        if exit_point is None:
            ax.scatter(
                start_point[0], start_point[1],
                c='green', marker='o', s=10, edgecolors='darkgreen',
                linewidths=2.0, alpha=1.0, zorder=30,
                label='Water Entry Point (Last)' if key == first_key else ''
            )
            continue

        distance = np.sqrt((start_point[0] - exit_point[0])**2 + (start_point[1] - exit_point[1])**2)
        if distance < COINCIDENCE_THRESHOLD:
            ax.scatter(
                exit_point[0], exit_point[1],
                c='red', marker='o', s=10, edgecolors='darkred',
                linewidths=2.0, alpha=1.0, zorder=30,
                label='Water Entry/Exit Point' if key == first_key else ''
            )
        else:
            ax.scatter(
                start_point[0], start_point[1],
                c='green', marker='o', s=10, edgecolors='darkgreen',
                linewidths=2.0, alpha=1.0, zorder=30,
                label='Water Entry Point' if key == first_key else ''
            )
            ax.scatter(
                exit_point[0], exit_point[1],
                c='red', marker='o', s=10, edgecolors='darkred',
                linewidths=2.0, alpha=1.0, zorder=30,
                label='Water Exit Point' if key == first_key else ''
            )


def _draw_second_water_label(ax, env_data, water_idx):
    """
    在图中标出水域块2
    
    :param ax: matplotlib axes对象
    :param env_data: 环境数据
    :param water_idx: 水域块2的water_idx
    """
    from shapely.geometry import Polygon as ShapelyPolygon
    
    water_with_hole = env_data.get("water_with_hole", [])
    water_no_hole = env_data.get("water_no_hole", [])
    
    # 查找水域块2
    water_info = None
    for water in water_with_hole:
        if water.get('idx') == water_idx:
            water_info = water
            break
    
    if water_info is None:
        for water in water_no_hole:
            if water.get('idx') == water_idx:
                water_info = water
                break
    
    if water_info is None:
        return
    
    # 创建水域多边形
    water_poly = ShapelyPolygon(
        water_info['outer'],
        holes=water_info.get('holes', [])
    )
    
    # 获取水域块的几何中心
    centroid = water_poly.centroid
    
    print(f"  绘制水域块2标注: water_idx={water_idx}, 中心点=({centroid.x:.2f}, {centroid.y:.2f})")
    
    # 在几何中心标注"水域块2" - 使用annotation确保显示在最顶层
    ax.annotate(
        'Water Block 2',
        xy=(centroid.x, centroid.y),
        ha='center',
        va='center',
        fontsize=16,
        fontweight='bold',
        color='blue',
        bbox=dict(
            boxstyle='round,pad=0.8',
            facecolor='yellow',
            edgecolor='blue',
            linewidth=3,
            alpha=0.9
        ),
        zorder=100  # 设置非常高的zorder确保显示在最顶层
    )
