# main.py 执行流程模拟

## 当前配置参数

```python
CALCULATE_ALL_PAIRS = False          # 不计算所有点对
SOLVE_TSP_WITH_LKH = False          # 不调用LKH求解器
VISUALIZE_TSP_TOUR = True            # 读取TSP数据用于可视化
SHOW_GRID_SKELETON_MAP = False       # 不生成栅格化-骨架路径图
SHOW_COVERAGE_SKELETON_MAP = False   # 不生成覆盖路径-骨架路径图
SHOW_COVERAGE_PATH_MAP = True        # 生成覆盖路径生成可视化地图
PRINT_ENTRY_EXIT_DEBUG = False       # 不打印调试信息
```

---

## 执行流程

### 步骤1: 加载环境对象
```
执行: env = OSM_ENV(UAV=1)
输出: "1. 加载环境对象..."
结果: 创建OSM环境对象
```

### 步骤2: 提取地图数据
```
执行: extract_env_cartesian(env, shrink_dist=0.5, sample_dist=10)
输出: "2. 提取地图数据..."
结果: 
  - sampling_points: 采样点列表
  - water_with_hole: 有孔洞水域列表
  - water_no_hole: 无孔洞水域列表
  - boundary_polygon: 边界多边形
  - water_hierarchy: 水域层级关系
```

### 步骤3: 生成栅格地图
```
执行: GridGenerator(grid_size=4.0, boundary_padding=50.0)
      grid_data = grid_generator.generate_grid(env_data)
输出: "\n3. 生成栅格地图..."
结果: 
  - grid_data['grid_type']: 栅格类型矩阵 (-1:禁飞区, 0:陆地, 1:水域)
  - grid_data['grid_centers']: 栅格中心坐标
  - grid_data['bounds']: 边界信息
```

### 步骤3.5: 计算核心点
```
执行: compute_core_points(env_data, grid_data)
输出: "\n3.5. 计算水域核心点..."
      "  ✓ 核心点生成完成！总核心点数: {total_core_points}"
结果:
  - core_points_data: 包含所有核心点数据
    - water_no_hole_core_points: 无孔洞水域核心点
    - water_with_hole_core_points: 有孔洞水域核心点
```

### 步骤4: 初始化成本计算器
```
执行: CostCalculator(base_water_cost=1.0, land_cost=10.0, outside_boundary_cost=100.0)
输出: "\n4. 初始化成本计算器..."
结果: cost_calculator 对象创建
```

### 步骤4.5: 计算核心点路径（跳过）
```
条件: CALCULATE_ALL_PAIRS = False
执行: 跳过整个 if CALCULATE_ALL_PAIRS 块
结果: path_result = None
```

### 步骤4.6: TSP求解（跳过）
```
条件: SOLVE_TSP_WITH_LKH = False
执行: 跳过整个 if SOLVE_TSP_WITH_LKH 块
结果: 不调用LKH求解器
```

### 步骤5: 读取TSP路径数据用于可视化
```
条件: VISUALIZE_TSP_TOUR = True
执行:
  1. 读取 core_points_tsp_path.txt（TSP访问顺序）
  2. 读取 core_points_path.txt（路径详情）
  3. 合并核心点获取坐标
输出: "\n5. 读取TSP路径数据用于可视化..."
      "  ✓ TSP路径数据准备完成"
      "    访问顺序: {len(tour_order)} 个点"
      "    可用路径: {len(paths_dict)} 条"
结果:
  - tsp_tour_result = {
      'tour_order': [0, 1, 2, ...],  # TSP访问顺序
      'paths_dict': {...},           # 路径字典
      'merged_points': [...]         # 合并后的核心点列表
    }
  或
  - tsp_tour_result = None（如果文件不存在）
```

### 步骤5: 处理第一个水域块的覆盖路径
```
执行: process_first_water_coverage(
        tsp_tour_result,
        water_no_hole,
        water_with_hole,
        grid_data,
        env_data
      )
输出: "\n5. 处理第一个水域块的覆盖路径..."
      "[调试] 进入水域块1覆盖路径生成函数"
      "[调试] TSP访问顺序: [0, 1, 2, ...]"
      "[调试] 第一个核心点索引: 0, 第二个核心点索引: 1"
      
流程:
  1. 检查 tsp_tour_result 是否有效
     - 如果无效 → 返回 (None, None, None, None)
  
  2. 获取第一个水域块信息
     - first_idx = tour_order[0]
     - second_idx = tour_order[1]
     - 查找对应的水域信息（water_no_hole 或 water_with_hole）
     - 如果找不到 → 打印错误，返回 (None, None, None, None)
  
  3. 计算起点和终点（骨架路径交点方法）
     - 获取骨架路径: paths_dict["0->1"] 或 paths_dict["1->0"]
     - 如果路径不存在 → 打印错误，返回
     - 计算骨架路径与水域外边界（exterior）的交点
     - 如果交点不足2个 → 打印错误，返回
     - 按路径距离排序，第一个交点为起点，最后一个交点为终点
     - 输出: "  ✓ 计算第一个水域块覆盖路径起点: (x, y)"
            "  ✓ 计算第一个水域块覆盖路径终点: (x, y)"
  
  4. 生成采样点
     - 调用 generate_first_water_sampling_points()
     - 使用基向量方法生成网格采样点
     - 如果起点或终点为None → 返回空列表
  
  5. 计算成本矩阵
     - 调用 calculate_sampling_points_cost_matrix()
     - 如果采样点不足2个 → 返回
  
  6. 生成TSP和PAR文件
     - 调用 generate_first_water_tsp_files()
     - 输出: "  ✓ TSP文件: ..."
            "  ✓ PAR文件: ..."
            "  ✓ 解决方案文件: ..."
  
  7. 调用LKH-3.exe求解TSP
     - 调用 solve_tsp_with_lkh()
     - 输出: "  调用LKH-3.exe求解第一个水域块采样点TSP问题..."
     - 如果求解失败 → 返回
  
  8. 处理TSP结果
     - 移除虚拟节点
     - 调整顺序确保从起点到终点
     - 输出: "  ✓ TSP求解完成！"
            "    访问顺序包含 {len(tour_order)} 个点"
     - 保存到 first_water_sampling_points_tsp_path.txt

结果:
  - first_water_coverage_start: [x, y] 或 None
  - first_water_sampling_points: [[x, y], ...] 或 None
  - first_water_coverage_tour: [0, 1, 2, ...] 或 None
  - first_water_exit_point: [x, y] 或 None
```

### 步骤5.1: 计算第二个水域块的起点和终点
```
条件: tsp_tour_result 存在且 tour_order 长度 >= 2
执行: calculate_water_entry_exit_points(...)
输出: （如果 PRINT_ENTRY_EXIT_DEBUG = True）
      "第二个水域块 (water_idx=19): 起点=(x, y) 终点=(x, y)"
结果:
  - second_water_start: [x, y] 或 None
  - second_water_exit: [x, y] 或 None
  - second_water_idx: int 或 None
```

### 步骤5.2: 计算所有聚类水域块的起点和终点
```
条件: tsp_tour_result 存在且 tour_order 长度 >= 2 且 core_points_data 存在
执行: calculate_all_clustered_water_entry_exit_points(...)
结果:
  - all_clustered_water_entry_exit_points: {water_idx: {'start': [...], 'exit': [...]}, ...}
```

### 步骤5.3: 计算所有单连通水域块的起点和终点
```
条件: tsp_tour_result 存在且 tour_order 长度 >= 2
执行: calculate_all_single_water_entry_exit_points(...)
结果:
  - all_single_water_entry_exit_points: {water_idx: {'start': [...], 'exit': [...]}, ...}
```

### 步骤6: 可视化栅格地图
```
输出: "\n6. 可视化栅格地图..."
```

#### 图1: 栅格化-骨架路径图（跳过）
```
条件: SHOW_GRID_SKELETON_MAP = False
执行: 跳过
```

#### 图2: 覆盖路径-骨架路径图（跳过）
```
条件: SHOW_COVERAGE_SKELETON_MAP = False
执行: 跳过
```

#### 图3: 覆盖路径生成可视化地图（执行）
```
条件: SHOW_COVERAGE_PATH_MAP = True
执行: visualize_grid_map(
        ...
        first_water_coverage_start=first_water_coverage_start,
        first_water_sampling_points=first_water_sampling_points,
        first_water_coverage_tour=first_water_coverage_tour,
        show_grid=False,
        show_sampling_points=True
      )
输出: "\n  生成覆盖路径生成可视化地图..."
      "  ✓ 覆盖路径生成可视化地图已保存: coverage_path_map.png"

可视化内容:
  - 水域边界（含孔洞）
  - 核心点（五角星/菱形）
  - TSP骨架路径（蓝色线）
  - 第一个水域块起点（红色点）
  - 第一个水域块采样点（黑色小点，如果 show_sampling_points=True）
  - 第一个水域块覆盖路径（红色线，如果 tour_order 和 sampling_points 都不为None）
  - 第二个水域块起点和终点（绿色/红色点）
  - 其他水域块起点和终点
  - 禁飞区（红色区域，不显示栅格线）
```

### 显示图形
```
条件: SHOW_GRID_SKELETON_MAP or SHOW_COVERAGE_SKELETON_MAP or SHOW_COVERAGE_PATH_MAP = True
执行: plt.show()
结果: 显示 coverage_path_map.png
```

### 统计第一个水域块覆盖路径信息
```
条件: first_water_sampling_points is not None and first_water_coverage_tour is not None
执行: calculate_first_water_coverage_statistics(...)
结果: 计算并打印覆盖路径统计信息
```

### 输出总结
```
输出:
  "运行完成！"
  "核心点: {total_core_points} 个"
  "TSP求解结果: ..."
  "栅格地图: {rows} x {cols}"
  "生成文件: ..."
```

---

## 关键判断点

### 1. TSP数据是否可用？
- **如果 tsp_tour_result = None**:
  - `process_first_water_coverage` 会立即返回 (None, None, None, None)
  - 不会生成覆盖路径
  - 可视化中不会显示覆盖路径

### 2. 第一个水域块起点终点计算是否成功？
- **如果骨架路径与水域边界交点不足2个**:
  - 会打印: "✗ 水域块X与骨架路径的交点不足2个"
  - 返回 (None, None, None, None)
  - 不会生成采样点和覆盖路径

### 3. 采样点生成是否成功？
- **如果起点或终点为None**:
  - `generate_first_water_sampling_points` 返回空列表
  - 不会生成覆盖路径

### 4. TSP求解是否成功？
- **如果 LKH-3.exe 求解失败**:
  - `tour_order_raw = None`
  - 返回 (None, None, None, None)
  - 不会生成覆盖路径

### 5. 可视化条件
- **覆盖路径显示条件**:
  ```python
  if first_water_coverage_tour is not None and first_water_sampling_points is not None:
      _draw_first_water_coverage_path(...)
  ```
  - 需要 **同时** 满足两个条件才会绘制覆盖路径

---

## 当前配置下的预期行为

1. ✅ **会读取TSP数据**（如果文件存在）
2. ✅ **会尝试生成水域块1覆盖路径**（如果TSP数据可用）
3. ✅ **会生成覆盖路径可视化地图**（coverage_path_map.png）
4. ❌ **不会计算所有点对路径**（CALCULATE_ALL_PAIRS = False）
5. ❌ **不会调用LKH求解骨架路径TSP**（SOLVE_TSP_WITH_LKH = False）
6. ❌ **不会生成栅格化-骨架路径图**（SHOW_GRID_SKELETON_MAP = False）
7. ❌ **不会生成覆盖路径-骨架路径图**（SHOW_COVERAGE_SKELETON_MAP = False）

---

## 可能的问题诊断

### 问题：可视化中没有生成水域块1的覆盖路径

**可能原因**：

1. **TSP数据文件不存在**
   - 检查: `data/core_points_tsp_path.txt` 和 `data/core_points_path.txt` 是否存在
   - 解决: 需要先运行 `CALCULATE_ALL_PAIRS = True` 和 `SOLVE_TSP_WITH_LKH = True`

2. **骨架路径与水域边界交点不足2个**
   - 检查: 查看终端输出是否有 "[调试] 水域块X与骨架路径的交点数量: X"
   - 如果交点数量 < 2，会提前返回

3. **LKH-3.exe 求解失败**
   - 检查: 查看终端输出是否有 "✗ TSP求解失败"
   - 检查: `data/first_water_sampling_points.tour` 文件是否存在

4. **采样点生成失败**
   - 检查: 查看终端输出是否有 "✗ 起点或退出点未提供，无法生成采样点"

5. **可视化开关未打开**
   - 检查: `SHOW_COVERAGE_PATH_MAP = True`（当前已打开）
