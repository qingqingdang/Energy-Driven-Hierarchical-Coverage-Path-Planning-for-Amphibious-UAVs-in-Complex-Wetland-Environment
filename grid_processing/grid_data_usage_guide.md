# Grid Data 存储结构使用指南

## 📦 数据结构总览

`grid_data` 是一个字典，包含栅格地图的所有信息：

```python
grid_data = {
    # ========== 基础信息 ==========
    'coordinates': np.ndarray,    # shape: (rows, cols, 2) - 每个栅格的中心坐标 [x, y]
    'grid_size': float,           # 栅格大小（米），例如：4.0
    'bounds': dict,               # 边界范围信息
    'grid_id': np.ndarray,        # shape: (rows, cols) - 栅格唯一ID（0 到 N-1）
    
    # ========== 核心分类字段 ==========
    'grid_type': np.ndarray,      # shape: (rows, cols), dtype: int
                                  # -1: 边界外（红色）
                                  #  0: 陆地（白色）
                                  #  1: 水域（蓝色）
    
    # ========== 布尔标记字段（便于快速查询）==========
    'is_water': np.ndarray,              # shape: (rows, cols), dtype: bool - 是否为水域
    'is_land': np.ndarray,               # shape: (rows, cols), dtype: bool - 是否为陆地
    'is_outside_boundary': np.ndarray    # shape: (rows, cols), dtype: bool - 是否在边界外
}
```

## 🎯 使用方法

### 1. 获取栅格地图数据

```python
from grid_processing import GridGenerator

# 创建生成器
grid_generator = GridGenerator(grid_size=4.0, boundary_padding=50.0)

# 生成栅格
grid_data = grid_generator.generate_grid(env_data)
```

### 2. 查询单个栅格的类型

#### 方法1：使用 `grid_type` 字段（推荐）
```python
# 获取栅格类型
grid_type = grid_data['grid_type'][row, col]

if grid_type == -1:
    print("边界外栅格（红色）")
elif grid_type == 0:
    print("陆地栅格（白色）")
elif grid_type == 1:
    print("水域栅格（蓝色）")
```

#### 方法2：使用布尔字段
```python
# 快速判断
if grid_data['is_water'][row, col]:
    print("这是水域")

if grid_data['is_land'][row, col]:
    print("这是陆地")

if grid_data['is_outside_boundary'][row, col]:
    print("这在边界外")
```

#### 方法3：使用便捷方法
```python
# 使用 GridGenerator 的方法
grid_type_str = grid_generator.get_grid_type_by_position(row, col)
# 返回: 'water', 'land', 或 'outside_boundary'

# 或者使用单独的判断方法
is_water = grid_generator.is_grid_water(row, col)
is_land = grid_generator.is_grid_land(row, col)
is_outside = grid_generator.is_grid_outside_boundary(row, col)
```

### 3. 批量查询栅格

#### 获取所有水域栅格
```python
# 方法1：使用 numpy.where
water_indices = np.where(grid_data['is_water'])
water_grids = list(zip(water_indices[0], water_indices[1]))
# 结果：[(row1, col1), (row2, col2), ...]

# 方法2：使用 GridGenerator 方法
water_grids = grid_generator.get_water_grids()
```

#### 获取所有陆地栅格
```python
land_indices = np.where(grid_data['is_land'])
land_grids = list(zip(land_indices[0], land_indices[1]))
```

#### 获取所有边界外栅格
```python
boundary_indices = np.where(grid_data['is_outside_boundary'])
boundary_grids = list(zip(boundary_indices[0], boundary_indices[1]))
```

### 4. 统计信息

```python
# 获取统计信息
stats = grid_generator.get_grid_statistics()

print(f"总栅格数: {stats['total_grids']}")
print(f"水域栅格: {stats['water_grids']} ({stats['water_ratio']:.1%})")
print(f"陆地栅格: {stats['land_grids']} ({stats['land_ratio']:.1%})")
print(f"边界外栅格: {stats['outside_boundary_grids']}")
print(f"有效栅格（边界内）: {stats['valid_grids']}")
```

### 5. 获取栅格坐标

```python
# 获取特定栅格的中心坐标
center_x, center_y = grid_data['coordinates'][row, col]
print(f"栅格 [{row}, {col}] 的中心坐标: ({center_x:.2f}, {center_y:.2f})")

# 或使用方法
center = grid_generator.get_grid_center(row, col)
```

### 6. 遍历栅格

#### 遍历所有栅格
```python
rows, cols = grid_data['grid_type'].shape

for i in range(rows):
    for j in range(cols):
        grid_type = grid_data['grid_type'][i, j]
        center = grid_data['coordinates'][i, j]
        
        if grid_type == 1:  # 水域
            print(f"水域栅格 [{i}, {j}] at ({center[0]:.1f}, {center[1]:.1f})")
```

#### 仅遍历水域栅格
```python
water_grids = grid_generator.get_water_grids()
for row, col in water_grids:
    center = grid_data['coordinates'][row, col]
    print(f"水域栅格 [{row}, {col}] at ({center[0]:.1f}, {center[1]:.1f})")
```

## 🔍 与可视化的对应关系

可视化颜色与存储的对应：

| 存储值 | grid_type | 布尔字段 | 可视化颜色 | 说明 |
|--------|-----------|----------|------------|------|
| 边界外 | -1 | `is_outside_boundary = True` | 🔴 红色 | 超出边界范围 |
| 陆地 | 0 | `is_land = True` | ⚪ 白色 | 边界内的陆地 |
| 水域 | 1 | `is_water = True` | 🔵 蓝色 | 边界内的水域 |

## 💡 最佳实践

### 推荐的查询方式

1. **性能优先**：直接使用 `grid_data['grid_type']` 数组
   ```python
   # 最快的方式
   is_water_mask = (grid_data['grid_type'] == 1)
   water_count = np.sum(is_water_mask)
   ```

2. **可读性优先**：使用布尔字段
   ```python
   # 最清晰的方式
   if grid_data['is_water'][row, col]:
       # 处理水域栅格
   ```

3. **封装优先**：使用 GridGenerator 的方法
   ```python
   # 最安全的方式（有错误检查）
   if grid_generator.is_grid_water(row, col):
       # 处理水域栅格
   ```

### 常见使用场景

#### 场景1：过滤掉边界外的栅格
```python
# 获取所有边界内的栅格
valid_mask = (grid_data['grid_type'] != -1)
valid_grids = np.where(valid_mask)
```

#### 场景2：路径规划时检查栅格类型
```python
def can_pass(grid_data, row, col):
    """检查栅格是否可通行（水域可通行，陆地和边界外不可）"""
    return grid_data['is_water'][row, col]
```

#### 场景3：计算水域面积
```python
water_grid_count = np.sum(grid_data['is_water'])
grid_area = grid_data['grid_size'] ** 2
total_water_area = water_grid_count * grid_area
print(f"水域总面积: {total_water_area:.2f} 平方米")
```

## 📊 完整示例代码

```python
from OSM_ENVIRONMENTS import OSM_ENV
from extract_map import extract_env_cartesian
from grid_processing import GridGenerator

# 1. 加载环境
env = OSM_ENV(UAV=1)

# 2. 提取地图数据
env_data = extract_env_cartesian(env, shrink_dist=0.5, sample_dist=10)

# 3. 生成栅格
grid_generator = GridGenerator(grid_size=4.0, boundary_padding=50.0)
grid_data = grid_generator.generate_grid(env_data)

# 4. 查询栅格信息
rows, cols = grid_data['grid_type'].shape
print(f"栅格地图大小: {rows} x {cols}")

# 5. 统计信息
stats = grid_generator.get_grid_statistics()
print(f"\n栅格统计:")
print(f"  总数: {stats['total_grids']}")
print(f"  水域: {stats['water_grids']} ({stats['water_ratio']:.1%})")
print(f"  陆地: {stats['land_grids']} ({stats['land_ratio']:.1%})")
print(f"  边界外: {stats['outside_boundary_grids']}")

# 6. 查询特定栅格
test_row, test_col = 10, 10
grid_type = grid_generator.get_grid_type_by_position(test_row, test_col)
center = grid_data['coordinates'][test_row, test_col]
print(f"\n栅格 [{test_row}, {test_col}]:")
print(f"  类型: {grid_type}")
print(f"  中心坐标: ({center[0]:.2f}, {center[1]:.2f})")

# 7. 获取所有水域栅格
water_grids = grid_generator.get_water_grids()
print(f"\n水域栅格数量: {len(water_grids)}")
print(f"前5个水域栅格: {water_grids[:5]}")
```

## 🚀 性能提示

- **大规模查询**：优先使用 numpy 的向量化操作
- **单个查询**：使用布尔字段或方法调用
- **频繁访问**：可以提前缓存常用的掩码（mask）

```python
# 预先计算掩码（高效）
water_mask = grid_data['is_water']
land_mask = grid_data['is_land']
outside_mask = grid_data['is_outside_boundary']

# 后续快速查询
if water_mask[row, col]:
    # 处理水域
```




