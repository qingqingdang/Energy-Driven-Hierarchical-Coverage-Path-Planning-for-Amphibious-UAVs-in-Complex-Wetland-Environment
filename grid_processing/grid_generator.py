##V0.1 - 栅格生成器
import numpy as np
from shapely.geometry import Point, Polygon

class GridGenerator:
    def __init__(self, grid_size=10.0, boundary_padding=50.0):
        """
        初始化栅格生成器
        :param grid_size: 栅格单元大小（米）
        :param boundary_padding: 边界填充距离（米）
        """
        self.grid_size = grid_size
        self.boundary_padding = boundary_padding
        self.grid_data = None
        self.grid_bounds = None
        
    def generate_grid(self, env_data):
        """
        基于地图提取结果生成栅格
        :param env_data: 来自extract_map.py的结果
        :return: 栅格数据字典
        """
        # 获取边界和水域信息
        boundary_polygon = env_data["boundary_polygon"]
        water_with_hole = env_data["water_with_hole"]
        water_no_hole = env_data["water_no_hole"]
        
        # 计算栅格边界
        self.grid_bounds = self._calculate_grid_bounds(boundary_polygon)
        
        # 生成栅格坐标
        grid_coords = self._generate_grid_coordinates()
        
        # 为每个栅格分配类型和属性
        grid_data = self._assign_grid_properties(
            grid_coords, water_with_hole, water_no_hole, boundary_polygon
        )
        
        self.grid_data = grid_data
        return grid_data
    
    def _calculate_grid_bounds(self, boundary_polygon):
        """计算栅格边界"""
        x_min = boundary_polygon[:, 0].min() - self.boundary_padding
        x_max = boundary_polygon[:, 0].max() + self.boundary_padding
        y_min = boundary_polygon[:, 1].min() - self.boundary_padding
        y_max = boundary_polygon[:, 1].max() + self.boundary_padding
        
        return {
            'x_min': x_min, 'x_max': x_max,
            'y_min': y_min, 'y_max': y_max,
            'width': x_max - x_min,
            'height': y_max - y_min
        }
    
    def _generate_grid_coordinates(self):
        """生成栅格坐标矩阵"""
        x_coords = np.arange(
            self.grid_bounds['x_min'],
            self.grid_bounds['x_max'] + self.grid_size,
            self.grid_size
        )
        y_coords = np.arange(
            self.grid_bounds['y_min'],
            self.grid_bounds['y_max'] + self.grid_size,
            self.grid_size
        )
        
        X, Y = np.meshgrid(x_coords, y_coords)
        return np.stack([X, Y], axis=-1)
    
    def _assign_grid_properties(self, grid_coords, water_with_hole, water_no_hole, boundary_polygon):
        """为栅格分配属性"""
        rows, cols = grid_coords.shape[:2]
        grid_data = {
            # 基础信息
            'coordinates': grid_coords,                              # 栅格中心坐标
            'grid_size': self.grid_size,                            # 栅格大小
            'bounds': self.grid_bounds,                             # 边界范围
            'grid_id': np.arange(rows * cols).reshape(rows, cols),  # 栅格唯一ID
            
            # 核心分类字段（主要使用）
            'grid_type': np.zeros((rows, cols), dtype=int),         # -1: 边界外, 0: 陆地, 1: 水域
            
            # 布尔标记字段（便于快速查询）
            'is_water': np.zeros((rows, cols), dtype=bool),         # 是否为水域
            'is_land': np.zeros((rows, cols), dtype=bool),          # 是否为陆地
            'is_outside_boundary': np.zeros((rows, cols), dtype=bool)  # 是否在边界外
        }
        
        # 检查每个栅格的类型
        for i in range(rows):
            for j in range(cols):
                center_point = Point(grid_coords[i, j])
                
                # 检查是否在边界内
                if not Polygon(boundary_polygon).contains(center_point):
                    grid_data['grid_type'][i, j] = -1  # 边界外
                    grid_data['is_outside_boundary'][i, j] = True
                    continue
                
                # 检查是否在水域内
                is_in_water = self._check_water_containment(
                    center_point, water_with_hole, water_no_hole
                )
                
                if is_in_water:
                    grid_data['grid_type'][i, j] = 1
                    grid_data['is_water'][i, j] = True
                else:
                    grid_data['grid_type'][i, j] = 0
                    grid_data['is_land'][i, j] = True
        
        return grid_data
    
    def _check_water_containment(self, point, water_with_hole, water_no_hole):
        """检查点是否在水域内（排除孔洞）"""
        for water in water_with_hole + water_no_hole:
            main_poly = Polygon(water["outer"])
            if main_poly.contains(point):
                # 检查是否在孔洞内
                in_hole = False
                for hole in water["holes"]:
                    if Polygon(hole).contains(point):
                        in_hole = True
                        break
                # 如果在外轮廓内但不在孔洞内，则是水域
                if not in_hole:
                    return True
        return False
    
    def get_grid_center(self, row, col):
        """获取栅格中心坐标"""
        return self.grid_data['coordinates'][row, col]
    
    def get_grid_neighbors(self, row, col):
        """获取栅格的相邻栅格"""
        neighbors = []
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                if di == 0 and dj == 0:
                    continue
                new_row, new_col = row + di, col + dj
                if (0 <= new_row < self.grid_data['coordinates'].shape[0] and
                    0 <= new_col < self.grid_data['coordinates'].shape[1]):
                    neighbors.append((new_row, new_col))
        return neighbors
    
    def get_water_grids(self):
        """获取所有水域栅格的坐标"""
        water_indices = np.where(self.grid_data['is_water'])
        return list(zip(water_indices[0], water_indices[1]))
    
    def get_grid_statistics(self):
        """获取栅格统计信息"""
        total_grids = self.grid_data['grid_type'].size
        water_grids = np.sum(self.grid_data['is_water'])
        land_grids = np.sum(self.grid_data['is_land'])
        boundary_grids = np.sum(self.grid_data['is_outside_boundary'])
        valid_grids = water_grids + land_grids  # 边界内的栅格
        
        return {
            'total_grids': total_grids,
            'water_grids': int(water_grids),
            'land_grids': int(land_grids),
            'outside_boundary_grids': int(boundary_grids),
            'valid_grids': int(valid_grids),
            'water_ratio': water_grids / valid_grids if valid_grids > 0 else 0,
            'land_ratio': land_grids / valid_grids if valid_grids > 0 else 0
        }
    
    def get_grid_type_by_position(self, row, col):
        """
        根据栅格位置获取类型
        :param row: 行索引
        :param col: 列索引
        :return: 'water', 'land', 或 'outside_boundary'
        """
        if self.grid_data is None:
            raise ValueError("Grid data not generated yet. Call generate_grid() first.")
        
        grid_type = self.grid_data['grid_type'][row, col]
        type_map = {-1: 'outside_boundary', 0: 'land', 1: 'water'}
        return type_map.get(grid_type, 'unknown')
    
    def is_grid_water(self, row, col):
        """检查指定栅格是否为水域"""
        return self.grid_data['is_water'][row, col] if self.grid_data else False
    
    def is_grid_land(self, row, col):
        """检查指定栅格是否为陆地"""
        return self.grid_data['is_land'][row, col] if self.grid_data else False
    
    def is_grid_outside_boundary(self, row, col):
        """检查指定栅格是否在边界外"""
        return self.grid_data['is_outside_boundary'][row, col] if self.grid_data else False
