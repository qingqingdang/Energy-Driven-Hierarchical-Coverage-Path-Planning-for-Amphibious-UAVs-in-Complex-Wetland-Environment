##V0.2 - 成本计算器（支持状态转换惩罚）
import numpy as np

class CostCalculator:
    def __init__(self, base_water_cost=1.0, land_cost=10.0, outside_boundary_cost=100.0, takeoff_penalty=10.0, landing_penalty=5.0):
        """
        初始化成本计算器
        :param base_water_cost: 基础水域成本（默认1.0）
        :param land_cost: 陆地成本（默认10.0）
        :param outside_boundary_cost: 边界外成本（默认100.0）
        :param takeoff_penalty: 从水域到陆地（1→0）的起飞惩罚成本（默认10.0）
        :param landing_penalty: 从陆地到水域（0→1）的降落惩罚成本（默认5.0）
        """
        self.base_water_cost = base_water_cost
        self.land_cost = land_cost
        self.outside_boundary_cost = outside_boundary_cost
        self.takeoff_penalty = takeoff_penalty
        self.landing_penalty = landing_penalty
        
    def calculate_grid_cost(self, grid_data, row, col):
        """
        计算单个栅格的通行成本
        :param grid_data: 栅格数据
        :param row, col: 栅格坐标
        :return: 成本值
        
        成本规则：
        - 水域栅格（蓝色，grid_type=1）：成本 = 1.0
        - 陆地栅格（白色，grid_type=0）：成本 = 10.0
        - 边界外栅格（红色，grid_type=-1）：成本 = 100.0
        """
        grid_type = grid_data['grid_type'][row, col]
        
        if grid_type == -1:  # 边界外（红色）
            return self.outside_boundary_cost
        elif grid_type == 0:  # 陆地（白色）
            return self.land_cost
        elif grid_type == 1:  # 水域（蓝色）
            return self.base_water_cost
        else:
            # 未知类型，返回极高成本
            return float('inf')
    
    def _calculate_water_cost(self, grid_data, row, col):
        """
        计算水域栅格的详细成本（高级功能，当前未使用）
        如果需要基于水深、流速等动态计算水域成本，可以在这里实现
        """
        base_cost = self.base_water_cost
        
        # 可选：添加更复杂的成本计算逻辑
        # 比如基于水深、流速、障碍物密度等
        # center = grid_data['coordinates'][row, col]
        # distance_from_center = np.linalg.norm(center)
        # distance_factor = 1.0 + distance_from_center * 0.001
        # return base_cost * distance_factor
        
        # 当前：返回固定的基础水域成本
        return base_cost
    
    def calculate_movement_cost(self, grid_data, from_pos, to_pos):
        """
        计算栅格间移动成本（用于A*算法）
        :param from_pos: 起始位置 (row, col)
        :param to_pos: 目标位置 (row, col)
        :return: 移动成本
        
        成本组成：
        1. 基础移动成本 = (from_cost + to_cost) / 2 × distance
        2. 转向成本（可选）
        3. 状态转换惩罚：水域→陆地(1→0)起飞惩罚；陆地→水域(0→1)降落惩罚
        """
        from_row, from_col = from_pos
        to_row, to_col = to_pos
        
        # 获取两个栅格的类型和成本
        from_type = grid_data['grid_type'][from_row, from_col]
        to_type = grid_data['grid_type'][to_row, to_col]
        from_cost = self.calculate_grid_cost(grid_data, from_row, from_col)
        to_cost = self.calculate_grid_cost(grid_data, to_row, to_col)
        
        # 如果任一栅格不可通行
        if from_cost == float('inf') or to_cost == float('inf'):
            return float('inf')
        
        # 计算距离成本
        distance = np.sqrt((to_row - from_row)**2 + (to_col - from_col)**2)
        
        # 计算转向成本（可选）
        turn_cost = self._calculate_turn_cost(from_pos, to_pos)
        
        # 计算基础移动成本
        base_cost = (from_cost + to_cost) / 2 * distance + turn_cost
        
        # 状态转换惩罚：水域→陆地 起飞惩罚；陆地→水域 降落惩罚
        transition_penalty = 0.0
        if from_type == 1 and to_type == 0:  # 水域(1) → 陆地(0)
            transition_penalty = self.takeoff_penalty
        elif from_type == 0 and to_type == 1:  # 陆地(0) → 水域(1)
            transition_penalty = self.landing_penalty
        
        # 综合成本
        total_cost = base_cost + transition_penalty
        
        return total_cost
    
    def _calculate_turn_cost(self, from_pos, to_pos):
        """计算转向成本（可选功能）"""
        # 这里可以实现转向成本计算
        # 暂时返回0
        return 0.0
    
    def update_cost_strategy(self, new_strategy):
        """更新成本计算策略"""
        # 这里可以实现动态成本调整
        pass
    
    def get_cost_matrix(self, grid_data):
        """获取整个栅格的成本矩阵"""
        rows, cols = grid_data['grid_type'].shape
        cost_matrix = np.zeros((rows, cols))
        
        for i in range(rows):
            for j in range(cols):
                cost_matrix[i, j] = self.calculate_grid_cost(grid_data, i, j)
        
        return cost_matrix
