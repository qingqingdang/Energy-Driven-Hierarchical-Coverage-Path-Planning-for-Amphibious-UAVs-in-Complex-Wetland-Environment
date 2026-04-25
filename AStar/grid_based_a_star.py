"""
基于栅格的A*路径规划算法 (V0.1)

基于现有的grid_data和cost_calculator实现A*算法，
用于计算核心点之间的最优路径。
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any


class GridBasedAStar:
    """基于栅格的A*路径规划器"""
    
    def __init__(self, grid_data: dict, cost_calculator):
        """
        初始化基于栅格的A*规划器
        
        :param grid_data: 栅格数据（来自GridGenerator.generate_grid）
        :param cost_calculator: 成本计算器（CostCalculator实例）
        """
        self.grid_data = grid_data
        self.cost_calculator = cost_calculator
        self.grid_size = grid_data['grid_size']
        self.coordinates = grid_data['coordinates']
        self.grid_type = grid_data['grid_type']
        
        # 获取栅格尺寸
        self.rows, self.cols = self.coordinates.shape[:2]
        
        # 获取边界范围
        self.bounds = grid_data['bounds']
        self.x_min = self.bounds['x_min']
        self.x_max = self.bounds['x_max']
        self.y_min = self.bounds['y_min']
        self.y_max = self.bounds['y_max']
        
        # 8方向移动模型（上下左右 + 4个对角线）
        self.motion = self._get_motion_model()
    
    class Node:
        """A*节点"""
        def __init__(self, row: int, col: int, g_cost: float, parent: Optional['Node'] = None):
            self.row = row
            self.col = col
            self.g_cost = g_cost  # 从起点到当前节点的实际成本
            self.h_cost = 0.0     # 从当前节点到终点的启发式估计
            self.f_cost = 0.0     # f = g + h
            self.parent = parent
        
        def __eq__(self, other):
            return self.row == other.row and self.col == other.col
        
        def __hash__(self):
            return hash((self.row, self.col))
    
    def _get_motion_model(self) -> List[Tuple[int, int, float]]:
        """
        获取移动模型
        返回: [(dx, dy, base_cost), ...]
        """
        # 8方向移动：上下左右 + 对角线
        # base_cost用于基础距离，实际成本会乘以栅格成本
        return [
            (0, 1, 1.0),      # 上
            (0, -1, 1.0),     # 下
            (1, 0, 1.0),      # 右
            (-1, 0, 1.0),     # 左
            (1, 1, math.sqrt(2)),   # 右上
            (1, -1, math.sqrt(2)),  # 右下
            (-1, 1, math.sqrt(2)),  # 左上
            (-1, -1, math.sqrt(2)), # 左下
        ]
    
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        将世界坐标转换为栅格索引
        
        :param x: 世界坐标x（米）
        :param y: 世界坐标y（米）
        :return: (row, col) 或 None（如果超出范围）
        """
        # 计算栅格索引
        col = int((x - self.x_min) / self.grid_size)
        row = int((y - self.y_min) / self.grid_size)
        
        # 检查是否在有效范围内
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (row, col)
        return None
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        将栅格索引转换为世界坐标
        
        :param row: 栅格行索引
        :param col: 栅格列索引
        :return: (x, y) 世界坐标
        """
        return (self.coordinates[row, col, 0], self.coordinates[row, col, 1])
    
    def _heuristic(self, node1: Node, node2: Node) -> float:
        """
        计算启发式成本（欧几里得距离）
        
        :param node1: 节点1
        :param node2: 节点2
        :return: 启发式成本
        """
        dx = node1.col - node2.col
        dy = node1.row - node2.row
        # 使用欧几里得距离作为启发式
        return math.sqrt(dx * dx + dy * dy)
    
    def _is_valid_node(self, row: int, col: int) -> bool:
        """
        检查节点是否有效（在范围内且可通行）
        
        :param row: 栅格行索引
        :param col: 栅格列索引
        :return: True如果节点有效
        """
        # 检查边界
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        
        # 检查是否在边界外（grid_type == -1 表示不可通行）
        if self.grid_type[row, col] == -1:
            return False
        
        return True
    
    def _get_movement_cost(self, from_row: int, from_col: int, 
                          to_row: int, to_col: int) -> float:
        """
        计算从from到to的移动成本
        
        :param from_row, from_col: 起始栅格
        :param to_row, to_col: 目标栅格
        :return: 移动成本
        """
        # 使用cost_calculator计算移动成本
        from_pos = (from_row, from_col)
        to_pos = (to_row, to_col)
        return self.cost_calculator.calculate_movement_cost(self.grid_data, from_pos, to_pos)
    
    def plan(self, start_x: float, start_y: float, 
             goal_x: float, goal_y: float) -> Dict[str, Any]:
        """
        使用A*算法规划路径
        
        :param start_x, start_y: 起点世界坐标
        :param goal_x, goal_y: 终点世界坐标
        :return: 路径结果字典
        """
        # 转换为栅格坐标
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        if start_grid is None:
            return {
                'success': False,
                'error': f'起点 ({start_x}, {start_y}) 超出栅格范围',
                'path': [],
                'cost': float('inf'),
                'distance': 0.0
            }
        
        if goal_grid is None:
            return {
                'success': False,
                'error': f'终点 ({goal_x}, {goal_y}) 超出栅格范围',
                'path': [],
                'cost': float('inf'),
                'distance': 0.0
            }
        
        start_row, start_col = start_grid
        goal_row, goal_col = goal_grid
        
        # 检查起点和终点是否可通行
        if not self._is_valid_node(start_row, start_col):
            return {
                'success': False,
                'error': f'起点栅格 ({start_row}, {start_col}) 不可通行',
                'path': [],
                'cost': float('inf'),
                'distance': 0.0
            }
        
        if not self._is_valid_node(goal_row, goal_col):
            return {
                'success': False,
                'error': f'终点栅格 ({goal_row}, {goal_col}) 不可通行',
                'path': [],
                'cost': float('inf'),
                'distance': 0.0
            }
        
        # 初始化起点节点
        start_node = self.Node(start_row, start_col, 0.0, None)
        goal_node = self.Node(goal_row, goal_col, 0.0, None)
        start_node.h_cost = self._heuristic(start_node, goal_node)
        start_node.f_cost = start_node.g_cost + start_node.h_cost
        
        # 开放集和关闭集
        open_set = {start_node}
        closed_set = set()
        
        # 用于快速查找的字典
        open_dict = {(start_row, start_col): start_node}
        
        while open_set:
            # 选择f_cost最小的节点
            current = min(open_set, key=lambda n: n.f_cost)
            
            # 如果到达目标
            if current.row == goal_node.row and current.col == goal_node.col:
                # 重构路径
                path_world = []
                path_grid = []
                total_cost = current.g_cost
                
                node = current
                while node is not None:
                    x, y = self.grid_to_world(node.row, node.col)
                    path_world.append([x, y])
                    path_grid.append((node.row, node.col))
                    node = node.parent
                
                path_world.reverse()
                path_grid.reverse()
                
                # 计算实际距离
                total_distance = 0.0
                for i in range(len(path_world) - 1):
                    dx = path_world[i+1][0] - path_world[i][0]
                    dy = path_world[i+1][1] - path_world[i][1]
                    total_distance += math.sqrt(dx*dx + dy*dy)
                
                return {
                    'success': True,
                    'path': path_world,
                    'grid_path': path_grid,
                    'cost': total_cost,
                    'distance': total_distance,
                    'start': [start_x, start_y],
                    'goal': [goal_x, goal_y],
                    'start_grid': start_grid,
                    'goal_grid': goal_grid
                }
            
            # 从开放集移除，加入关闭集
            open_set.remove(current)
            closed_set.add((current.row, current.col))
            del open_dict[(current.row, current.col)]
            
            # 扩展邻居节点
            for dx, dy, base_cost in self.motion:
                new_row = current.row + dy
                new_col = current.col + dx
                
                # 检查是否有效
                if not self._is_valid_node(new_row, new_col):
                    continue
                
                # 检查是否在关闭集中
                if (new_row, new_col) in closed_set:
                    continue
                
                # 计算移动成本
                movement_cost = self._get_movement_cost(
                    current.row, current.col, new_row, new_col
                )
                
                if movement_cost == float('inf'):
                    continue
                
                # 计算新的g_cost
                new_g_cost = current.g_cost + movement_cost
                
                # 检查是否在开放集中
                if (new_row, new_col) in open_dict:
                    existing_node = open_dict[(new_row, new_col)]
                    if new_g_cost < existing_node.g_cost:
                        # 找到更好的路径，更新节点
                        existing_node.g_cost = new_g_cost
                        existing_node.h_cost = self._heuristic(existing_node, goal_node)
                        existing_node.f_cost = existing_node.g_cost + existing_node.h_cost
                        existing_node.parent = current
                else:
                    # 新节点
                    new_node = self.Node(new_row, new_col, new_g_cost, current)
                    new_node.h_cost = self._heuristic(new_node, goal_node)
                    new_node.f_cost = new_node.g_cost + new_node.h_cost
                    open_set.add(new_node)
                    open_dict[(new_row, new_col)] = new_node
        
        # 未找到路径
        return {
            'success': False,
            'error': '未找到路径',
            'path': [],
            'cost': float('inf'),
            'distance': 0.0
        }


__all__ = ['GridBasedAStar']

