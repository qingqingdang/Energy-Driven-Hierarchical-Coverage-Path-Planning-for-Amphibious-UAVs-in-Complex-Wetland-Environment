# monitoring_algorithmsimport numpy as np
# import os
# import matplotlib.pyplot as plt
# from pyDOE import lhs
# from shapely.geometry import Point, Polygon
# from sklearn.cluster import KMeans
# from sklearn.cluster import MiniBatchKMeans
# import sklearn.neighbors as skn
# from math import sqrt, tan, radians
# import latlongcartconv as lc
# import copy
# import time
# import pickle
# import networkx as nx
# from shapely.geometry import LineString
# #import mlrose              #use this only if the planner is TSP. 
# import SCoPP_settings
# import random


# class QLB:
#     """
#         Base class for the Quick, Load-Balanced (SCoPP) Monitoring algorithm.
#         This class's attributes contain details of the partitions created. See "Class Attributes" below for a list of
#         other attributes attributes.

#         Parameters
#         ----------
#         number_of_robots: int - number of robots intended to survey the area
#         environment: class object - environment object which contains information pertaining to the environment. See
#             "environments.py" for details.
#         plot (optional): string or tuple of strings - choose between plotting none, all, or the final phase(s) of the
#             algorithm, by passing either no argument, "full", or "partial", respectively. Note that choosing the "full"
#             plot option will increase computation time. Additionally, the user may specify whether or not to display the
#             coverage over time plot using "area".
#                 Example: To see only the final plot and area surveyed per second, the argument passed would be
#                 ("partial", "area")
#         plot_settings (optional): "plots" class object - specify the size of points, lines, etc. of the plot(s). See
#             the "plots" class in "SCoPP_settings.py" for details.
#         algorithm_settings (optional): "algorithm" class object - "algorithm" class object - specify the algorithm
#             settings. See the "algorithm" class in "SCoPP_settings.py" for details.

#         Useful attributes
#         ----------
#         area_covered_over_time: list - area surveyed as a function of time. Use for plotting and visualization of
#             results

#         Returns
#         ----------
#         None: Initializes SCoPP environment instance. To run the algorithm, the user must call the "run" method. See "run"
#         below for details.
#     """

#     def __init__(
#             self,
#             environment,
#             number_of_robots=None,
#             plot=True,
#             plot_settings=SCoPP_settings.plots(),
#             algorithm_settings=SCoPP_settings.algorithm()
#     ):
#         # Initialize environment parameters
#         if len(environment.starting_position) > 1:
#             geographical_starting_positions = environment.starting_position
#         else:
#             geographical_starting_positions = []
#             for agent in range(number_of_robots):
#                 geographical_starting_positions.extend(environment.starting_position)
#         geographical_boundary_points = environment.boundary_points
#         self.robot_FOV = environment.robot_FOV
#         self.robot_operating_height = environment.robot_operating_height
#         self.robot_velocity = environment.robot_velocity
#         geographical_geo_fencing_zones = environment.geo_fencing_holes
#         geographical_water_fencing_zones = environment.water_terrain
#         self.save_path = environment.save_path
#         self.water_velocity = environment.water_velocity if hasattr(environment, 'water_velocity') else self.robot_velocity
#         self.water_cells_x = []  # 存储水域单元格中心坐标
#         self.water_cells_y = []        
#         self.water_bias_factor = 1


#         # Initialize reference frame
#         self.robot_initial_positions_in_cartesian = [] # 初始化机器人初始位置的笛卡尔坐标
#         self.boundary_points_in_cartesian = [] # 边界点的笛卡尔坐标
#         self.boundary_points_in_geographical = [] # 边界点的地理坐标
#         self.robot_initial_positions_in_geographical = [] # 机器人初始位置的地理坐标

#         # 计算所有起始位置和边界点中的最小经纬度，并将其作为坐标系的原点
#         origin = [0, 0]
#         origin[1] = min(np.concatenate(
#             (np.transpose(geographical_starting_positions)[0], np.transpose(geographical_boundary_points)[0])))
#         origin[0] = min(np.concatenate(
#             (np.transpose(geographical_starting_positions)[1], np.transpose(geographical_boundary_points)[1])))
#         self.origin = origin

#         # 处理电子围栏区域（geo_fencing_holes）# 示例：若有3个围栏区域 → 生成 [[], [], []]
#         if geographical_geo_fencing_zones:
#             self.geographical_geo_fencing_zones_in_cartesian = [[] for item in
#                                                                 range(len(geographical_geo_fencing_zones))]
#             self.geographical_geo_fencing_zones_in_geographical = [[] for item in
#                                                                    range(len(geographical_geo_fencing_zones))]
#         else:
#             self.geographical_geo_fencing_zones_in_cartesian = None
#             self.geographical_geo_fencing_zones_in_geographical = None

#         # 处理水域区域（water_terrain）# 示例：若有3个水域区域 → 生成 [[], [], []]
#         if geographical_water_fencing_zones:
#             self.geographical_water_fencing_zones_in_cartesian = [[] for item in range(len(geographical_water_fencing_zones))]
#             self.geographical_water_fencing_zones_in_geographical = [[] for item in range(len(geographical_water_fencing_zones))]

#         else:
#             self.geographical_water_fencing_zones_in_cartesian = None
#             self.geographical_water_fencing_zones_in_geographical = None

#         # 将机器人的初始位置转换为将其转换为前经后纬
#         for robot, position in enumerate(geographical_starting_positions):
#             self.robot_initial_positions_in_geographical.append([position[1], position[0]])

#         # 将环境的边界点转换为地理坐标将其转换为前经后纬
#         for point in geographical_boundary_points:
#             self.boundary_points_in_geographical.append([point[1], point[0]])

#         # 处理电子围栏区域边界点，将其转换为地理坐标将其转换为前经后纬
#         if geographical_geo_fencing_zones:
#             for list_id, fence_list in enumerate(geographical_geo_fencing_zones):
#                 for point in fence_list:
#                     self.geographical_geo_fencing_zones_in_geographical[list_id].append([point[1], point[0]])

#         # 处理水域区域，将其转换为前经后纬
#         if geographical_water_fencing_zones:
#             for list_id, water_list in enumerate(geographical_water_fencing_zones):
#                 for point in water_list:
#                     self.geographical_water_fencing_zones_in_geographical[list_id].append([point[1], point[0]])

        

#         # Initialize method variables
#         self.optimal_paths = None
#         if number_of_robots is None:
#             self.number_of_partitions = len(environment.starting_position)
#         else:
#             self.number_of_partitions = number_of_robots
#         self.dominated_cells_x = [[] for robot in range(self.number_of_partitions)]
#         self.dominated_cells_y = [[] for robot in range(self.number_of_partitions)]
#         self.robot_WOV = tan(radians(self.robot_FOV / 2)) * self.robot_operating_height * 2  # in meters
#         self.cell_size = int(self.robot_WOV)
#         self.cell_wall_interval = 3
#         if self.cell_size % 2 != 0:
#             self.cell_size -= 1
#         self.cell_area = self.cell_size ** 2  # in meters squared
#         self.robot_assignment_information = None

#         # Algorithm settings
#         self.bias_factor = algorithm_settings.bias_factor
#         self.sampling_rate = algorithm_settings.sampling_rate
#         self.leaf_size = algorithm_settings.leaf_size
#         self.conflict_resolution_mode = algorithm_settings.conflict_resolution_mode
#         self.planner = algorithm_settings.planner
#         if algorithm_settings.partition_tolerance is None:
#             self.partition_tolerance = 1 / 8 * self.cell_size
#         else:
#             self.partition_tolerance = algorithm_settings.partition_tolerance
#         self.partition_max_iter = algorithm_settings.partition_max_iter

#         # Initialize computation time attributes
#         self.time_for_conversion = 0
#         self.time_for_partitioning = 0
#         self.time_for_discretization = 0
#         self.time_for_conflict_resolution = 0
#         self.time_for_path_planning = 0
#         self.total_time = 0

#         # Initialize run data attributes for statistics
#         self.data_tasks_per_robot = None
#         self.data_distance_travelled_per_robot = None
#         self.data_completion_time_per_robot = None
#         self.data_total_mission_completion_time = None
#         self.data_computation_time_without_path_planning = None
#         self.data_computation_time_with_path_planning = None

#         # Initialize variables for plotting
#         self.rough_partitioning_x = None
#         self.rough_partitioning_y = None
#         self.cluster_centers = None
#         self.conflict_cells_x = None
#         self.conflict_cells_y = None
#         self.final_partitioning_x = None
#         self.final_partitioning_y = None
#         self.data_tasks_per_robot = None

#         # Plot settings
#         self.plot = plot
#         self.plot_cell_boundary_size = plot_settings.cell_boundary
#         self.plot_robot_path_size = plot_settings.robot_path_size
#         self.plot_robot_size = plot_settings.robot_size
#         self.plot_cell_size = plot_settings.cell_size
#         self.partition_colors = None

#         # Initialize other useful information
#         self.total_survey_area = None
#         self.area_per_robot = None
#         self.area_covered_over_time = []
#         self.area_covered_over_time_time_vector = []

#     def run(self, info=False):
#         """
#         Runs the SCoPP-Monitoring algorithm in its entirety.
#         Parameters
#         ----------
#         info (optional): string, tuple of strings - choose what kind of information of the algorithm results to display
#         in the run window. Takes any of the following strings as inputs:
#             "verbose" - display all available relevant information
#             "time" - display computation times per phase of the algorithm
#             "mission" - display information pertaining to the performance of the resulting paths determined for the
#                 robots
#             "area" - display area allocation information
#             ""
#         Returns
#         ----------
#         final_paths_in_geographical: list of lists of tuples - the final, ordered list of waypoints for each robot,
#             beginning at their starting position.
#         """
#         t0 = time.time()
#     # 将原点的经纬度转换为笛卡尔坐标，基于此建立局部坐标系，以此来进行坐标转换，转换机器人初始位置、环境边界点、围栏区域、水域区域
#         time_stamp = time.time()
#         coordinate_converter = lc.LLCCV(self.origin) #设置一个局部坐标转换器，以origin为原点

#         for item in self.robot_initial_positions_in_geographical:#将k个机器人的初始位置的经纬度坐标转换为笛卡尔坐标
#             self.robot_initial_positions_in_cartesian.append(coordinate_converter.get_cartesian(item))
#         for item in self.boundary_points_in_geographical:   #将环境的边界点的经纬度坐标转换为笛卡尔坐标
#             self.boundary_points_in_cartesian.append(coordinate_converter.get_cartesian(item))

#         # 首先检查是否有地理围栏，将围栏的每个点从地理坐标转换为笛卡尔坐标
#         if self.geographical_geo_fencing_zones_in_geographical:# 检查是否有电子围栏数据
#             for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones_in_geographical):# 遍历每个围栏
#                 for item in fence_list:
#                     self.geographical_geo_fencing_zones_in_cartesian[list_id].append(
#                         coordinate_converter.get_cartesian(item))
        
#         # 处理水域区域（水域区域的转换）
#         if self.geographical_water_fencing_zones_in_geographical:
#             # 遍历水域区域并将每个点的经纬度坐标转换为笛卡尔坐标
#             for list_id, fence_list in enumerate(self.geographical_water_fencing_zones_in_geographical):
#                 for item in fence_list:
#                     # 将每个水域区域的经纬度坐标转换为笛卡尔坐标并存储
#                     self.geographical_water_fencing_zones_in_cartesian[list_id].append(coordinate_converter.get_cartesian(item))

#         self.time_for_conversion += time.time() - time_stamp

#         # 根据地图的边界和指定的分辨率（例如单元格大小）将地图离散化为一个网格
#         time_stamp = time.time()
#         cell_space, cells = self.discretize_massive_area()

#         # if hasattr(self, 'water_cells_x') and self.water_cells_x:
#         #     print(f"find{len(self.water_cells_x)}watergrid")
#         #     # 进行水域相关处理
#         #     for x, y in zip(self.water_cells_x, self.water_cells_y):
#         #         print(f"watergrid: ({x}, {y})")
#         # else:
#         #     print("未发现水域单元格")        

#         self.time_for_discretization += time.time() - time_stamp

#         # 函数会将离散化后的区域进行划分，确保每个机器人都能覆盖一部分区域
#         time_stamp = time.time()
#         cells_as_dict, cell_space_as_dict, robot_assignment_information = self.partition_area(cell_space, cells)
#         self.time_for_partitioning += time.time() - time_stamp

#         # 检查机器人之间是否有区域重叠或任务冲突
#         time_stamp = time.time()
#         robot_assignment_information, waypoints_for_robots = \
#             self.resolve_conflicts(cells_as_dict, cell_space_as_dict, robot_assignment_information,
#                                    self.conflict_resolution_mode)
#         self.time_for_conflict_resolution += time.time() - time_stamp

#         # 调用 find_optimal_paths() 方法来计算每个机器人从其分配的起点到目标点的最优路径
#         timestamp = time.time()
#         paths, distances = self.find_optimal_paths(self.planner, robot_assignment_information, waypoints_for_robots)
#         self.time_for_path_planning = time.time() - timestamp

#         # 计算随时间变化的区域覆盖率和任务完成时间
#         self.calculate_surveillance_rate(distances)

#         # 将路径从笛卡尔坐标系转换为地理坐标系
#         time_stamp = time.time()
#         final_paths_in_geographical = []
#         for path in paths:
#             entry = []
#             for item in path:
#                 entry.append(coordinate_converter.get_geographic(item))
#             final_paths_in_geographical.append(entry)
#         self.time_for_conversion += time.time() - time_stamp
#         self.total_time = time.time() - t0  #这行代码计算整个算法执行过程的总时间

#         self.area_per_robot = np.array(self.data_tasks_per_robot) * self.cell_area #
        

#         # Display algorithm attributes
#         if info:
#             if info == "verbose":
#                 # Display computation time attributes
#                 print("Computation times for various operations (in seconds):")
#                 print("     Conversions:", self.time_for_conversion)
#                 print("     Initial partitioning:", self.time_for_partitioning)
#                 print("     Discretization:", self.time_for_discretization)
#                 print("     Conflict resolution:", self.time_for_conflict_resolution)
#                 print("     Path planning:", self.time_for_path_planning)
#                 print("     Entire algorithm:", self.total_time)
#                 # Display data attributes for statistics
#                 print("Statistics:")
#                 print("     Tasks per robot:", self.data_tasks_per_robot)
#                 print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
#                 print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
#                 print("     Mission completion time of the swarm:", self.data_total_mission_completion_time, "seconds")
#                 # Display other environment information
#                 print("Area allocation information:")
#                 print("     Cell width:", self.cell_size, "meters")
#                 print("     Cell area:", self.cell_area, "square meters")
#                 print("     Total area to be surveyed:", self.total_survey_area, "square meters")
#                 print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")

#             else:
#                 if info == "time":
#                     # Display computation time attributes
#                     print("Computation times for various operations (in seconds):")
#                     print("     Conversions:", self.time_for_conversion)
#                     print("     Initial partitioning:", self.time_for_partitioning)
#                     print("     Discretization:", self.time_for_discretization)
#                     print("     Conflict resolution:", self.time_for_conflict_resolution)
#                     print("     Path planning:", self.time_for_path_planning)
#                     print("     Entire algorithm:", self.total_time)

#                 if info == "mission":
#                     # Display data attributes for statistics
#                     print("Statistics:")
#                     print("     Tasks per robot:", self.data_tasks_per_robot)
#                     print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
#                     print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
#                     print("     Mission completion time of the swarm:", self.data_total_mission_completion_time,
#                           "seconds")

#                 if info == "area":
#                     # Display other environment information
#                     print("Area allocation information:")
#                     print("     Cell width:", self.cell_size, "meters")
#                     print("     Cell area:", self.cell_area, "square meters")
#                     print("     Total area to be surveyed:", self.total_survey_area, "square meters")
#                     print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")
#         # Plot results
#         if self.plot:
#             self.plot_partitioning()

#         # Return final list of paths per robot
#         return final_paths_in_geographical

#     def discretize_area(self):
#         """Discretization phase: distretizes the surveyable area and finds cells which lie within the bounds specified
#         by the environment parameters
#         """
#         x_max = -np.inf
#         y_max = -np.inf
#         for item in self.robot_initial_positions_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
#         for item in self.boundary_points_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
#         if self.geographical_geo_fencing_zones_in_cartesian:
#             boundary_polygon = Polygon(
#                 self.boundary_points_in_cartesian,
#                 holes=self.geographical_geo_fencing_zones_in_cartesian
#             )
#         else:
#             boundary_polygon = Polygon(self.boundary_points_in_cartesian)
#         self.total_survey_area = boundary_polygon.area
#         print("Total Survey Area:", self.total_survey_area)

#         # Create initial grid zone
#         grid_space = np.zeros((y_max, x_max)) + -np.inf
#         column_1 = 0
#         column_2 = column_1 + self.cell_size
#         row_1 = 0
#         row_2 = row_1 + self.cell_size
#         cells_within_boundary_x = []
#         cells_within_boundary_y = []
#         while True:
#             cell_boundary = []
#             for i in range(self.cell_size + 1):
#                 for j in range(self.cell_size + 1):
#                     if Point((i + row_1, j + column_1)).within(boundary_polygon):
#                         if i == 0 or i == self.cell_size or j == 0 or j == self.cell_size:
#                             cell_boundary.append((i + row_1, j + column_1))
#             if len(cell_boundary) == self.cell_size * 4:
#                 cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
#                 for count in range(len(cell_boundary_points_x)):
#                     grid_space[cell_boundary_points_x[count], cell_boundary_points_y[count]] = -1
#                     cells_within_boundary_x.append(cell_boundary_points_x[count])
#                     cells_within_boundary_y.append(cell_boundary_points_y[count])
#             column_1 = int(column_1 + self.cell_size)
#             column_2 = int(column_2 + self.cell_size)
#             if column_2 >= len(grid_space[0]):
#                 row_1 = int(row_1 + self.cell_size)
#                 row_2 = int(row_2 + self.cell_size)
#                 column_1 = 0
#                 column_2 = column_1 + self.cell_size
#             if row_2 >= len(grid_space):
#                 break
#         return [cells_within_boundary_x, cells_within_boundary_y], grid_space

#     def discretize_massive_area(self):
#         """Discretization phase: distretizes the surveyable area and finds cells which lie within the bounds specified
#         by the environment parameters
#         """
#         x_max = -np.inf
#         y_max = -np.inf
#         # 通过轮询，找到地图边界点和无人机放置位置中最大的
#         for item in self.robot_initial_positions_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
#         for item in self.boundary_points_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
        
#         # 创建水域多边形
#         water_polygons = []
#         if self.geographical_water_fencing_zones_in_cartesian:
#             for water_zone in self.geographical_water_fencing_zones_in_cartesian:
#                 water_polygons.append(Polygon(water_zone))
        
#         # 创建包含电子围栏孔洞的勘测区域多边形
#         if self.geographical_geo_fencing_zones_in_cartesian:
#             boundary_polygon = Polygon( 
#                 self.boundary_points_in_cartesian,
#                 holes=self.geographical_geo_fencing_zones_in_cartesian
#             )
#         else:
#             boundary_polygon = Polygon(self.boundary_points_in_cartesian)
#         self.total_survey_area = boundary_polygon.area
#         print("Total survey area:", self.total_survey_area)

#         # 创建网格地图
#         cells = np.zeros((int(x_max / self.cell_size), int(y_max / self.cell_size))) + -np.inf
#         cell_squares_within_boundary_x = []
#         cell_squares_within_boundary_y = []
#         water_cells_x = []  # 新增：存储水域单元格中心在网格内的x坐标
#         water_cells_y = []  # 新增：存储水域单元格中心在网格内的y坐标
        
#         for row in range(len(cells)):
#             for column in range(len(cells[0])):
#                 cell_boundary = []
#                 is_water_cell = False  # 新增：标记是否为水域单元格
                
#                 for i in range(self.cell_size + 1):
#                     if i % self.cell_wall_interval == 0 or i == self.cell_size:
#                         for j in range(self.cell_size + 1):
#                             if j % self.cell_wall_interval == 0 or j == self.cell_size:
#                                 point = Point((i + column * self.cell_size, j + row * self.cell_size))
#                                 if point.within(boundary_polygon):
#                                     # 新增：检查是否在水域内
#                                     if water_polygons:
#                                         for water_poly in water_polygons:
#                                             if point.within(water_poly):
#                                                 is_water_cell = True
#                                                 break
                                    
#                                     if i == 0 or i == self.cell_size or j == 0 or j == self.cell_size:
#                                         cell_boundary.append((i + column * self.cell_size, j + row * self.cell_size))
                
#                 if self.cell_size % self.cell_wall_interval == 0:
#                     if len(cell_boundary) == (self.cell_size / self.cell_wall_interval) * 4:
#                         cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
#                         for count in range(len(cell_boundary_points_x)):
#                             cell_squares_within_boundary_x.append(cell_boundary_points_x[count])
#                             cell_squares_within_boundary_y.append(cell_boundary_points_y[count])
#                         # 新增：如果是水域单元格，记录中心点
#                         if is_water_cell and water_polygons:
#                             center_x = column * self.cell_size + self.cell_size / 2
#                             center_y = row * self.cell_size + self.cell_size / 2
#                             water_cells_x.append(center_x)
#                             water_cells_y.append(center_y)
#                 else:
#                     if len(cell_boundary) == (np.floor(self.cell_size / self.cell_wall_interval) + 1) * 4:
#                         cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
#                         for count in range(len(cell_boundary_points_x)):
#                             cell_squares_within_boundary_x.append(cell_boundary_points_x[count])
#                             cell_squares_within_boundary_y.append(cell_boundary_points_y[count])
#                         # 新增：如果是水域单元格，记录中心点
#                         if is_water_cell and water_polygons:
#                             center_x = column * self.cell_size + self.cell_size / 2
#                             center_y = row * self.cell_size + self.cell_size / 2
#                             water_cells_x.append(center_x)
#                             water_cells_y.append(center_y)
        
#         if len(cells) < self.number_of_partitions:
#             print("Allocatable cells less than total number of robots provided")
#             raise
        
#         # 将水域单元格信息存储到实例变量中
#         self.water_cells_x = water_cells_x
#         self.water_cells_y = water_cells_y
        
#         return [cell_squares_within_boundary_x, cell_squares_within_boundary_y], cells

#     def partition_area(self, cell_space, cells):
#         """
#         将可调查区域划分为相等大小的子区域，每个机器人一个区域
#         """
#         cell_squares_within_boundary_x = cell_space[0]
#         cell_squares_within_boundary_y = cell_space[1]
#         cells_within_boundary_x = []
#         cells_within_boundary_y = []
#         for row, _ in enumerate(cells):
#             for column, _ in enumerate(cells[0]):
#                 cells_within_boundary_x.append(row)
#                 cells_within_boundary_y.append(column)
#         clustering_set = np.transpose([cell_squares_within_boundary_x, cell_squares_within_boundary_y])
#         kmeans = MiniBatchKMeans(
#             n_clusters=self.number_of_partitions,   # 簇数不变
#             max_iter=self.partition_max_iter,       # 最大迭代次数不变
#             tol=self.partition_tolerance,           # 收敛容忍度不变
#             n_init=20,                              # 随机初始化次数设为 20
#             batch_size=4096,                        # 建议 ≥4096，避免 Windows+MKL 内存泄漏
#             random_state=42  
#         )
#         cluster_indices = kmeans.fit_predict(clustering_set)



#         self.cluster_centers = [kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1]]
#         self.partition_colors = lhs(3, samples=self.number_of_partitions)
#         self.partition_colors = np.round(self.partition_colors, decimals=1)
#         robot_assignment_information = [[] for robot in range(self.number_of_partitions)]
#         robot_initial_positions_copy = self.robot_initial_positions_in_cartesian.copy()
#         self.rough_partitioning_x = [[] for robot in range(self.number_of_partitions)]
#         self.rough_partitioning_y = [[] for robot in range(self.number_of_partitions)]
#         minimum_distance = [[np.inf] for robot in range(self.number_of_partitions)]
#         cell_space_as_dict = dict()
#         cells_as_dict = dict()
#         for robot_id in range(self.number_of_partitions):
#             for point_index, point in enumerate(clustering_set):
#                 if cluster_indices[point_index] != robot_id:
#                     continue
#                 else:
#                     row = point[0]
#                     column = point[1]
#                     self.rough_partitioning_x[robot_id].append(row)
#                     self.rough_partitioning_y[robot_id].append(column)
#                     cell_space_as_dict[column, row] = robot_id
#                     if row % self.cell_size == 0 and column % self.cell_size == 0:
#                         cells_as_dict[column, row] = robot_id
#                         for pos in robot_initial_positions_copy:
#                             if np.linalg.norm(
#                                     [row - pos[0] + 0.0001,
#                                      column - pos[1] + 0.0001]) < minimum_distance[robot_id]:
#                                 minimum_distance[robot_id] = np.linalg.norm(
#                                     [row - pos[0] + 0.0001, column - pos[1] + 0.0001])
#                                 assignment_information = [pos[0], pos[1], row + self.cell_size / 2,
#                                                           column + self.cell_size / 2,
#                                                           round(minimum_distance[robot_id] / self.cell_size)]
#             robot_assignment_information[robot_id] = assignment_information
#             robot_initial_positions_copy.remove([assignment_information[0], assignment_information[1]])
        
#         return cells_as_dict, cell_space_as_dict, robot_assignment_information
        
#     def resolve_conflicts(self, cells_as_dict, cell_space_as_dict, robot_assignment_information, mode):
#         """
#         Conflict resolution phase: resolves conflicts at the boundaries between initial partition regions based on
#         robot states.
#         """
#         # FIND INHERENT CONFLICTS
#         self.conflict_cells_x = []
#         self.conflict_cells_y = []
#         self.data_tasks_per_robot = [0 for robot in range(self.number_of_partitions)]
#         non_dominated_cells = []

#         # ===== 新增水域检测部分开始 =====
#         # 创建水域多边形列表
#         water_polygons = []
#         if hasattr(self, 'geographical_water_fencing_zones_in_cartesian') and \
#         self.geographical_water_fencing_zones_in_cartesian:
#             for water_zone in self.geographical_water_fencing_zones_in_cartesian:
#                 water_polygons.append(Polygon(water_zone))
        
#         # 初始化各机器人水域单元格计数
#         water_cell_counts = [0] * self.number_of_partitions
#         # ===== 新增水域检测部分结束 =====

#         for corner in cells_as_dict:
#             cell_boundary_values = []
#             cell_boundary_points_y = []
#             cell_boundary_points_x = []

#             # ===== 新增水域单元格检测开始 =====
#             # 检查当前单元格中心点是否在水域内
#             cell_center_x = corner[1] + self.cell_size/2
#             cell_center_y = corner[0] + self.cell_size/2
#             is_water_cell = False
#             if water_polygons:
#                 center_point = Point(cell_center_x, cell_center_y)
#                 for poly in water_polygons:
#                     if center_point.within(poly):
#                         is_water_cell = True
#                         break
#             # ===== 新增水域单元格检测结束 =====

#             for i in range(self.cell_size + 1):
#                 for j in range(self.cell_size + 1):
#                     try:
#                         cell_boundary_values.append(cell_space_as_dict[i + corner[0], j + corner[1]])
#                         cell_boundary_points_x.append(i + corner[1])
#                         cell_boundary_points_y.append(j + corner[0])
#                     except KeyError:
#                         continue
#             if len(cell_boundary_values) >= self.cell_size / self.cell_wall_interval * 4:
#                 if not all(elem == cell_boundary_values[0] for elem in cell_boundary_values):
#                     self.conflict_cells_y.extend(cell_boundary_points_y)
#                     self.conflict_cells_x.extend(cell_boundary_points_x)
#                     occupied_partitions = []
#                     for partition_number in cell_boundary_values:
#                         if partition_number not in occupied_partitions:
#                             occupied_partitions.append(partition_number)
#                     non_dominated_cells.append([[corner[0], corner[1]], occupied_partitions])
#                 else:
#                     robot = int(cell_boundary_values[0])
#                     self.data_tasks_per_robot[robot] += 1
#                     # === 新增开始：水域单元格计数 ===
#                     if is_water_cell:
#                         water_cell_counts[robot] += 1
#                     # === 新增结束 ===                    
#                     # self.data_tasks_per_robot[int(cell_boundary_values[0])] += 1
#                     self.dominated_cells_x[int(cell_boundary_values[0])].append(int(corner[1] + self.cell_size / 2))
#                     self.dominated_cells_y[int(cell_boundary_values[0])].append(int(corner[0] + self.cell_size / 2))

#         # ===== 新增水域占比计算开始 =====
#         # 计算各机器人当前水域占比
#         robot_water_ratios = []
#         for robot in range(self.number_of_partitions):
#             total_cells = self.data_tasks_per_robot[robot] if self.data_tasks_per_robot[robot] > 0 else 1
#             ratio = water_cell_counts[robot] / total_cells
#             robot_water_ratios.append(ratio)
#         # ===== 新增水域占比计算结束 =====

#         # ===== 新增水域统计打印开始 =====
#         print("\n=== Water Terrain sum ===")
#         for robot in range(self.number_of_partitions):
#             total_cells = self.data_tasks_per_robot[robot]
#             water_cells = water_cell_counts[robot]
#             land_cells = total_cells - water_cells
#             print(f"Robot {robot}: Total grids={total_cells} | "
#                 f"water grids={water_cells}({water_cells/total_cells:.1%}) | "
#                 f"land grids={land_cells}({land_cells/total_cells:.1%})")
#         print("====================\n")
#         # ===== 新增水域统计打印结束 =====

#         # 解决冲突 (保持原结构)
#         self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
#         self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
#         # Get initial partitioning distribution information
#         # self.data_tasks_per_robot, non_dominated_cells = self.sum_cells_per_robot(cell_space_as_dict)
#         if mode == "bias":
#             # Add bias to robots based on robot states
#             distance_bias = np.transpose(robot_assignment_information.copy())[4] * self.bias_factor

#             # ===== 新增水域偏置计算开始 =====
#             # 水域偏置系数 (可在__init__中配置，默认0.5)
#             water_bias = [water_cell_counts[robot] * self.water_bias_factor 
#                         for robot in range(self.number_of_partitions)]
#             total_bias = distance_bias + water_bias
#             # ===== 新增水域偏置计算结束 =====

#             # Resolve conflicts (nondominated cells)

#             for cell in non_dominated_cells:
#                 column = int(cell[0][0])
#                 row = int(cell[0][1])
#                 lower_cell_count_robot = cell[1][0]
#                 for region in cell[1]:
#                     if self.data_tasks_per_robot[int(region)] + total_bias[int(region)] < \
#                             self.data_tasks_per_robot[int(lower_cell_count_robot)] +\
#                             total_bias[int(lower_cell_count_robot)]:
#                         lower_cell_count_robot = region
#                 cell_space_as_dict[cell_space_as_dict[cell[0][0], cell[0][1]]] = lower_cell_count_robot
#                 self.final_partitioning_y[int(lower_cell_count_robot)].append(int(column + self.cell_size / 2))
#                 self.final_partitioning_x[int(lower_cell_count_robot)].append(int(row + self.cell_size / 2))
#                 self.data_tasks_per_robot[int(lower_cell_count_robot)] += 1
#         else:
#             # Resolve conflicts using random walk (nondominated cells)
#             self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
#             self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
#             for cell in non_dominated_cells:
#                 row = int(cell[0][0] - self.cell_size / 2)
#                 column = int(cell[0][1] - self.cell_size / 2)
#                 robot = np.random.randint(0, self.number_of_partitions, )
#                 self.final_partitioning_y[robot].append(int(column + self.cell_size / 2))
#                 self.final_partitioning_x[robot].append(int(row + self.cell_size / 2))
#                 self.data_tasks_per_robot[robot] += 1

#         # SAVE FINAL PARTITIONS
#         final_partitions = [[] for robot in range(self.number_of_partitions)]
#         for robot_id in range(self.number_of_partitions):
#             final_partitions[robot_id] = [self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id]]
#         return robot_assignment_information, final_partitions

#     def find_optimal_paths(self, planner, robot_assignment_information, waypoints_for_robots):
#         """
#         Path planning phase: find the optimal path for each robot based on their assigned list of cells
#         """
#         self.optimal_paths = [[] for robot in range(self.number_of_partitions)]
#         waypoint_distances = [[] for robot in range(self.number_of_partitions)]
        

#         print(len(waypoints_for_robots[0]))
        
#         # Set the save path to the desired directory (recommend relative paths for collaboration)
#         self.save_path = r".\output"
#         # 创建水域多边形列表
#         water_polygons = []
#         if self.geographical_water_fencing_zones_in_cartesian:
#             for water_zone in self.geographical_water_fencing_zones_in_cartesian:
#                 water_polygons.append(Polygon(water_zone))

#         if planner == "nn":
#             for robot, cell_list in enumerate(waypoints_for_robots):
#                 current_position = robot_assignment_information[robot][0:2]
#                 task_list = np.transpose(cell_list).tolist()
#                 task_list.append(current_position)
#                 while True:
#                     if len(task_list) == 1:
#                         waypoint_distances[robot].append(
#                             sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
#                                 (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
#                         self.optimal_paths[robot].append(current_position)
#                         task_list.remove(current_position)
#                         break
#                     self.optimal_paths[robot].append(current_position)
#                     nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
#                         task_list)
#                     distances, indices = nbrs.kneighbors(task_list)
#                     waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
#                     next_position = task_list[indices[task_list.index(current_position)][1]]
#                     task_list.remove(current_position)
#                     current_position = next_position

#         elif planner == "random_walk":
#             for robot, cell_list in enumerate(waypoints_for_robots):
#                 current_position = robot_assignment_information[robot][0:2]
#                 task_list = np.transpose(cell_list).tolist()

#                 self.optimal_paths[robot].append(current_position)
#                 radius = 1
#                 while len(task_list) > 0:
#                     for task in random.sample(task_list, len(task_list)):
#                         distance = sqrt((current_position[0] - task[0]) ** 2 +
#                                         (current_position[1] - task[1]) ** 2)
#                         if distance <= radius:
#                             waypoint_distances[robot].append(distance)
#                             task_list.remove(task)
#                             current_position = task
#                             self.optimal_paths[robot].append(task)
#                             radius = 0
#                             continue
#                     radius += 1

#         elif planner == "tsp:brute":
#             for robot, path in enumerate(waypoints_for_robots):
#                 shortest_distance = np.inf
#                 start = self.robot_assignment_information[robot][0:2]
#                 path = np.concatenate((np.array([start]), np.transpose(path).tolist())).tolist()
#                 import itertools
#                 all_permutations = itertools.permutations(path)
#                 for path_permutation in all_permutations:
#                     if path_permutation[0] == start:
#                         distance = 0
#                         distances = []
#                         current_position = start
#                         for next_position in path_permutation[1:]:
#                             distance += sqrt((current_position[0] - next_position[0]) ** 2 +
#                                             (current_position[1] - next_position[1]) ** 2)
#                             distances.append(sqrt((current_position[0] - next_position[0]) ** 2 +
#                                                 (current_position[1] - next_position[1]) ** 2))
#                             current_position = next_position
#                         if distance < shortest_distance:
#                             self.optimal_paths[robot] = list(path_permutation)
#                             waypoint_distances[robot] = distances

#         elif planner == "tsp:ga":
#             optimal_paths_init = [None for robot in range(self.number_of_partitions)]
#             for robot, partition in enumerate(waypoints_for_robots):
#                 start = self.robot_assignment_information[robot][0:2]
#                 coords_list = np.concatenate((np.array([start]), np.transpose(partition)))
#                 coord_graph = nx.Graph()
#                 node = 0
#                 dist_list = []
#                 for coord in coords_list:
#                     coord_graph.add_node(node, location=coord)
#                     node += 1
#                 for coord_from in coord_graph.nodes:
#                     for coord_to in coord_graph.nodes:
#                         if coord_from == coord_to:
#                             continue
#                         x2 = coord_graph.nodes[coord_to]["location"][0]
#                         x1 = coord_graph.nodes[coord_from]["location"][0]
#                         y2 = coord_graph.nodes[coord_to]["location"][1]
#                         y1 = coord_graph.nodes[coord_from]["location"][1]
#                         if sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) == 0:
#                             continue
#                         dist_list.append((coord_from, coord_to, sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)))
#                 fit_dists = mlrose.TravellingSales(distances=dist_list)
#                 problem_fit = mlrose.TSPOpt(length=node, fitness_fn=fit_dists, maximize=False)
#                 best_state, best_fitness = mlrose.genetic_alg(problem_fit, pop_size=400, mutation_prob=0.2,
#                                                             max_attempts=5, max_iters=100)
#                 entry = []
#                 for node in best_state:
#                     entry.append(tuple(coord_graph.nodes[node]["location"]))
#                 optimal_paths_init[robot] = entry
#             self.optimal_paths = []
#             for robot, path in enumerate(optimal_paths_init):
#                 ind = path.index(tuple(start))
#                 path = [path[(i + ind) % len(path)]
#                         for i, x in enumerate(path)]
#                 self.optimal_paths.append(path)
#             for robot, optimal_path in enumerate(self.optimal_paths):
#                 distances = []
#                 current_position = start
#                 for next_position in optimal_path[1:]:
#                     distances.append(sqrt((current_position[0] - next_position[0]) ** 2 +
#                                         (current_position[1] - next_position[1]) ** 2))
#                     current_position = next_position

#                 waypoint_distances[robot] = distances

#         # Save optimal path information to specified save path
#         with open(self.save_path + "waypoints", 'wb') as f:
#             pickle.dump(self.optimal_paths, f)
        

#         self._init_water_polygons()
        
#         return self.optimal_paths, waypoint_distances     
       
#     def _init_water_polygons(self):
#         """初始化水域多边形（新增方法）"""
#         if hasattr(self, 'geographical_water_fencing_zones_in_cartesian') and self.geographical_water_fencing_zones_in_cartesian:
#             self.water_polygons = [Polygon(zone) for zone in self.geographical_water_fencing_zones_in_cartesian]
#         else:
#             self.water_polygons = []
    
#     def _get_water_speed_ratio(self):
#         """获取水域速度比例（新增方法）"""
#         if hasattr(self, 'water_velocity') and hasattr(self, 'robot_velocity'):
#             return self.water_velocity / self.robot_velocity
#         return 1.0  # 默认不减速
    
#     def _calculate_segment_water_distance(self, start, end):
#         """计算单段路径的水域距离（新增方法）"""
#         if not hasattr(self, 'water_polygons'):
#             return 0.0
        
#         line = LineString([start, end])
#         total = 0.0
#         for poly in self.water_polygons:
#             if line.intersects(poly):
#                 intersection = line.intersection(poly)
#                 if intersection.geom_type == 'MultiLineString':
#                     total += sum(seg.length for seg in intersection)
#                 else:
#                     total += intersection.length
#         return total
        

#     def plot_partitioning(self):
#         """
#         Plot algorithm run results
#         """
#         if "full" in self.plot:
#             import pickle
#             with open(self.save_path + "/rough_partitioning.txt", "wb") as fp:  # Pickling
#                 pickle.dump([self.rough_partitioning_x, self.rough_partitioning_y], fp)
#             with open(self.save_path + "/final_partitioning.txt", "wb") as fp:  # Pickling
#                 pickle.dump([self.final_partitioning_x, self.final_partitioning_y], fp)
#             with open(self.save_path + "/dominated_cells.txt", "wb") as fp:  # Pickling
#                 pickle.dump([self.dominated_cells_x, self.dominated_cells_y], fp)
#             with open(self.save_path + "/conflict_cells.txt", "wb") as fp:  # Pickling
#                 pickle.dump([self.conflict_cells_x, self.conflict_cells_y], fp)
#             with open(self.save_path + "/partition_colors.txt", "wb") as fp:  # Pickling
#                 pickle.dump(self.partition_colors, fp)

#             with open(self.save_path + "/robot_initial_positions_in_cartesian.txt", "wb") as fp:  # Pickling
#                 pickle.dump(self.robot_initial_positions_in_cartesian, fp)

#             with open(self.save_path + "/cluster_centers.txt", "wb") as fp:  # Pickling
#                 pickle.dump(self.cluster_centers, fp)

#             with open(self.save_path + "/number_of_partitions.txt", "wb") as fp:  # Pickling
#                 pickle.dump(self.number_of_partitions, fp)

#             plt.figure(figsize=(10, 8))
#             plt.subplot(2, 2, 1)

#             for robot_id in range(self.number_of_partitions):
#                 plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
#                             s=self.plot_cell_boundary_size,
#                             c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
#                 plt.scatter(self.cluster_centers[0], self.cluster_centers[1], s=self.plot_robot_size, c='black')


#             plt.axis("equal")

#             plt.subplot(2, 2, 2)
#             for robot_id in range(self.number_of_partitions):
#                 plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
#                             s=self.plot_cell_boundary_size,
#                             c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
#                 plt.scatter(self.dominated_cells_x[robot_id], self.dominated_cells_y[robot_id], marker="s",
#                             s=self.plot_cell_size,
#                             c=np.ones((len(self.dominated_cells_x[robot_id]), 3)) * self.partition_colors[robot_id])
#             plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
#                         c="black")
       

#             plt.axis("equal")

#             plt.subplot(2, 2, 3)
#             for robot_id in range(self.number_of_partitions):
#                 plt.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
#                             s=self.plot_cell_size,
#                             c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
#             plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
#                         c="black")
        

#             plt.axis("equal")

#             ax4 = plt.subplot(2, 2, 4)
#             ax4.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
#                         np.transpose(self.robot_initial_positions_in_cartesian)[1],
#                         s=self.plot_robot_size, c="black")
#             for robot_id in range(self.number_of_partitions):
#                 ax4.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
#                             s=self.plot_cell_size,
#                             c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
#             plt.axis("equal")

#             # # ===== 2. 单独绘制大尺寸图4 =====
#             # plt.figure(figsize=(16, 12))  # 更大的画布尺寸
#             # ax_large = plt.gca()
#             # ax_large.scatter(
#             #     np.transpose(self.robot_initial_positions_in_cartesian)[0],
#             #     np.transpose(self.robot_initial_positions_in_cartesian)[1],
#             #     s=self.plot_robot_size * 3,  # 增大标记尺寸
#             #     c="black", marker='o', label="Start Positions"
#             # )
#             # for robot_id in range(self.number_of_partitions):
#             #     ax_large.scatter(
#             #         self.final_partitioning_x[robot_id], 
#             #         self.final_partitioning_y[robot_id], 
#             #         marker="s", 
#             #         s=self.plot_cell_size * 2,  # 增大单元格尺寸
#             #         c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id],
#             #         label=f"Robot {robot_id} Coverage"
#             #     )
#             #     path = np.array(self.optimal_paths[robot_id])
#             #     ax_large.plot(
#             #         path[:, 0], path[:, 1], 
#             #         c=self.partition_colors[robot_id], 
#             #         linewidth=1.5,  # 加粗路径线
#             #         linestyle='-'
#             #     )
#             # if hasattr(self, 'water_cells_x') and self.water_cells_x:
#             #     ax_large.scatter(
#             #         self.water_cells_x, self.water_cells_y, 
#             #         c='blue', marker='s', alpha=0.3, 
#             #         s=self.plot_cell_size * 1.5,  # 增大水域标记尺寸
#             #         label="Water Zones"
#             #     )
#             # ax_large.axis("equal")
#             # ax_large.set_xlabel("X Coordinate (m)", fontsize=12)
#             # ax_large.set_ylabel("Y Coordinate (m)", fontsize=12)
#             # ax_large.set_title("Multi-Robot Coverage Path Planning Results", fontsize=14)
#             # ax_large.legend(loc='upper right', fontsize=10)
#             # ax_large.grid(True, alpha=0.3)
#             # plt.tight_layout()
#             # plt.savefig("final_partition_large.png", dpi=300, bbox_inches='tight')  # 保存大图
#             # plt.show()



#             #绘制水域区域
#             for ax in [plt.subplot(2,2,i) for i in range(1,5)]:
#                 if hasattr(self, 'water_cells_x') and self.water_cells_x:
#                     ax.scatter(self.water_cells_x, self.water_cells_y, 
#                             c='blue', marker='s', alpha=0.3, label='Water Cells')
                        
#         elif "partial" in self.plot:
#             plt.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
#                         np.transpose(self.robot_initial_positions_in_cartesian)[1],
#                         s=self.plot_robot_size, c="black")
#             for robot_id in range(self.number_of_partitions):
#                 plt.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
#                             s=self.cell_size,
#                             c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
#         self.data_tasks_per_robot = np.round(self.data_tasks_per_robot)
#         self.area_per_robot = np.array(self.data_tasks_per_robot) * (self.cell_size ** 2)

#         optimal_paths_clone = []
#         for robot_id in range(self.number_of_partitions):
#             optimal_paths_clone.append(np.array(self.optimal_paths[robot_id]))
#         if "full" in self.plot:
#             for robot_id in range(self.number_of_partitions):
#                 ax4.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
#                          c=self.partition_colors[robot_id])
#             # with open("Lejeune/optimal_paths_clone.txt", "wb") as fp:  # Pickling
#             #     pickle.dump(optimal_paths_clone, fp)

#         elif "partial" in self.plot:
#             for robot_id in range(self.number_of_partitions):
#                 plt.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
#                          c=self.partition_colors[robot_id])
#                 plt.axis("equal")

#         if "area" in self.plot:
#             plt.figure()
#             plt.plot(self.area_covered_over_time_time_vector, self.area_covered_over_time)
#             plt.title("Area Surveyed Over Time")
#             plt.xlabel("Time (s)")
#             plt.ylabel("Area Surveyed (Percentage)")
#         plt.show()
        
#     def calculate_surveillance_rate(self, waypoint_distances):
#         """Calculate the total area surveyed as a function of time with water velocity"""
#         # 创建水域多边形（与discretize_massive_area保持一致）
#         water_polygons = []
#         if hasattr(self, 'geographical_water_fencing_zones_in_cartesian') and \
#         self.geographical_water_fencing_zones_in_cartesian:
#             for water_zone in self.geographical_water_fencing_zones_in_cartesian:
#                 water_polygons.append(Polygon(water_zone))
        
#         # 初始化能量消耗统计
#         self.energy_consumption = [0.0 for _ in range(self.number_of_partitions)]
#         segment_energy = [[] for _ in range(self.number_of_partitions)]  # 存储每航段能耗
#             # 计算每个航段的水域比例

#         segment_water_ratios = [[] for _ in range(self.number_of_partitions)]
#         for robot in range(self.number_of_partitions):
#             path = self.optimal_paths[robot]
#             for i in range(len(path)-1):
#                 start, end = path[i], path[i+1]
#                 total_dist = np.linalg.norm(np.array(end)-np.array(start))
                
#                 # 计算水域距离
#                 water_dist = 0.0
#                 if water_polygons and total_dist > 0:
#                     line = LineString([start, end])
#                     for poly in water_polygons:
#                         if line.intersects(poly):
#                             intersection = line.intersection(poly)
#                             if intersection.geom_type == 'MultiLineString':
#                                 water_dist += sum(seg.length for seg in intersection)
#                             else:
#                                 water_dist += intersection.length


#                 # 修改航段能耗计算部分
#                 water_ratio = water_dist / total_dist if total_dist > 0 else 0  # 先计算water_ratio
#                 segment_water_ratios[robot].append(water_ratio)  # 存储到列表
                
#                 # 计算当前航段能耗（关键修改点）
#                 land_dist = total_dist * (1 - water_ratio)
#                 water_dist = total_dist * water_ratio
#                 segment_energy[robot].append(land_dist * 1.0 + water_dist *0.1)  
#                 # 陆地1.0能量单位/m，水域0.1能量单位/m
#                 # # ========== 新增调试输出 ==========
#                 # land_dist = total_dist - water_dist
#                 # print(
#                 #     f"Segment {i}: Start={start} → End={end} | "
#                 #     f"Total={total_dist:.2f}m | "
#                 #     f"Water={water_dist:.2f}m ({water_dist/total_dist:.1%}) | "
#                 #     f"Land={land_dist:.2f}m"
#                 # )
#                 # # =================================
                

#         # --- 原有时间计算逻辑修改如下 ---
#         trip_distances = [0 for _ in range(self.number_of_partitions)]
#         for robot in range(self.number_of_partitions):
#             trip_distances[robot] = sum(waypoint_distances[robot])
        
#         initial_travel_distance = [0 for _ in range(self.number_of_partitions)]
#         for robot, d in enumerate(waypoint_distances):
#             initial_travel_distance[robot] = d[0]
        
#         trip_distances = np.array(trip_distances) - np.array(initial_travel_distance)
#         area_covered = 0
#         t = 0
#         self.data_completion_time_per_robot = [0 for _ in range(self.number_of_partitions)]
        
#         while any(waypoint_distances):
#             for robot in range(self.number_of_partitions):
#                 if not waypoint_distances[robot]:
#                     continue
                    
#                 # 获取当前航段的水域比例
#                 current_idx = len(waypoint_distances[robot]) - 1
#                 water_ratio = segment_water_ratios[robot][current_idx] if current_idx < len(segment_water_ratios[robot]) else 0
                
#                 # 计算有效速度
#                 effective_speed = self.robot_velocity * (1 - water_ratio) + self.water_velocity * water_ratio
                
#                 # 消耗距离
#                 if initial_travel_distance[robot] > 0:
#                     initial_travel_distance[robot] -= effective_speed * self.sampling_rate
#                 else:
#                     waypoint_distances[robot][0] -= effective_speed * self.sampling_rate
#                     if waypoint_distances[robot][0] <= 0:
#                         waypoint_distances[robot].pop(0)
#                         area_covered += self.cell_area
                        
#                         # 记录完成时间
#                         if not waypoint_distances[robot] and self.data_completion_time_per_robot[robot] == 0:
#                             self.data_completion_time_per_robot[robot] = t
            
#                 current_segment = len(waypoint_distances[robot]) - 1
#                 if current_segment < len(segment_energy[robot]):
#                     self.energy_consumption[robot] += segment_energy[robot][current_segment] * (
#                         effective_speed * self.sampling_rate / total_dist
#                     )   

#             # 更新时间状态
#             self.area_covered_over_time_time_vector.append(round(t, 2))
#             t += self.sampling_rate
#             self.area_covered_over_time.append(round(area_covered / self.total_survey_area, 2))
#         print("Total energy consumption of each drone:", self.energy_consumption)  
#         # —— 新增：打印所有无人机的总能耗 ——
#         total_energy = sum(self.energy_consumption)
#         print(f"Total energy consumption of all drones: {total_energy:.2f}")        
       

        
#         self.data_total_mission_completion_time = t

# class Baseline:
#     """
#         Baseline class for comparison with the SCoPP Monitoring algorithm.
#     """
#     def __init__(
#             self,
#             number_of_robots,
#             environment,
#             plot=False,
#             plot_settings=SCoPP_settings.plots()
#     ):
#         # Initialize environment parameters
#         if len(environment.starting_position) > 1:
#             geographical_starting_positions = environment.starting_position
#         else:
#             geographical_starting_positions = []
#             for agent in range(number_of_robots):
#                 geographical_starting_positions.extend(environment.starting_position)
#         geographical_boundary_points = environment.boundary_points
#         self.robot_FOV = environment.robot_FOV
#         self.robot_operating_height = environment.robot_operating_height
#         self.robot_velocity = environment.robot_velocity
#         geographical_geo_fencing_zones = environment.geo_fencing_holes
#         self.save_path = environment.save_path

#         # Initialize reference frame
#         self.robot_initial_positions_in_cartesian = []
#         self.boundary_points_in_cartesian = []
#         self.boundary_points_in_geographical = []
#         self.robot_initial_positions_in_geographical = []
#         origin = [0, 0]
#         origin[1] = min(np.concatenate(
#             (np.transpose(geographical_starting_positions)[0], np.transpose(geographical_boundary_points)[0])))
#         origin[0] = min(np.concatenate(
#             (np.transpose(geographical_starting_positions)[1], np.transpose(geographical_boundary_points)[1])))
#         self.origin = origin
#         if geographical_geo_fencing_zones:
#             self.geographical_geo_fencing_zones_in_cartesian = [[] for item in
#                                                                 range(len(geographical_geo_fencing_zones))]
#             self.geographical_geo_fencing_zones_in_geographical = [[] for item in
#                                                                    range(len(geographical_geo_fencing_zones))]
#         else:
#             self.geographical_geo_fencing_zones_in_cartesian = None
#             self.geographical_geo_fencing_zones_in_geographical = None
#         for robot, position in enumerate(geographical_starting_positions):
#             self.robot_initial_positions_in_geographical.append([position[1], position[0]])
#         for point in geographical_boundary_points:
#             self.boundary_points_in_geographical.append([point[1], point[0]])
#         if geographical_geo_fencing_zones:
#             for list_id, fence_list in enumerate(geographical_geo_fencing_zones):
#                 for point in fence_list:
#                     self.geographical_geo_fencing_zones_in_geographical[list_id].append([point[1], point[0]])

#         # Initialize method variables
#         self.number_of_partitions = number_of_robots
#         self.dominated_cells_x = [[] for robot in range(self.number_of_partitions)]
#         self.dominated_cells_y = [[] for robot in range(self.number_of_partitions)]
#         self.robot_WOV = tan(radians(self.robot_FOV / 2)) * self.robot_operating_height * 2  # in meters
#         self.cell_size = int(self.robot_WOV)
#         self.cell_area = self.cell_size ** 2  # in meters squared
#         self.robot_assignment_information = None

#         # Initialize computation time attributes
#         self.time_for_conversion = 0
#         self.time_for_partitioning = 0
#         self.time_for_discretization = 0
#         self.total_time = 0

#         # Initialize run data attributes for statistics
#         self.data_tasks_per_robot = None
#         self.data_distance_travelled_per_robot = None
#         self.data_completion_time_per_robot = None
#         self.data_total_mission_completion_time = None
#         self.data_computation_time_without_path_planning = None
#         self.data_computation_time_with_path_planning = None

#         # Initialize variables for plotting
#         self.rough_partitioning_x = None
#         self.rough_partitioning_y = None
#         self.cluster_centers = None
#         self.conflict_cells_x = None
#         self.conflict_cells_y = None
#         self.final_partitioning_x = None
#         self.final_partitioning_y = None
#         self.data_tasks_per_robot = None

#         # Plot settings
#         self.plot = plot
#         self.plot_cell_boundary_size = plot_settings.cell_boundary
#         self.plot_robot_path_size = plot_settings.robot_path_size
#         self.plot_robot_size = plot_settings.robot_size
#         self.plot_cell_size = plot_settings.cell_size
#         self.partition_colors = None

#         # Initialize other useful information
#         self.total_survey_area = None
#         self.area_per_robot = None
#         self.area_covered_over_time = []



#     def run(self):
#         t0 = time.time()
#         convert_to_cart_start_time_stamp = time.time()
#         coordinate_converter = lc.LLCCV(self.origin)
#         for item in self.robot_initial_positions_in_geographical:
#             self.robot_initial_positions_in_cartesian.append(coordinate_converter.get_cartesian(item))
#         for item in self.boundary_points_in_geographical:
#             self.boundary_points_in_cartesian.append(coordinate_converter.get_cartesian(item))
#         if self.geographical_geo_fencing_zones_in_geographical:
#             for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones_in_geographical):
#                 for item in fence_list:
#                     self.geographical_geo_fencing_zones_in_cartesian[list_id].append(
#                         coordinate_converter.get_cartesian(item))
#         self.time_for_conversion += time.time() - convert_to_cart_start_time_stamp
#         x_max = -np.inf
#         y_max = -np.inf
#         for item in self.robot_initial_positions_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
#         for item in self.boundary_points_in_cartesian:
#             if item[1] > x_max:
#                 x_max = item[1]
#             if item[0] > y_max:
#                 y_max = item[0]
#         if self.geographical_geo_fencing_zones_in_cartesian:
#             boundary_polygon = Polygon(
#                 self.boundary_points_in_cartesian,
#                 holes=self.geographical_geo_fencing_zones_in_cartesian
#             )
#         else:
#             boundary_polygon = Polygon(self.boundary_points_in_cartesian)
#         # Create initial grid world
#         grid_space = np.zeros((y_max, x_max)) + -np.inf
#         points_within_boundary_rows = []
#         points_within_boundary_columns = []
#         for point in range(len(grid_space)):
#             for column in range(len(grid_space[0])):
#                 if (point % self.cell_size == 0 and column % self.cell_size == 0) \
#                         and Point((point, column)).within(boundary_polygon):
#                     points_within_boundary_rows.append(point)
#                     points_within_boundary_columns.append(column)
#         clustering_set = np.transpose([points_within_boundary_rows, points_within_boundary_columns])
#         kmeans = KMeans(n_clusters=self.number_of_partitions, max_iter=10, tol=1,
#                         precompute_distances=True)
#         cluster_indices = kmeans.fit_predict(clustering_set)

#         # 保存聚类中心（笛卡尔坐标）
#         self.cluster_centers_cartesian = kmeans.cluster_centers_  # 形状为 (n_clusters, 2)

        
#         # —— 在这里插入 ——  
#         coordinate_converter = lc.LLCCV(self.origin)
#         self.cluster_centers_geographical = []
#         for x_cart, y_cart in self.cluster_centers_cartesian:
#             # get_geographic 要的是 [lon_offset, lat_offset]
#             lon, lat = coordinate_converter.get_geographic([x_cart, y_cart])
#             # 如果你更习惯 (lat, lon) 顺序，就按下面这样存
#             self.cluster_centers_geographical.append((lat, lon))
#         # —— 插入结束 ——  

#         # 接着，你就可以打印或在后面直接用 self.cluster_centers_geographical
#         print("\n=== Cluster Centers (Lat, Lon) ===")
#         for i, (lat, lon) in enumerate(self.cluster_centers_geographical):
#             print(f"Center {i}: {lat:.6f}, {lon:.6f}")
#         print("===============================\n")

#         partitions = [[] for robot in self.robot_initial_positions_in_cartesian]
#         for index, point in enumerate(clustering_set):
#             partitions[cluster_indices[index]].append(list(point))
#         first_point = [[] for robot in self.robot_initial_positions_in_cartesian]
#         initial_travel_distance = [0 for robot in partitions]
#         for robot, position in enumerate(self.robot_initial_positions_in_cartesian):
#             first_point[robot] = partitions[robot][0]
#             initial_travel_distance[robot] += sqrt((partitions[robot][0][0] - position[0]) ** 2 +
#                                                    (partitions[robot][0][1] - position[1]) ** 2)
#         first_point = np.array(first_point)
#         plt.scatter(clustering_set[:, 0], clustering_set[:, 1], c=cluster_indices)
#         plt.scatter(first_point[0:len(self.robot_initial_positions_in_cartesian), 0],
#                     first_point[0:len(self.robot_initial_positions_in_cartesian), 1], c="black",
#                     s=30)
#         self.data_tasks_per_robot = [0 for robot in range(self.number_of_partitions)]
#         sweep_lengths = [0 for robot in partitions]
#         for robot, partition in enumerate(partitions):
#             sweep_lengths[robot] += len(partition) + self.cell_size
#             self.data_tasks_per_robot[robot] += len(partition)
#         self.data_completion_time_per_robot = np.array(
#             sweep_lengths) * self.cell_size / self.robot_velocity + np.array(initial_travel_distance)
#         self.data_total_mission_completion_time = max(self.data_completion_time_per_robot)
#         self.total_time = time.time() - t0
