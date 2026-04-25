"""
LKH TSP求解器调用模块 (V0.1)

本模块提供调用LKH-3.exe求解TSP问题的功能：
- 调用LKH-3.exe求解TSP
- 解析LKH输出文件
- 保存访问顺序
"""

import os
import subprocess
import sys


def solve_tsp_with_lkh(data_dir_full, par_name="core_points.par", 
                       solution_name="core_points_solution.txt",
                       output_name="core_points_tsp_path.txt"):
    """
    调用LKH-3.exe求解TSP问题
    
    :param data_dir_full: 数据目录（绝对路径）
    :param par_name: PAR文件名
    :param solution_name: LKH输出的解决方案文件名
    :param output_name: 最终保存的文件名
    :return: 访问顺序列表（点索引），如果失败返回None
    """
    try:
        # LKH-3.exe的路径
        lkh_exe_path = os.path.join(data_dir_full, "LKH-3.exe")
        par_path = os.path.join(data_dir_full, par_name)
        solution_path = os.path.join(data_dir_full, solution_name)
        output_path = os.path.join(data_dir_full, output_name)
        
        # 检查文件是否存在
        if not os.path.exists(lkh_exe_path):
            print(f"  ✗ LKH-3.exe 不存在: {lkh_exe_path}")
            return None
        
        if not os.path.exists(par_path):
            print(f"  ✗ PAR文件不存在: {par_path}")
            return None
        
        print(f"  调用LKH-3.exe求解TSP问题...")
        print(f"    PAR文件: {par_path}")
        print(f"    工作目录: {data_dir_full}")
        
        # 切换到data目录执行LKH-3.exe
        original_cwd = os.getcwd()
        try:
            os.chdir(data_dir_full)
            
            # 方法1: 不使用PIPE，让输出直接到控制台（避免缓冲区阻塞）
            # 在Windows上，使用CREATE_NO_WINDOW标志避免弹出窗口
            popen_kwargs = {
                'cwd': data_dir_full,
                'stdin': subprocess.DEVNULL,  # 关闭stdin，避免等待输入
            }
            
            if sys.platform == 'win32':
                # Windows: 使用CREATE_NO_WINDOW避免弹出控制台窗口
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                startupinfo.wShowWindow = subprocess.SW_HIDE
                popen_kwargs['startupinfo'] = startupinfo
                
                # CREATE_NO_WINDOW在Python 3.7+可用
                if hasattr(subprocess, 'CREATE_NO_WINDOW'):
                    popen_kwargs['creationflags'] = subprocess.CREATE_NO_WINDOW
            
            print(f"    启动LKH-3.exe进程...")
            process = subprocess.Popen(
                [lkh_exe_path, par_name],
                **popen_kwargs
            )
            
            # 使用wait()而不是communicate()，避免缓冲区阻塞
            # wait()会等待进程完成，但不会读取输出（输出直接到控制台或丢弃）
            try:
                print(f"    等待LKH-3.exe完成（最多等待1小时）...")
                return_code = process.wait(timeout=3600)  # 1小时超时
                
            except subprocess.TimeoutExpired:
                print(f"  ✗ LKH-3.exe 执行超时（超过1小时），正在终止进程...")
                try:
                    process.terminate()  # 先尝试优雅终止
                    try:
                        process.wait(timeout=5)  # 等待5秒
                    except subprocess.TimeoutExpired:
                        process.kill()  # 强制终止
                        process.wait()  # 等待进程完全退出
                except Exception as e:
                    print(f"    终止进程时出错: {e}")
                    try:
                        process.kill()
                    except:
                        pass
                os.chdir(original_cwd)
                return None
            except KeyboardInterrupt:
                # 处理Ctrl+C
                print(f"\n  检测到中断信号，正在终止LKH-3.exe进程...")
                try:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                except Exception as e:
                    print(f"    终止进程时出错: {e}")
                    try:
                        process.kill()
                    except:
                        pass
                os.chdir(original_cwd)
                raise  # 重新抛出KeyboardInterrupt
            
            # 恢复原工作目录
            os.chdir(original_cwd)
            
            if return_code != 0:
                print(f"  ⚠ LKH-3.exe 返回码非0 (返回码: {return_code})，但继续检查输出文件...")
            else:
                print(f"  ✓ LKH-3.exe 执行成功 (返回码: {return_code})")
            
        except Exception as e:
            os.chdir(original_cwd)
            print(f"  ✗ 执行LKH-3.exe时发生错误: {e}")
            import traceback
            traceback.print_exc()
            # 即使出错，也检查是否生成了输出文件
            if not os.path.exists(solution_path):
                return None
            # 如果文件存在，继续解析（return_code 在异常情况下未定义，设为 None）
            return_code = None
        
        # 解析LKH输出文件（即使返回码非0，只要文件存在就尝试解析）
        if not os.path.exists(solution_path):
            if return_code is not None and return_code != 0:
                print(f"  ✗ 解决方案文件不存在: {solution_path}，且返回码非0，求解失败")
            else:
                print(f"  ✗ 解决方案文件不存在: {solution_path}")
            return None
        
        print(f"  解析LKH输出文件: {solution_path}")
        tour_order = parse_lkh_solution(solution_path)
        
        if tour_order is None:
            print(f"  ✗ 解析LKH输出文件失败")
            return None
        
        print(f"  ✓ 解析成功，访问顺序包含 {len(tour_order)} 个点")
        
        # 保存访问顺序到文件
        print(f"  保存TSP访问顺序到: {output_path}")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("# 核心点TSP最优访问顺序\n")
            f.write(f"# 总点数: {len(tour_order)}\n")
            f.write(f"# 格式: 从起点出发，访问所有点，最后返回起点\n")
            f.write(f"# 注意: 点索引从0开始（LKH输出从1开始，已转换为0-based）\n\n")
            
            f.write("TOUR_ORDER:\n")
            for idx, point_idx in enumerate(tour_order):
                f.write(f"{point_idx}")
                if idx < len(tour_order) - 1:
                    f.write(" -> ")
                f.write("\n")
            
            f.write("\n# 完整访问序列（包含返回起点）\n")
            f.write("FULL_TOUR:\n")
            for idx, point_idx in enumerate(tour_order):
                f.write(f"{point_idx}")
                if idx < len(tour_order) - 1:
                    f.write(" -> ")
            # 添加返回起点
            if len(tour_order) > 0:
                f.write(f" -> {tour_order[0]}\n")
        
        print(f"  ✓ TSP访问顺序已保存到: {output_path}")
        return tour_order
        
    except Exception as e:
        print(f"  ✗ TSP求解过程发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None


def parse_lkh_solution(solution_path):
    """
    解析LKH输出的解决方案文件
    
    LKH输出格式示例:
    NAME: core_points
    TYPE: TOUR
    DIMENSION: 54
    TOUR_SECTION
    1
    2
    3
    ...
    54
    -1
    EOF
    
    或者可能直接是数字列表（每行一个数字）
    
    :param solution_path: 解决方案文件路径
    :return: 访问顺序列表（点索引，0-based），如果失败返回None
    """
    try:
        with open(solution_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        tour_order = []
        in_tour_section = False
        found_numbers = False
        
        for line in lines:
            line = line.strip()
            
            # 跳过空行
            if not line:
                continue
            
            # 跳过注释行
            if line.startswith('#'):
                continue
            
            # 检查是否进入TOUR_SECTION
            if line == "TOUR_SECTION":
                in_tour_section = True
                continue
            
            # 如果找到TOUR_SECTION标记，开始解析
            # 如果没有TOUR_SECTION标记，直接尝试解析所有数字
            if not in_tour_section and not found_numbers:
                # 尝试解析第一个数字，如果成功则开始收集
                try:
                    test_num = int(line)
                    if test_num > 0:
                        found_numbers = True
                        in_tour_section = True
                except ValueError:
                    continue
            
            # 如果不在TOUR_SECTION，跳过
            if not in_tour_section:
                continue
            
            # 检查是否结束（-1或EOF）
            if line == "-1" or line == "EOF":
                break
            
            # 解析点索引（LKH使用1-based，转换为0-based）
            try:
                point_idx = int(line)
                if point_idx > 0:  # LKH输出从1开始
                    tour_order.append(point_idx - 1)  # 转换为0-based
                elif point_idx == -1:  # 结束标记
                    break
            except ValueError:
                # 忽略无法解析的行
                continue
        
        if len(tour_order) == 0:
            print(f"    警告: 未找到有效的访问顺序")
            return None
        
        print(f"    解析到 {len(tour_order)} 个点的访问顺序")
        return tour_order
        
    except Exception as e:
        print(f"    解析文件时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None

