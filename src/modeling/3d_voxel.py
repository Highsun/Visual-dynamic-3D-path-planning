import numpy as np
import open3d as o3d
import time

class VoxelEnvironment:
    def __init__(self, width=130, depth=80, height=30, res=0.5): 
        self.res = res
        self.dims = np.array([width, depth, height])
        self.grid_shape = (self.dims / res).astype(int)
        
        # 0: 未知(迷雾), 1: 自由空间, 2: 内部墙, 3: 外围墙, 4: 地板/天花板
        self.grid = np.zeros(self.grid_shape, dtype=np.int8)

        # 默认起终点位置
        self.start = np.array([20.0, 30.0, 1.0]) 
        self.goal = np.array([110.0, 40.0, 7.0])

        # 渲染控制开关
        self.render_config = {
            'show_1f': True,   
            'show_2f': True,
            'show_outer_wall': True
        }

    def add_block(self, start_xyz, size_xyz, state=2):
        s = (np.array(start_xyz) / self.res).astype(int)
        sz = (np.array(size_xyz) / self.res).astype(int)
        e = np.minimum(s + sz, self.grid_shape)
        self.grid[s[0]:e[0], s[1]:e[1], s[2]:e[2]] = state

    def add_stairs(self, start_xyz, size_xyz, height):
        step_h = 0.5
        num_steps = int(height / step_h)
        for i in range(num_steps):
            step_pos = [start_xyz[0], start_xyz[1] + i * 0.5, start_xyz[2] + i * step_h]
            self.add_block(step_pos, [size_xyz[0], size_xyz[1], step_h], state=5)

    def create_custom_scene(self):
        # 1. 基础大基座地面 (厚度1m)
        self.add_block([0, 0, 0], [self.dims[0], self.dims[1], 1], state=4)

        # 核心几何参数
        west_w, west_d = 60, 40
        east_w, east_d = 36, 56
        gap = 8
        base_x, base_y = 10, 12
        top_y = base_y + east_d 
        
        # 适配后的层高逻辑：
        # 1F: Z=0~1(地板), Z=1~6(空间) | 2F: Z=6~7(地板), Z=7~12(空间)
        total_h = 12 
        
        # --- 西楼 ---
        w_y_start = top_y - west_d
        self.add_block([base_x, w_y_start, 1], [west_w, west_d, total_h-1], state=0) # 迷雾
        self.add_block([base_x, w_y_start, 0], [west_w, 1, total_h], state=3)        # 南墙
        self.add_block([base_x, top_y-1, 0], [west_w, 1, total_h], state=3)         # 北墙
        self.add_block([base_x, w_y_start, 0], [1, west_d, total_h], state=3)        # 西墙
        self.add_block([base_x+west_w-1, w_y_start, 0], [1, west_d, total_h], state=3) # 东墙
        self.add_block([base_x, w_y_start, 6], [west_w, west_d, 1], state=4)        # 2F地板 (抬高到6m)

        # --- 东楼 ---
        east_x = base_x + west_w + gap
        self.add_block([east_x, base_y, 1], [east_w, east_d, total_h-1], state=0)
        self.add_block([east_x, base_y, 0], [east_w, 1, total_h], state=3)
        self.add_block([east_x, top_y-1, 0], [east_w, 1, total_h], state=3)
        self.add_block([east_x, base_y, 0], [1, east_d, total_h], state=3)
        self.add_block([east_x+east_w-1, base_y, 0], [1, east_d, total_h], state=3)
        self.add_block([east_x, base_y, 6], [east_w, east_d, 1], state=4)            # 2F地板 (抬高到6m)

        # --- 二楼双连廊 ---
        wall_thickness = 1

        # -- 北连廊 --
        b1_y = top_y - 17    # Y 轴起始位置
        b1_z = 6             # Z 轴高度
        b1_width = 5         # 北连廊净宽 
        
        b1_total_y = b1_width + 2 * wall_thickness
        
        # 地板
        self.add_block([base_x+west_w, b1_y - wall_thickness, b1_z], [gap, b1_total_y, 1], state=4)
        # 护栏
        self.add_block([base_x+west_w, b1_y - wall_thickness, b1_z+1], [gap, 1, 2], state=3)
        self.add_block([base_x+west_w, b1_y + b1_width, b1_z+1], [gap, 1, 2], state=3)
        # 墙体开孔 (开孔宽度必须等于连廊净宽 b1_width)
        self.add_block([base_x + west_w - 1, b1_y, b1_z+1], [1, b1_width, 4], state=1) 
        self.add_block([east_x, b1_y, b1_z+1], [1, b1_width, 4], state=1)

        # 南连廊 (控制 b2_width)
        b2_y = top_y - 27    # Y 轴起始位置
        b2_z = 6             # Z 轴高度
        b2_width = 3         # 南连廊净宽
        
        b2_total_y = b2_width + 2 * wall_thickness
        
        # 地板
        self.add_block([base_x+west_w, b2_y - wall_thickness, b2_z], [gap, b2_total_y, 1], state=4)
        # 护栏
        self.add_block([base_x+west_w, b2_y - wall_thickness, b2_z+1], [gap, 1, 2], state=3)
        self.add_block([base_x+west_w, b2_y + b2_width, b2_z+1], [gap, 1, 2], state=3)
        # 墙体开孔
        self.add_block([base_x + west_w - 1, b2_y, b2_z+1], [1, b2_width, 4], state=1)
        self.add_block([east_x, b2_y, b2_z+1], [1, b2_width, 4], state=1)

        # --- 2F 地板开孔 ---
        # 西楼红房下楼梯口
        self.add_block([base_x + 1, w_y_start + 13, 6], [4, 15, 1], state=1) 
        # 西楼服务器室 2 右楼梯口
        self.add_block([base_x + 45, w_y_start + 27, 6], [4, 8, 1], state=1)
        # 东楼左会议室左楼梯口
        self.add_block([east_x + 1, base_y + 9, 6], [4, 8, 1], state=1)
        # 东楼右会议室右楼梯口
        self.add_block([east_x + 31, base_y + 9, 6], [4, 8, 1], state=1)
        # 西楼玻璃隔断内部
        self.add_block([base_x + 11, w_y_start + 13, 6], [12, 15, 1], state=1)
        self.add_block([base_x + 38, w_y_start + 13, 6], [6, 15, 1], state=1)
        self.add_block([base_x + 23, w_y_start + 13, 6], [15, 9, 1], state=1)

        # --- 楼梯 (连接1F和2F) ---
        # 西楼红房下楼梯
        self.add_stairs([base_x + 1, w_y_start + 20, 1], [4, 2.5, 0.5], 6.0) 
        # 西楼服务器室 2 右楼梯
        self.add_stairs([base_x + 45, w_y_start + 29, 1], [4, 0.5, 0.5], 6.0)
        # 东楼左会议室左楼梯
        self.add_stairs([east_x + 1, base_y + 9, 1], [4, 4, 0.5], 6.0)
        # 东楼右会议室右楼梯
        self.add_stairs([east_x + 31, base_y + 9, 1], [4, 4, 0.5], 6.0)

        # --- 1F 内部 ---
        # -- 西楼 --
        # 服务器 1
        self.add_block([base_x+10, w_y_start+10, 1], [3, 1, 5], state=2)  # 内墙
        self.add_block([base_x+23, w_y_start+10, 1], [3, 1, 5], state=2)  # 内墙
        self.add_block([base_x+10, w_y_start+1, 1], [1, 4, 5], state=2)  # 内墙
        self.add_block([base_x+10, w_y_start+7, 1], [1, 4, 5], state=2)  # 内墙
        self.add_block([base_x+25, w_y_start+1, 1], [1, 9, 5], state=2)  # 内墙
        # 工作台
        self.add_block([base_x+35, w_y_start+1, 1], [1, 9, 5], state=2)  # 内墙
        self.add_block([base_x+35, w_y_start+10, 1], [3, 1, 5], state=2)  # 内墙
        self.add_block([base_x+44, w_y_start+1, 1], [1, 8, 5], state=2)  # 内墙
        # 武器室
        self.add_block([base_x+49, w_y_start+8, 1], [10, 1, 5], state=2)  # 内墙
        # 服务器 2
        self.add_block([base_x+49, w_y_start+15, 1], [1, 20, 5], state=2)  # 内墙
        self.add_block([base_x+50, w_y_start+26, 1], [9, 1, 5], state=2)  # 内墙
        self.add_block([base_x+53, w_y_start+15, 1], [6, 1, 5], state=2)  # 内墙
        self.add_block([base_x+45, w_y_start+34, 1], [5, 1, 5], state=2)  # 楼梯间（下）
        # 红房
        self.add_block([base_x+1, w_y_start+31, 1], [5, 1, 5], state=2)  # 内墙
        self.add_block([base_x+8, w_y_start+31, 1], [1, 8, 5], state=2)  # 内墙
        self.add_block([base_x+5, w_y_start+18, 1], [1, 13, 5], state=2)  # 楼梯间（左）
        # 健身房
        self.add_block([base_x+23, w_y_start+31, 1], [1, 8, 5], state=2)  # 内墙
        self.add_block([base_x+30, w_y_start+31, 1], [1, 1, 5], state=2)  # 内墙
        self.add_block([base_x+37, w_y_start+31, 1], [1, 8, 5], state=2)  # 内墙
        self.add_block([base_x+23, w_y_start+22, 1], [15, 1, 5], state=2)  # 隔板
        # 任务房
        self.add_block([base_x+44, w_y_start+29, 1], [1, 10, 5], state=2)  # 内墙
        self.add_block([base_x+41, w_y_start+31, 1], [3, 1, 5], state=2)  # 内墙

        # -- 东楼 --
        # 会议室
        self.add_block([east_x+5, base_y+9, 1], [1, 15, 5], state=2)  # 内墙 + 楼梯间（左）
        self.add_block([east_x+15, base_y+9, 1], [1, 6, 5], state=2)  # 内墙
        self.add_block([east_x+15, base_y+18, 1], [1, 6, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+9, 1], [1, 6, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+18, 1], [1, 6, 5], state=2)  # 内墙
        self.add_block([east_x+30, base_y+9, 1], [1, 15, 5], state=2)  # 内墙 + 楼梯间（右）

        self.add_block([east_x+5, base_y+9, 1], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+12, base_y+9, 1], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+9, 1], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+27, base_y+9, 1], [4, 1, 5], state=2)  # 内墙

        self.add_block([east_x+1, base_y+24, 1], [15, 1, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+24, 1], [11, 1, 5], state=2)  # 内墙

        self.add_block([east_x+1, base_y+17, 1], [4, 1, 5], state=2)  # 楼梯间
        self.add_block([east_x+31, base_y+17, 1], [4, 1, 5], state=2)  # 楼梯间

        # 办公区
        self.add_block([east_x+5, base_y+25, 1], [1, 4, 5], state=2)  # 内墙
        self.add_block([east_x+5, base_y+32, 1], [1, 4, 5], state=2)  # 内墙
        self.add_block([east_x+15, base_y+25, 1], [1, 11, 5], state=2)  # 内墙
        self.add_block([east_x+5, base_y+36, 1], [11, 1, 5], state=2)  # 内墙

        # 厕所
        self.add_block([east_x+1, base_y+40, 1], [6, 1, 5], state=2)  # 内墙
        self.add_block([east_x+7, base_y+40, 1], [1, 10, 5], state=2)  # 内墙
        self.add_block([east_x+7, base_y+52, 1], [1, 3, 5], state=2)  # 内墙

        # 资料室
        self.add_block([east_x+15, base_y+45, 1], [5, 1, 5], state=2)  # 内墙
        self.add_block([east_x+23, base_y+45, 1], [2, 1, 5], state=2)  # 内墙
        self.add_block([east_x+14, base_y+45, 1], [1, 10, 5], state=2)  # 内墙

        # 设备室 + 设备领用室
        self.add_block([east_x+26, base_y+48, 1], [9, 1, 5], state=2)  # 内墙
        self.add_block([east_x+25, base_y+42, 1], [1, 7, 5], state=2)  # 内墙
        self.add_block([east_x+25, base_y+27, 1], [1, 12, 5], state=2)  # 内墙
        self.add_block([east_x+29, base_y+37, 1], [6, 1, 5], state=2)  # 内墙
        self.add_block([east_x+26, base_y+29, 1], [9, 1, 5], state=2)  # 内墙

        # --- 2F 内部 ---
        # -- 西楼 --
        # 玻璃隔断
        self.add_block([base_x+10, w_y_start+12, 6], [34, 1, 5], state=2)  # 内墙
        self.add_block([base_x+44, w_y_start+12, 6], [1, 23, 5], state=2)  # 内墙
        self.add_block([base_x+10, w_y_start+28, 6], [14, 1, 5], state=2)  # 内墙
        self.add_block([base_x+37, w_y_start+28, 6], [7, 1, 5], state=2)  # 内墙
        self.add_block([base_x+23, w_y_start+23, 6], [1, 5, 5], state=2)  # 内墙
        self.add_block([base_x+23, w_y_start+22, 6], [15, 1, 5], state=2)  # 内墙
        self.add_block([base_x+37, w_y_start+23, 6], [1, 5, 5], state=2)  # 内墙
        self.add_block([base_x+10, w_y_start+13, 6], [1, 15, 5], state=2)  # 内墙

        # 拉闸间
        self.add_block([base_x+24, w_y_start+28, 6], [11, 1, 5], state=2)  # 内墙
        self.add_block([base_x+27, w_y_start+25, 6], [1, 3, 5], state=2)  # 内墙
        self.add_block([base_x+33, w_y_start+23, 6], [1, 3, 5], state=2)  # 内墙

        # 机箱房
        self.add_block([base_x+38, w_y_start+34, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([base_x+37, w_y_start+29, 6], [1, 6, 5], state=2)  # 内墙

        # 楼梯间 2 墙
        self.add_block([base_x+45, w_y_start+26, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([base_x+49, w_y_start+26, 6], [1, 9, 5], state=2)  # 内墙

        # 连廊延伸墙
        self.add_block([base_x+49, w_y_start+22, 6], [10, 1, 5], state=2)  # 内墙
        self.add_block([base_x+49, w_y_start+16, 6], [10, 1, 5], state=2)  # 内墙
        self.add_block([base_x+49, w_y_start+17, 6], [1, 5, 5], state=2)  # 内墙

        # 厕所 1
        self.add_block([base_x+1, w_y_start+12, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([base_x+5, w_y_start+7, 6], [1, 21, 5], state=2)  # 内墙
        self.add_block([base_x+6, w_y_start+7, 6], [3, 1, 5], state=2)  # 内墙
        self.add_block([base_x+8, w_y_start+1, 6], [1, 2, 5], state=2)  # 内墙
        self.add_block([base_x+8, w_y_start+5, 6], [1, 2, 5], state=2)  # 内墙

        # 厕所 2
        self.add_block([base_x+15, w_y_start+1, 6], [1, 2, 5], state=2)  # 内墙
        self.add_block([base_x+15, w_y_start+5, 6], [1, 2, 5], state=2)  # 内墙
        self.add_block([base_x+15, w_y_start+7, 6], [8, 1, 5], state=2)  # 内墙
        self.add_block([base_x+23, w_y_start+1, 6], [1, 9, 5], state=2)  # 内墙

        # 指挥室
        self.add_block([base_x+37, w_y_start+3, 6], [1, 9, 5], state=2)  # 内墙

        # 监视室
        self.add_block([base_x+38, w_y_start+5, 6], [7, 1, 5], state=2)  # 内墙
        self.add_block([base_x+44, w_y_start+6, 6], [1, 2, 5], state=2)  # 内墙
        self.add_block([base_x+44, w_y_start+10, 6], [1, 2, 5], state=2)  # 内墙

        # 服务器间
        self.add_block([base_x+49, w_y_start+6, 6], [1, 7, 5], state=2)  # 内墙
        self.add_block([base_x+50, w_y_start+12, 6], [9, 1, 5], state=2)  # 内墙
        self.add_block([base_x+49, w_y_start+5, 6], [6, 1, 5], state=2)  # 内墙
        self.add_block([base_x+54, w_y_start+1, 6], [1, 4, 5], state=2)  # 内墙

        # 升降机 + 厨房
        self.add_block([base_x+1, w_y_start+33, 6], [10, 1, 5], state=2)  # 内墙
        self.add_block([base_x+10, w_y_start+36, 6], [1, 3, 5], state=2)  # 内墙
        self.add_block([base_x+13, w_y_start+33, 6], [20, 1, 5], state=2)  # 内墙
        self.add_block([base_x+22, w_y_start+36, 6], [1, 3, 5], state=2)  # 内墙
        self.add_block([base_x+32, w_y_start+34, 6], [1, 5, 5], state=2)  # 内墙

        # 免保房
        self.add_block([base_x+53, w_y_start+29, 6], [1, 8, 5], state=2)  # 内墙
        self.add_block([base_x+53, w_y_start+28, 6], [6, 1, 5], state=2)  # 内墙

        # -- 东楼 --
        # 免保房
        self.add_block([east_x+6, base_y+50, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+13, base_y+50, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+6, base_y+36, 6], [1, 3, 5], state=2)  # 内墙
        self.add_block([east_x+6, base_y+44, 6], [1, 6, 5], state=2)  # 内墙
        self.add_block([east_x+6, base_y+35, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+13, base_y+35, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+16, base_y+36, 6], [1, 14, 5], state=2)  # 内墙

        # 暗房
        self.add_block([east_x+20, base_y+36, 6], [1, 14, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+50, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+27, base_y+50, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+20, base_y+35, 6], [11, 1, 5], state=2)  # 内墙
        self.add_block([east_x+30, base_y+36, 6], [1, 14, 5], state=2)  # 内墙

        # 东楼经理室
        self.add_block([east_x+10, base_y+28, 6], [2, 1, 5], state=2)  # 内墙
        self.add_block([east_x+10, base_y+18, 6], [2, 1, 5], state=2)  # 内墙
        self.add_block([east_x+14, base_y+18, 6], [13, 1, 5], state=2)  # 内墙
        self.add_block([east_x+18, base_y+21, 6], [1, 7, 5], state=2)  # 内墙
        self.add_block([east_x+14, base_y+28, 6], [13, 1, 5], state=2)  # 内墙
        self.add_block([east_x+10, base_y+10, 6], [1, 18, 5], state=2)  # 内墙
        self.add_block([east_x+26, base_y+10, 6], [1, 18, 5], state=2)  # 内墙
        self.add_block([east_x+10, base_y+9, 6], [17, 1, 5], state=2)  # 内墙

        # 楼梯间 + 阳台
        self.add_block([east_x+30, base_y+8, 6], [5, 1, 5], state=2)  # 内墙
        self.add_block([east_x+26, base_y+8, 6], [2, 1, 5], state=2)  # 内墙
        self.add_block([east_x+1, base_y+8, 6], [5, 1, 5], state=2)  # 内墙
        self.add_block([east_x+9, base_y+8, 6], [2, 1, 5], state=2)  # 内墙

        self.add_block([east_x+30, base_y+9, 6], [1, 9.5, 5], state=2)  # 内墙
        self.add_block([east_x+29, base_y+17.5, 6], [1, 1, 5], state=2)  # 内墙
        self.add_block([east_x+27, base_y+21, 6], [4, 1, 5], state=2)  # 内墙
        self.add_block([east_x+5, base_y+9, 6], [1, 9.5, 5], state=2)  # 内墙

    def visualize(self):
        print("开始构建地图渲染数据...")
        start_time = time.time()
        
        all_indices = np.argwhere(self.grid > 1)
        valid_indices, valid_colors = [], []

        for idx in all_indices:
            # 计算体素底部的物理高度
            phys_z = idx[2] * self.res
            state = self.grid[idx[0], idx[1], idx[2]]
            
            # --- 1. 外墙独立控制 (最高优先级) ---
            if state == 3 and not self.render_config['show_outer_wall']:
                continue

            # --- 2. 地基逻辑 (Z < 1.0m) ---
            # 地基作为地图基座，只要 show_1f 开启就显示，或者设定为永久显示
            if phys_z < 1.0:
                if not self.render_config['show_1f']: # 如果连 1F 都不看，通常底座也不看
                    continue
                # 否则直接进入颜色分配，不参与后面的楼层过滤
            
            else:
                # --- 3. 楼层绑定逻辑 ---
                
                # 1F 内部内容 (1.0m <= Z < 6.0m)
                # 包含 1F 内墙
                if 1.0 <= phys_z < 6.0:
                    if not self.render_config['show_1f']:
                        continue
                
                # 2F 内部内容 (Z >= 6.0m)
                # 包含 2F 地板(天花板) 和 2F 内墙
                if phys_z >= 6.0:
                    if not self.render_config['show_2f']:
                        continue

            # --- 4. 颜色分配 ---
            if state == 3: 
                color = [0.25, 0.25, 0.25] # 外墙：深灰
            elif state == 2: 
                color = [0.6, 0.4, 0.2]    # 内墙：棕色
            elif state == 4: 
                color = [0.75, 0.75, 0.75] # 地板/天花板：浅灰
            elif state == 5:
                color = [0.6, 0.2, 0.8]    # 楼梯：紫色
            else: 
                color = [1.0, 1.0, 1.0]
                
            valid_indices.append(idx)
            valid_colors.append(color)

        if not valid_indices:
            print("警告：没有可渲染的体素。")
            return

        # --- 5. Open3D 渲染 ---
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(valid_indices) * self.res + self.res / 2.0)
        pcd.colors = o3d.utility.Vector3dVector(np.array(valid_colors))
        v_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.res)

        s_node = o3d.geometry.TriangleMesh.create_sphere(radius=1.5); s_node.paint_uniform_color([0.2, 0.8, 0.2]); s_node.translate(self.start)
        g_node = o3d.geometry.TriangleMesh.create_sphere(radius=1.5); g_node.paint_uniform_color([0.9, 0.1, 0.1]); g_node.translate(self.goal)

        process_time = time.time() - start_time
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="3D Voxel Scene - WQHD", width=2560, height=1440)
        vis.add_geometry(v_grid); vis.add_geometry(s_node); vis.add_geometry(g_node)
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0))
        
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.05, 0.05, 0.05])
        vis.get_view_control().set_zoom(0.4)
        
        print(f"地图构建完成！渲染体素: {len(valid_indices)} | 耗时: {process_time:.4f}s")
        vis.run(); vis.destroy_window()

if __name__ == "__main__":
    env = VoxelEnvironment()
    env.create_custom_scene()
    env.visualize()