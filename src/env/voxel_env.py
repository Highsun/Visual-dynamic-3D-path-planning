import numpy as np
import open3d as o3d

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
            self.add_block(step_pos, [size_xyz[0], size_xyz[1], step_h], state=4)

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

        # --- 二楼双连廊 (高度同步抬升到 6m) ---
        bridge_w = 2
        for y_pos in [top_y - 10, top_y - 10 - 16 - bridge_w]:
            self.add_block([base_x+west_w, y_pos, 6], [gap, bridge_w, 1], state=4)   # 连廊地板
            self.add_block([base_x+west_w, y_pos, 7], [gap, 1, 2], state=3)          # 护栏北
            self.add_block([base_x+west_w, y_pos+bridge_w-1, 7], [gap, 1, 2], state=3)# 护栏南

    def visualize(self):
        import time
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
                if not self.render_config['show_1f']: # 如果连1楼都不看，通常底座也不看
                    continue
                # 否则直接进入颜色分配，不参与后面的楼层过滤
            
            else:
                # --- 3. 楼层绑定逻辑 ---
                
                # 1F 内部内容 (1.0m <= Z < 6.0m)
                # 包含1F内墙
                if 1.0 <= phys_z < 6.0:
                    if not self.render_config['show_1f']:
                        continue
                
                # 2F 内部内容 (Z >= 6.0m)
                # 包含2F地板(天花板) 和 2F内墙
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
        vis.create_window(window_name="3D Voxel Scene - Logic Fixed", width=2560, height=1440)
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