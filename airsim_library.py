import airsim
import time
import numpy as np


class Airsim_Library:

    def Find_Coordinate_for_Target_Time(self, h, L, t_target, GT_Position):
        def Communication_time(h, d_k, L):
            f_c = 9 * 10 ** 8
            a = 9.61
            b = 0.16
            n_los = 1
            n_nlos = 20
            B = 2 * 10 ** 6
            c = 3 * 10 ** 8
            p = 0.005  # w

            d_k_safe = np.where(d_k == 0, 1e-10, d_k)

            P = 1 / (1 + a * np.exp(-b * (np.arctan(h / d_k_safe) - a)))
            A = n_los - n_nlos
            C = 20 * np.log10(4 * np.pi * f_c / c) + n_nlos
            l = 20 * np.log10(np.sqrt(h ** 2 + d_k_safe ** 2)) + A * P + C
            N_0 = 10 ** (-169 / 10)  # -114dbm/Hz
            R = B * np.log(1 + (P * 10 ** (-l / 10)) / (B * N_0))

            t = L / R
            return t

        def find_point_for_target_time(h, L, t_target, GT_Position):
            d_k = np.sqrt(GT_Position[0] ** 2 + GT_Position[1] ** 2)
            original_time = Communication_time(h, d_k, L)
            target_time = original_time * t_target

            num_samples = 10000
            for i in np.linspace(0, 1, num_samples):
                x = GT_Position[0] * i
                y = GT_Position[1] * i
                d = np.sqrt((x - GT_Position[0]) ** 2 + (y - GT_Position[1]) ** 2)
                t = Communication_time(h, d, L)

                if abs(t - target_time) < 0.01:
                    return [x, y, h]

        new_coordinates = []
        for i, GT in enumerate(GT_Position):
            new_coordinate = find_point_for_target_time(h, L[i], t_target, GT)
            new_coordinates.append(new_coordinate)

        return None

    def UAV_Energy_Consumption(self,distance):
        rou = 1.225
        s = 0.05
        G = 0.503
        U_tip = 120
        d_0 = 0.6
        v_0 = 0.6
        P_0 = 12 * 30 ** 3 * 0.4 ** 3 / 8 * rou * s * G
        P_1 = (1.1 * 20 ** (3 / 2)) / (2 * rou * G) ** 0.5
        P_2 = 11.46
        v_h = 10
        v_v = 0

        t = distance / v_h
        E_uav = t * (P_0 * (1 + (3 * (v_h) ** 2) / (U_tip) ** 2) + 0.5 * d_0 * rou * s * G * (v_h) ** 3 + P_1 * (
                    (1 + ((v_h) ** 4) / (4 * v_0 ** 4)) ** 0.5 - (v_h ** 2) / (2 * (v_0) ** 2)) ** 0.5 + P_2 * v_v)
        return E_uav

    def Communication_Energy_Consumption(self,h,d_k,L):
        f_c = 9 * 10 ** 8
        a = 9.61
        b = 0.16
        n_los = 1
        n_nlos = 20
        B = 2 * 10 ** 6
        c = 3 * 10 ** 8
        p = 0.005  # w

        P = 1 / (1 + a * np.exp(-b * (np.arctan(h / d_k) - a)))
        A = n_los - n_nlos
        C = 20 * np.log10(4 * np.pi * f_c / c) + n_nlos
        l = 20 * np.log10(np.sqrt(h ** 2 + d_k ** 2)) + A * P + C
        N_0 = 10 ** (-169 / 10)  # -114dbm/Hz
        R = B * np.log(1 + (P * 10 ** (-l / 10)) / (B * N_0))

        t = L / R
        E = p * t
        return(E)

    # def TSP_solve(self,GTs):
    #
    #     # 计算两个城市之间的距离
    #     def distance(GT1, GT2):
    #         return math.sqrt((GT1[0] - GT2[0]) ** 2 + (GT1[1] - GT2[1]) ** 2)
    #
    #     # 计算一条路径的总距离
    #     def total_distance(GTs, path):
    #         return sum(distance(GTs[path[i]], GTs[path[i + 1]]) for i in range(len(path) - 1))
    #
    #     # 解决 TSP 问题
    #     def solve_tsp(GTS):
    #         n = len(GTs)
    #         # 生成所有可能的城市访问顺序
    #         all_possible_paths = itertools.permutations(range(n))
    #
    #         # 初始化最优路径和最短距离
    #         shortest_path = None
    #         min_distance = float('inf')
    #
    #         # 遍历每个路径，找到最短路径
    #         for path in all_possible_paths:
    #             path_distance = total_distance(GTS, path + (path[0],))  # 返回起点
    #             if path_distance < min_distance:
    #                 min_distance = path_distance
    #                 shortest_path = path
    #
    #         return shortest_path, min_distance
    #
    #
    #     # 解决 TSP 问题并输出最优路径和最短距离
    #     path, distance = solve_tsp(GTs)
    #     solutions = [GTs[i] for i in path]
    #     return solutions

    def time_sleep(self,times):
        time.sleep(times)
    def takeoff(self):
        self.client.takeoffAsync().join()


    def fly_to(self, point):
        if point[2] > 0:
            self.client.moveToPositionAsync(point[0], point[1], -point[2], 20).join()
        else:
            self.client.moveToPositionAsync(point[0], point[1], point[2], 10).join()
    def draw_grid(self):
        grid_size = 80
        spacing = 1
        height =0

        lines = []

        # 绘制水平线
        for i in range(grid_size + 1):
            start_point = airsim.Vector3r(i * spacing, 0, height)
            end_point = airsim.Vector3r(i * spacing, grid_size * spacing, height)
            lines.append(start_point)
            lines.append(end_point)

        # 绘制垂直线
        for i in range(grid_size + 1):
            start_point = airsim.Vector3r(0, i * spacing, height)
            end_point = airsim.Vector3r(grid_size * spacing, i * spacing, height)
            lines.append(start_point)
            lines.append(end_point)

        # 绘制网格线条
        self.client.simPlotLineList(lines, color_rgba=[0, 0, 1, 0], thickness=1, is_persistent=True)

    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.draw_grid()
        self.takeoff()
        self.fly_to([0,0,-0.1])


    def fly_path(self, points):
        airsim_points = []
        for point in points:
            if point[2] > 0:
                airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
            else:
                airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
        self.client.moveOnPathAsync(airsim_points, 2).join()


    def trace_line(self):
        self.client.simSetTraceLine([0, 1, 0, 1], thickness=5)

    


    def land(self):
        self.client.landAsync().join()

    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]




