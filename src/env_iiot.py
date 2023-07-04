import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import random
import math


pre_action = []
view_node = []
hop_collection = []


class Drones(object):
    def __init__(self, pos, view_range):
        self.pos = pos
        self.view_range = view_range    # 显示距离


class Human(object):
    id = 0
    max_size = 600
    # sol_size = 400
    cur_size = 100
    cos_speed = 50
    tru_id1 = 0.85
    tru_id2 = 0.85
    tru_id3 = 0.85
    tru_id4 = 0.85
    tru_ths = 0.6
    flag = 1

    def __init__(self, pos, i, si, csi, cos_sp, ne_id1, ne_id2, ne_id3, ne_id4):
        self.pos = pos
        self.id = i
        self.max_size = si
        # self.sol_size = ssi
        self.cur_size = csi
        self.cos_speed = cos_sp
        self.ne_id1 = ne_id1
        self.ne_id2 = ne_id2
        self.ne_id3 = ne_id3
        self.ne_id4 = ne_id4
        self.flag = 1

    def compu(self, n1, ev, jud):
        # print(self.tru_id1, self.tru_id2)
        if jud == 1 and self.ne_id1 == n1 and 1 > self.tru_id1 >= self.tru_ths:
            self.tru_id1 = self.tru_id1 + (self.tru_id1 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 1 and self.ne_id2 == n1 and 1 > self.tru_id2 >= self.tru_ths:
            self.tru_id2 = self.tru_id2 + (self.tru_id2 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 0 and self.ne_id1 == n1 and 1 > self.tru_id1 >= self.tru_ths:
            self.tru_id1 = self.tru_id1 - (self.tru_id1 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 0 and self.ne_id2 == n1 and 1 > self.tru_id2 >= self.tru_ths:
            self.tru_id2 = self.tru_id2 - (self.tru_id2 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 1 and self.ne_id3 == n1 and 1 > self.tru_id3 >= self.tru_ths:
            self.tru_id3 = self.tru_id3 + (self.tru_id3 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 1 and self.ne_id4 == n1 and 1 > self.tru_id4 >= self.tru_ths:
            self.tru_id4 = self.tru_id4 + (self.tru_id4 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 0 and self.ne_id3 == n1 and 1 > self.tru_id3 >= self.tru_ths:
            self.tru_id3 = self.tru_id3 - (self.tru_id3 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev
        elif jud == 0 and self.ne_id4 == n1 and 1 > self.tru_id4 >= self.tru_ths:
            self.tru_id4 = self.tru_id4 - (self.tru_id4 / (np.mean(self.tru_id1 + self.tru_id2 + self.tru_id3 +
                                                                   self.tru_id4)*2.0)) * ev

    def send(self, des):
        if des.id == 7 or des.id == 9:
            rand = random.randrange(-2, 1)
            if rand <= 0:
                return 0
            else:
                return 1
        if des.id == self.ne_id1 and self.tru_id1 > self.tru_ths:
            des.cur_size -= des.cos_speed
            des.cur_size += 100
            if des.cur_size > des.max_size:
                # print("solve10")
                des.cur_size = des.max_size
                return 0
            else:
                # print("solve11")
                return 1
        elif des.id == self.ne_id2 and self.tru_id2 > self.tru_ths:
            des.cur_size -= des.cos_speed
            des.cur_size += 100
            if des.cur_size > des.max_size:
                des.cur_size = des.max_size
                # print("solve20")
                return 0
            else:
                # print("solve21")
                return 1
        elif des.id == self.ne_id3 and self.tru_id3 > self.tru_ths:
            des.cur_size -= des.cos_speed
            des.cur_size += 100
            if des.cur_size > des.max_size:
                des.cur_size = des.max_size
                return 0
            else:
                return 1
        elif des.id == self.ne_id4 and self.tru_id4 > self.tru_ths:
            des.cur_size -= des.cos_speed
            des.cur_size += 100
            if des.cur_size > des.max_size:
                des.cur_size = des.max_size
                return 0
            else:
                return 1
        else:
            # print("solve-1")
            return -1

    def solve(self, sol):
        self.sol_size = random.randrange(60, 151)
        self.cur_size -= self.sol_size
        if self.cur_size < 0:
            self.cur_size = 0


class Task(object):
    def __init__(self, ind, task_size, task_time, task_flag):
        self.ind = ind
        self.task_size = task_size
        self.task_time = task_time
        self.task_flag = task_flag


class EnvDrones(object):
    def __init__(self, map_size, drone_num, view_range, tree_num, human_num):
        self.map_size = map_size
        self.drone_num = drone_num
        self.tree_num = tree_num
        self.human_num = human_num     # 定义地图大小、无人机数量、树的数量、人的数量
        self.n_actions = 4
        self.n_features = 3

        # initialize blocks and trees
        self.land_mark_map = np.zeros((self.map_size, self.map_size))
        # for i in range(self.map_size):
        #     for j in range(self.map_size):
        #         if random.random() < 0.01:   # 生成0~1的浮点型
        #             self.land_mark_map[i, j] = 1    # wall

        # initialize tree
        # for i in range(self.tree_num):
        #     temp_pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]
        #     while self.land_mark_map[temp_pos[0], temp_pos[1]] != 0:
        #         temp_pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]  # 重置坐标
        #     self.land_mark_map[temp_pos[0], temp_pos[1]] = 2

        # initialize humans
        self.human_list = []
        self.hop_collection2 = []
        self.hop_count = 0
        self.tru_value = 1
        self.e_c = 1
        num_j = 4
        for i in range(self.human_num):
            temp_pos = [random.randint(0, self.map_size - 1), random.randint(0, self.map_size - 1)]
            while self.land_mark_map[temp_pos[0], temp_pos[1]] != 0:
                temp_pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]   # 重置坐标
            if i < 4:    # node 0 1 2 3
                temp_human = Human(temp_pos, i, 1200, 200, 100, num_j, num_j + 1, num_j + 2, num_j + 3)
                num_j += 4
                self.human_list.append(temp_human)
            else:
                temp_human = Human(temp_pos, i, 1200, 200, 50, 0, 0, 0, 0)
                self.human_list.append(temp_human)
        # for i in range(self.human_num):
        #     short_d = 20000
        #     for j in range(self.human_num):
        #         if i == j:
        #             continue
        #         temp_d = math.sqrt(((self.human_list[i].pos[0] - self.human_list[j].pos[0])**2 +
        #                             (self.human_list[i].pos[1] - self.human_list[j].pos[1])**2))
        #         if short_d > temp_d:
        #             short_d = temp_d
        #             self.human_list[i].ne_id1 = j
        #     short_d = 20000
        #     for k in range(self.human_num):
        #         if i == k or self.human_list[i].ne_id1 == k:
        #             continue
        #         temp_d = math.sqrt(((self.human_list[i].pos[0] - self.human_list[k].pos[0])**2 +
        #                             (self.human_list[i].pos[1] - self.human_list[k].pos[1])**2))
        #         if short_d > temp_d:
        #             short_d = temp_d
        #             self.human_list[i].ne_id2 = k
        # self.human_list[6].ne_id1 = 20
        # self.human_list[6].flag = 0
        #
        # self.human_list[20].ne_id1 = 21
        # self.human_list[20].flag = 0
        #
        # self.human_list[21].ne_id1 = 22
        # self.human_list[21].flag = 0

        self.human_list[6].max_size = 1200
        self.human_list[6].cur_size = 100
        self.human_list[6].cos_speed = 100
        for i in range(4):
            self.human_list[i].max_size = 1200
            self.human_list[i].cur_size = 200
            self.human_list[i].cos_speed = 100
        for i in range(4, 20):
            if i == 6:
                continue
            self.human_list[i].max_size = 800
            self.human_list[i].cur_size = 100
            self.human_list[i].cos_speed = 50
        self.human_list[10].max_size = 1200
        self.human_list[10].cur_size = 500
        self.human_list[10].cos_speed = 500

        # initialize drones
        self.start_pos = [50, 50]
        self.drone_list = []
        for i in range(drone_num):
            temp_drone = Drones(self.start_pos, view_range)
            self.drone_list.append(temp_drone)

    def get_full_obs(self):     # 全局观测
        obs = np.ones((self.map_size, self.map_size, 3))
        for i in range(self.map_size):
            for j in range(self.map_size):
                if self.land_mark_map[i, j] == 1:    # wall  [0, 0, 0]
                    obs[i, j, 0] = 0
                    obs[i, j, 1] = 0
                    obs[i, j, 2] = 0
                if self.land_mark_map[i, j] == 2:    # tree  [0, 1, 0]
                    obs[i, j, 0] = 0
                    obs[i, j, 1] = 1
                    obs[i, j, 2] = 0

        for i in range(self.human_num):
            obs[self.human_list[i].pos[0], self.human_list[i].pos[1], 0] = 1    # person [1, 0, 0]
            obs[self.human_list[i].pos[0], self.human_list[i].pos[1], 1] = 0
            obs[self.human_list[i].pos[0], self.human_list[i].pos[1], 2] = 0
        return obs

    def get_drone_obs(self, drone):   # 无人机观测
        obs_size = 2 * drone.view_range - 1
        obs = np.ones((obs_size, obs_size, 3))
        for i in range(obs_size):
            for j in range(obs_size):
                x = i + drone.pos[0] - drone.view_range + 1
                y = j + drone.pos[1] - drone.view_range + 1

                for k in range(self.human_num):
                    if self.human_list[k].pos[0] == x and self.human_list[k].pos[1] == y:
                        obs[i, j, 0] = 1
                        obs[i, j, 1] = 0
                        obs[i, j, 2] = 0
                if 0 <= x <= self.map_size - 1 and 0 <= y <= self.map_size - 1:
                    if self.land_mark_map[x, y] == 1:
                        obs[i, j, 0] = 0
                        obs[i, j, 1] = 0
                        obs[i, j, 2] = 0
                    if self.land_mark_map[x, y] == 2:
                        obs[i, j, 0] = 0
                        obs[i, j, 1] = 1
                        obs[i, j, 2] = 0
                else:
                    obs[i, j, 0] = 0.5
                    obs[i, j, 1] = 0.5
                    obs[i, j, 2] = 0.5

                if (drone.view_range - 1 - i)*(drone.view_range - 1 - i)+(drone.view_range - 1 - j)*(drone.view_range - 1 - j) > drone.view_range*drone.view_range:
                    obs[i, j, 0] = 0.5
                    obs[i, j, 1] = 0.5
                    obs[i, j, 2] = 0.5

        return obs

    def get_joint_obs(self):
        obs = np.ones((self.map_size, self.map_size, 3))
        for i in range(self.map_size):
            for j in range(self.map_size):
                obs[i, j, 0] = 0.5
                obs[i, j, 1] = 0.5
                obs[i, j, 2] = 0.5
        for k in range(self.drone_num):
            temp = self.get_drone_obs(self.drone_list[k])
            size = temp.shape[0]
            for i in range(size):
                for j in range(size):
                    x = i + self.drone_list[k].pos[0] - self.drone_list[k].view_range + 1
                    y = j + self.drone_list[k].pos[1] - self.drone_list[k].view_range + 1
                    if_obs = True
                    if temp[i, j, 0] == 0.5 and temp[i, j, 1] == 0.5 and temp[i, j, 2] == 0.5:
                        if_obs = False
                    if if_obs is True:
                        obs[x, y, 0] = temp[i, j, 0]
                        obs[x, y, 1] = temp[i, j, 1]
                        obs[x, y, 2] = temp[i, j, 2]
        return obs

    def rand_reset_drone_pos(self):
        for k in range(self.drone_num):
            self.drone_list[k].pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]

    def drone_step(self, drone_act_list):
        if len(drone_act_list) != self.drone_num:
            return
        for k in range(self.drone_num):
            if drone_act_list[k] == 0:
                if self.drone_list[k].pos[0] > 0:
                    self.drone_list[k].pos[0] = self.drone_list[k].pos[0] - 1
            elif drone_act_list[k] == 1:
                if self.drone_list[k].pos[0] < self.map_size - 1:
                    self.drone_list[k].pos[0] = self.drone_list[k].pos[0] + 1
            elif drone_act_list[k] == 2:
                if self.drone_list[k].pos[1] > 0:
                    self.drone_list[k].pos[1] = self.drone_list[k].pos[1] - 1
            elif drone_act_list[k] == 3:
                if self.drone_list[k].pos[1] < self.map_size - 1:
                    self.drone_list[k].pos[1] = self.drone_list[k].pos[1] + 1

    def human_step(self, human_act_list):
        if len(human_act_list) != self.human_num:
            return
        for k in range(self.human_num):
            if human_act_list[k] == 0:
                if self.human_list[k].pos[0] > 0:
                    free_space = self.land_mark_map[self.human_list[k].pos[0] - 1, self.human_list[k].pos[1]]
                    if free_space == 0:
                        self.human_list[k].pos[0] = self.human_list[k].pos[0] - 1
            elif human_act_list[k] == 1:
                if self.human_list[k].pos[0] < self.map_size - 1:
                    free_space = self.land_mark_map[self.human_list[k].pos[0] + 1, self.human_list[k].pos[1]]
                    if free_space == 0:
                        self.human_list[k].pos[0] = self.human_list[k].pos[0] + 1
            elif human_act_list[k] == 2:
                if self.human_list[k].pos[1] > 0:
                    free_space = self.land_mark_map[self.human_list[k].pos[0], self.human_list[k].pos[1] - 1]
                    if free_space == 0:
                        self.human_list[k].pos[1] = self.human_list[k].pos[1] - 1
            elif human_act_list[k] == 3:
                if self.human_list[k].pos[1] < self.map_size - 1:
                    free_space = self.land_mark_map[self.human_list[k].pos[0], self.human_list[k].pos[1] + 1]
                    if free_space == 0:
                        self.human_list[k].pos[1] = self.human_list[k].pos[1] + 1

    def human_trust(self, count):
        self.e_c = count
        while count > 0:
            for i in range(self.human_num):
                # print(self.human_list[i].ne_id1, self.human_list[i].ne_id2)
                self.human_list[i].compu(self.human_list[i].ne_id1, 0.1, self.human_list[i].send(
                    self.human_list[self.human_list[i].ne_id1]))
                self.human_list[i].compu(self.human_list[i].ne_id2, 0.1, self.human_list[i].send(
                    self.human_list[self.human_list[i].ne_id2]))
                self.human_list[i].compu(self.human_list[i].ne_id3, 0.1, self.human_list[i].send(
                    self.human_list[self.human_list[i].ne_id3]))
                self.human_list[i].compu(self.human_list[i].ne_id4, 0.1, self.human_list[i].send(
                    self.human_list[self.human_list[i].ne_id4]))
    #         print(self.human_list[2].cur_size, self.human_list[2].sol_size)
    #         print("Node1: %.2f,%.2f, Node2: %.2f,%.2f, Node3: %.2f,%.2f, Node4: %.2f,%.2f\n"
    #               "Node5: %.2f,%.2f, Node6: %.2f,%.2f, Node7: %.2f,%.2f, Node8: %.2f,%.2f\n"
    #               "Node9: %.2f,%.2f, Node10: %.2f,%.2f," % (self.human_list[0].tru_id1, self.human_list[0].tru_id2,
    #                                                         self.human_list[1].tru_id1, self.human_list[1].tru_id2,
    #                                                         self.human_list[2].tru_id1, self.human_list[2].tru_id2,
    #                                                         self.human_list[3].tru_id1, self.human_list[3].tru_id2,
    #                                                         self.human_list[4].tru_id1, self.human_list[4].tru_id2,
    #                                                         self.human_list[5].tru_id1, self.human_list[5].tru_id2,
    #                                                         self.human_list[6].tru_id1, self.human_list[6].tru_id2,
    #                                                         self.human_list[7].tru_id1, self.human_list[7].tru_id2,
    #                                                         self.human_list[8].tru_id1, self.human_list[8].tru_id2,
    #                                                         self.human_list[9].tru_id1, self.human_list[9].tru_id2))
    #         print("------------------------")
            count -= 1

    # def step(self, human_act_list):    # def step(self, human_act_list, drone_act_list)
    #     # self.drone_step(drone_act_list)
    #     for i in range(3):
    #         self.human_step(human_act_list)
    #     self.human_trust(1)

    def reset(self):
        self.task_test = Task(1, 2000, 10, 0)
        self.human_list.clear()
        self.tru_value = 1
        self.hop_count = 0
        num_j = 4
        for i in range(self.human_num):
            temp_pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]
            while self.land_mark_map[temp_pos[0], temp_pos[1]] != 0:
                temp_pos = [random.randint(0, self.map_size-1), random.randint(0, self.map_size-1)]   # 重置坐标
            if i < 4:    # node 0 1 2 3
                temp_human = Human(temp_pos, i, 1200, 200, 100, num_j, num_j + 1, num_j + 2, num_j + 3)
                num_j += 4
                self.human_list.append(temp_human)
            else:
                temp_human = Human(temp_pos, i, 1200, 200, 50, 0, 0, 0, 0)
                self.human_list.append(temp_human)
        # self.human_list[6].ne_id1 = 20
        # self.human_list[6].flag = 0
        #
        # self.human_list[20].ne_id1 = 21
        # self.human_list[20].flag = 0
        #
        # self.human_list[21].ne_id1 = 22
        # self.human_list[21].flag = 0

        self.human_list[6].max_size = 1200
        self.human_list[6].cur_size = 100
        self.human_list[6].cos_speed = 100

        for i in range(4):
            self.human_list[i].max_size = 1200
            self.human_list[i].cur_size = 200
            self.human_list[i].cos_speed = 100
        for i in range(4, 20):    # 20有改变
            if i == 6:
                continue
            self.human_list[i].max_size = 800
            self.human_list[i].cur_size = 100
            self.human_list[i].cos_speed = 50
        self.human_list[10].max_size = 1200
        self.human_list[10].cur_size = 500
        self.human_list[10].cos_speed = 500
        # for j in range(int(self.task_test.task_size * 10000 / self.human_num)):
        #     view_node.clear()
        # 增加设备数
        # for i in range(20, 30):
        #     if i == 20 or i == 22:
        #         continue
        #     self.human_list[i].max_size = 300
        # self.human_list[9].ne_id1 = 20
        # self.human_list[9].ne_id2 = 21
        # self.human_list[10].ne_id1 = 22
        # self.human_list[10].ne_id2 = 23
        # self.human_list[11].ne_id1 = 24
        # self.human_list[11].ne_id2 = 25
        # self.human_list[12].ne_id1 = 26
        # self.human_list[12].ne_id2 = 27
        # self.human_list[13].ne_id1 = 28
        # self.human_list[13].ne_id2 = 29
        # for i in range(9, 14):
        #     self.human_list[i].flag = 0
        return np.array([self.task_test.task_size, self.task_test.task_time, (1 - self.tru_value)])

    def step(self, action):
        global pre_action
        global view_node
        global hop_collection
        flag = 1
        s = np.array([self.task_test.task_size, self.task_test.task_time, (1 - self.tru_value)])
        base_action = s
        if action == 0 and len(pre_action) == 0:  # 0
            if (self.task_test.task_time * self.human_list[0].cos_speed) <= (self.human_list[0].max_size -
                                                                             self.human_list[0].cur_size):
                base_action[0] -= self.task_test.task_time * self.human_list[0].cos_speed
                self.human_list[0].cur_size += self.task_test.task_time * self.human_list[0].cos_speed
            else:
                base_action[0] -= self.human_list[0].max_size - self.human_list[0].cur_size
                self.human_list[0].cur_size += self.human_list[0].max_size - self.human_list[0].cur_size
            view_node.append(0)
            pre_action.append(action)
            self.tru_value = 0.85
        elif action == 0 and len(pre_action) == 1 and pre_action[0] == 0:   # 00 4
            if (self.human_list[self.human_list[0].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[0].ne_id1].max_size -
                 self.human_list[self.human_list[0].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[0].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[0].ne_id1].cur_size += \
                    self.human_list[self.human_list[0].ne_id1].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[0].ne_id1].max_size - \
                                  self.human_list[self.human_list[0].ne_id1].cur_size
                self.human_list[self.human_list[0].ne_id1].cur_size = self.human_list[self.human_list[0].ne_id1].max_size
            view_node.append(4)
            pre_action.append(action)
            self.tru_value = self.human_list[0].tru_id1
        elif action == 1 and len(pre_action) == 1 and pre_action[0] == 0:  # 01 5
            if (self.human_list[self.human_list[0].ne_id2].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[0].ne_id2].max_size -
                 self.human_list[self.human_list[0].ne_id2].cur_size):
                base_action[0] -= self.human_list[self.human_list[0].ne_id2].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[0].ne_id2].cur_size += \
                    self.human_list[self.human_list[0].ne_id2].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[0].ne_id2].max_size - \
                                  self.human_list[self.human_list[0].ne_id2].cur_size
                self.human_list[self.human_list[0].ne_id2].cur_size = self.human_list[self.human_list[0].ne_id2].max_size
            view_node.append(5)
            pre_action.append(action)
            self.tru_value = self.human_list[0].tru_id2
        elif action == 2 and len(pre_action) == 1 and pre_action[0] == 0:  # 02 6
            if (self.human_list[self.human_list[0].ne_id3].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[0].ne_id3].max_size -
                 self.human_list[self.human_list[0].ne_id3].cur_size):
                base_action[0] -= self.human_list[self.human_list[0].ne_id3].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[0].ne_id3].cur_size += \
                    self.human_list[self.human_list[0].ne_id3].cos_speed * self.task_test.task_time
            elif (self.human_list[self.human_list[0].ne_id3].cos_speed * self.task_test.task_time) >= base_action[0] \
                    and base_action[0] <= self.human_list[self.human_list[0].ne_id3].max_size - \
                    self.human_list[self.human_list[0].ne_id3].cur_size:
                base_action[0] -= self.human_list[self.human_list[0].ne_id3].max_size - \
                                  self.human_list[self.human_list[0].ne_id3].cur_size
                self.human_list[self.human_list[0].ne_id3].cur_size = self.human_list[self.human_list[0].ne_id3].max_size
            view_node.append(6)
            pre_action.append(action)
            self.tru_value = self.human_list[0].tru_id3
        elif action == 0 and len(pre_action) == 2 and pre_action[0] == 0 and pre_action[1] == 2:
            if (self.human_list[self.human_list[6].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[6].ne_id1].max_size -
                 self.human_list[self.human_list[6].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[6].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[6].ne_id1].cur_size += \
                    self.human_list[self.human_list[6].ne_id1].cos_speed * self.task_test.task_time
            elif (self.human_list[self.human_list[6].ne_id1].cos_speed * self.task_test.task_time) >= base_action[0] \
                    and base_action[0] <= self.human_list[self.human_list[6].ne_id1].max_size - \
                    self.human_list[self.human_list[6].ne_id1].cur_size:
                base_action[0] -= self.human_list[self.human_list[6].ne_id1].max_size - \
                                  self.human_list[self.human_list[6].ne_id1].cur_size
                self.human_list[self.human_list[6].ne_id1].cur_size = self.human_list[self.human_list[6].ne_id1].max_size
            view_node.append(20)
            pre_action.append(action)
            self.tru_value = self.human_list[19].tru_id1
        elif action == 0 and len(pre_action) == 3 and pre_action[0] == 0 and pre_action[1] == 2 and pre_action[2] == 0:
            if (self.human_list[self.human_list[20].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[20].ne_id1].max_size -
                 self.human_list[self.human_list[20].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[20].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[20].ne_id1].cur_size += \
                    self.human_list[self.human_list[20].ne_id1].cos_speed * self.task_test.task_time
            elif (self.human_list[self.human_list[20].ne_id1].cos_speed * self.task_test.task_time) >= base_action[0] \
                    and base_action[0] <= self.human_list[self.human_list[20].ne_id1].max_size - \
                    self.human_list[self.human_list[20].ne_id1].cur_size:
                base_action[0] -= self.human_list[self.human_list[20].ne_id1].max_size - \
                                  self.human_list[self.human_list[20].ne_id1].cur_size
                self.human_list[self.human_list[20].ne_id1].cur_size = self.human_list[self.human_list[20].ne_id1].max_size
            view_node.append(21)
            pre_action.append(action)
        elif action == 0 and len(pre_action) == 4 and pre_action[0] == 0 and pre_action[1] == 2 and \
                pre_action[2] == 0 and pre_action[3] == 0:
            if (self.human_list[self.human_list[21].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[21].ne_id1].max_size -
                 self.human_list[self.human_list[21].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[21].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[21].ne_id1].cur_size += \
                    self.human_list[self.human_list[21].ne_id1].cos_speed * self.task_test.task_time
            elif (self.human_list[self.human_list[21].ne_id1].cos_speed * self.task_test.task_time) >= base_action[0] \
                    and base_action[0] <= self.human_list[self.human_list[21].ne_id1].max_size - \
                    self.human_list[self.human_list[21].ne_id1].cur_size:
                base_action[0] -= self.human_list[self.human_list[21].ne_id1].max_size - \
                                  self.human_list[self.human_list[21].ne_id1].cur_size
                self.human_list[self.human_list[21].ne_id1].cur_size = self.human_list[self.human_list[21].ne_id1].max_size
            view_node.append(22)
            pre_action.append(action)
        elif action == 3 and len(pre_action) == 1 and pre_action[0] == 0:  # 03 7
            if (self.human_list[self.human_list[0].ne_id4].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[0].ne_id4].max_size -
                 self.human_list[self.human_list[0].ne_id4].cur_size):
                base_action[0] -= self.human_list[self.human_list[0].ne_id4].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[0].ne_id4].cur_size += \
                    self.human_list[self.human_list[0].ne_id4].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[0].ne_id4].max_size - \
                                  self.human_list[self.human_list[0].ne_id4].cur_size
                self.human_list[self.human_list[0].ne_id4].cur_size = self.human_list[self.human_list[0].ne_id4].max_size
            view_node.append(7)
            pre_action.append(action)
            self.tru_value = self.human_list[0].tru_id4
###
        elif action == 1 and len(pre_action) == 0:  # 1
            if (self.task_test.task_time * self.human_list[1].cos_speed) <= (self.human_list[1].max_size -
                                                                             self.human_list[1].cur_size):
                base_action[0] -= self.task_test.task_time * self.human_list[1].cos_speed
                self.human_list[1].cur_size += self.task_test.task_time * self.human_list[1].cos_speed
            else:
                base_action[0] -= self.human_list[1].max_size - self.human_list[1].cur_size
                self.human_list[1].cur_size += self.human_list[1].max_size - self.human_list[1].cur_size
            view_node.append(1)
            pre_action.append(action)
            self.tru_value = 0.85
        elif action == 0 and len(pre_action) == 1 and pre_action[0] == 1:   # 10 8
            if (self.human_list[self.human_list[1].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[1].ne_id1].max_size -
                 self.human_list[self.human_list[1].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[1].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[1].ne_id1].cur_size += \
                    self.human_list[self.human_list[1].ne_id1].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[1].ne_id1].max_size - \
                                  self.human_list[self.human_list[1].ne_id1].cur_size
                self.human_list[self.human_list[1].ne_id1].cur_size = self.human_list[self.human_list[1].ne_id1].max_size
            view_node.append(8)
            pre_action.append(action)
            self.tru_value = self.human_list[1].tru_id1
        elif action == 1 and len(pre_action) == 1 and pre_action[0] == 1:  # 11 9
            if (self.human_list[self.human_list[1].ne_id2].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[1].ne_id2].max_size -
                 self.human_list[self.human_list[1].ne_id2].cur_size):
                base_action[0] -= self.human_list[self.human_list[1].ne_id2].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[1].ne_id2].cur_size += \
                    self.human_list[self.human_list[1].ne_id2].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[1].ne_id2].max_size - \
                                  self.human_list[self.human_list[1].ne_id2].cur_size
                self.human_list[self.human_list[1].ne_id2].cur_size = self.human_list[self.human_list[1].ne_id2].max_size
            view_node.append(9)
            pre_action.append(action)
            self.tru_value = self.human_list[1].tru_id2
        elif action == 2 and len(pre_action) == 1 and pre_action[0] == 1:  # 12 10
            if (self.human_list[self.human_list[1].ne_id3].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[1].ne_id3].max_size -
                 self.human_list[self.human_list[1].ne_id3].cur_size):
                base_action[0] -= self.human_list[self.human_list[1].ne_id3].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[1].ne_id3].cur_size += \
                    self.human_list[self.human_list[1].ne_id3].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[1].ne_id3].max_size - \
                                  self.human_list[self.human_list[1].ne_id3].cur_size
                self.human_list[self.human_list[1].ne_id3].cur_size = self.human_list[self.human_list[1].ne_id3].max_size
            view_node.append(10)
            pre_action.append(action)
            self.tru_value = self.human_list[1].tru_id3
        elif action == 3 and len(pre_action) == 1 and pre_action[0] == 1:  # 13 11
            if (self.human_list[self.human_list[1].ne_id4].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[1].ne_id4].max_size -
                 self.human_list[self.human_list[1].ne_id4].cur_size):
                base_action[0] -= self.human_list[self.human_list[1].ne_id4].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[1].ne_id4].cur_size += \
                    self.human_list[self.human_list[1].ne_id4].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[1].ne_id4].max_size - \
                                  self.human_list[self.human_list[1].ne_id4].cur_size
                self.human_list[self.human_list[1].ne_id4].cur_size = self.human_list[self.human_list[1].ne_id4].max_size
            view_node.append(11)
            pre_action.append(action)
            self.tru_value = self.human_list[1].tru_id4

        # # 添加设备
        # elif action == 0 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 1:  # 110 20
        #     if (self.human_list[self.human_list[9].ne_id1].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[9].ne_id1].max_size -
        #          self.human_list[self.human_list[9].ne_id1].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[9].ne_id1].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[9].ne_id1].cur_size += \
        #             self.human_list[self.human_list[9].ne_id1].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[9].ne_id1].max_size - \
        #                           self.human_list[self.human_list[9].ne_id1].cur_size
        #         self.human_list[self.human_list[9].ne_id1].cur_size = self.human_list[self.human_list[9].ne_id1].max_size
        #     view_node.append(20)
        #     pre_action.append(action)
        # elif action == 1 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 1:  # 111 21
        #     if (self.human_list[self.human_list[9].ne_id2].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[9].ne_id2].max_size -
        #          self.human_list[self.human_list[9].ne_id2].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[9].ne_id2].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[9].ne_id2].cur_size += \
        #             self.human_list[self.human_list[9].ne_id2].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[9].ne_id2].max_size - \
        #                           self.human_list[self.human_list[9].ne_id2].cur_size
        #         self.human_list[self.human_list[9].ne_id2].cur_size = self.human_list[self.human_list[9].ne_id2].max_size
        #     view_node.append(21)
        #     pre_action.append(action)
        # elif action == 0 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 2:  # 120 22
        #     if (self.human_list[self.human_list[10].ne_id1].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[10].ne_id1].max_size -
        #          self.human_list[self.human_list[10].ne_id1].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[10].ne_id1].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[10].ne_id1].cur_size += \
        #             self.human_list[self.human_list[10].ne_id1].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[10].ne_id1].max_size - \
        #                           self.human_list[self.human_list[10].ne_id1].cur_size
        #         self.human_list[self.human_list[10].ne_id1].cur_size = self.human_list[self.human_list[10].ne_id1].max_size
        #     view_node.append(22)
        #     pre_action.append(action)
        # elif action == 1 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 2:  # 121 23
        #     if (self.human_list[self.human_list[10].ne_id2].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[10].ne_id2].max_size -
        #          self.human_list[self.human_list[10].ne_id2].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[10].ne_id2].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[10].ne_id2].cur_size += \
        #             self.human_list[self.human_list[10].ne_id2].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[10].ne_id2].max_size - \
        #                           self.human_list[self.human_list[10].ne_id2].cur_size
        #         self.human_list[self.human_list[10].ne_id2].cur_size = self.human_list[self.human_list[10].ne_id2].max_size
        #     view_node.append(23)
        #     pre_action.append(action)
        # elif action == 0 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 3:  # 130 24
        #     if (self.human_list[self.human_list[11].ne_id1].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[11].ne_id1].max_size -
        #          self.human_list[self.human_list[11].ne_id1].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[11].ne_id1].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[11].ne_id1].cur_size += \
        #             self.human_list[self.human_list[11].ne_id1].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[11].ne_id1].max_size - \
        #                           self.human_list[self.human_list[11].ne_id1].cur_size
        #         self.human_list[self.human_list[11].ne_id1].cur_size = self.human_list[self.human_list[11].ne_id1].max_size
        #     view_node.append(24)
        #     pre_action.append(action)
        # elif action == 1 and len(pre_action) == 2 and pre_action[0] == 1 and pre_action[1] == 3:  # 131 25
        #     if (self.human_list[self.human_list[11].ne_id2].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[11].ne_id2].max_size -
        #          self.human_list[self.human_list[11].ne_id2].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[11].ne_id2].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[11].ne_id2].cur_size += \
        #             self.human_list[self.human_list[11].ne_id2].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[11].ne_id2].max_size - \
        #                           self.human_list[self.human_list[11].ne_id2].cur_size
        #         self.human_list[self.human_list[11].ne_id2].cur_size = self.human_list[self.human_list[11].ne_id2].max_size
        #     view_node.append(25)
        #     pre_action.append(action)
        # # 20 21 22 23 24 25

###
        elif action == 2 and len(pre_action) == 0:  # 2
            if (self.task_test.task_time * self.human_list[2].cos_speed) <= (self.human_list[2].max_size -
                                                                             self.human_list[2].cur_size):
                base_action[0] -= self.task_test.task_time * self.human_list[2].cos_speed
                self.human_list[2].cur_size += self.task_test.task_time * self.human_list[2].cos_speed
            else:
                base_action[0] -= self.human_list[2].max_size - self.human_list[2].cur_size
                self.human_list[2].cur_size += self.human_list[2].max_size - self.human_list[2].cur_size
            view_node.append(2)
            pre_action.append(action)
            self.tru_value = 0.85
        elif action == 0 and len(pre_action) == 1 and pre_action[0] == 2:   # 20 12
            if (self.human_list[self.human_list[2].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[2].ne_id1].max_size -
                 self.human_list[self.human_list[2].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[2].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[2].ne_id1].cur_size += \
                    self.human_list[self.human_list[2].ne_id1].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[2].ne_id1].max_size - \
                                  self.human_list[self.human_list[2].ne_id1].cur_size
                self.human_list[self.human_list[2].ne_id1].cur_size = self.human_list[self.human_list[2].ne_id1].max_size
            view_node.append(12)
            pre_action.append(action)
            self.tru_value = self.human_list[2].tru_id1
        elif action == 1 and len(pre_action) == 1 and pre_action[0] == 2:  # 21 13
            if (self.human_list[self.human_list[2].ne_id2].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[2].ne_id2].max_size -
                 self.human_list[self.human_list[2].ne_id2].cur_size):
                base_action[0] -= self.human_list[self.human_list[2].ne_id2].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[2].ne_id2].cur_size += \
                    self.human_list[self.human_list[2].ne_id2].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[2].ne_id2].max_size - \
                                  self.human_list[self.human_list[2].ne_id2].cur_size
                self.human_list[self.human_list[2].ne_id2].cur_size = self.human_list[self.human_list[2].ne_id2].max_size
            view_node.append(13)
            pre_action.append(action)
            self.tru_value = self.human_list[2].tru_id2
        elif action == 2 and len(pre_action) == 1 and pre_action[0] == 2:  # 22 14
            if (self.human_list[self.human_list[2].ne_id3].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[2].ne_id3].max_size -
                 self.human_list[self.human_list[2].ne_id3].cur_size):
                base_action[0] -= self.human_list[self.human_list[2].ne_id3].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[2].ne_id3].cur_size += \
                    self.human_list[self.human_list[2].ne_id3].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[2].ne_id3].max_size - \
                                  self.human_list[self.human_list[2].ne_id3].cur_size
                self.human_list[self.human_list[2].ne_id3].cur_size = self.human_list[self.human_list[2].ne_id3].max_size
            view_node.append(14)
            pre_action.append(action)
            self.tru_value = self.human_list[2].tru_id3
        elif action == 3 and len(pre_action) == 1 and pre_action[0] == 2:  # 23 15
            if (self.human_list[self.human_list[2].ne_id4].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[2].ne_id4].max_size -
                 self.human_list[self.human_list[2].ne_id4].cur_size):
                base_action[0] -= self.human_list[self.human_list[2].ne_id4].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[2].ne_id4].cur_size += \
                    self.human_list[self.human_list[2].ne_id4].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[2].ne_id4].max_size - \
                                  self.human_list[self.human_list[2].ne_id4].cur_size
                self.human_list[self.human_list[2].ne_id4].cur_size = self.human_list[self.human_list[2].ne_id4].max_size
            view_node.append(15)
            pre_action.append(action)
            self.tru_value = self.human_list[2].tru_id4

        # # 增加节点
        # elif action == 0 and len(pre_action) == 2 and pre_action[0] == 2 and pre_action[1] == 0:  # 200 26
        #     if (self.human_list[self.human_list[12].ne_id1].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[12].ne_id1].max_size -
        #          self.human_list[self.human_list[12].ne_id1].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[12].ne_id1].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[12].ne_id1].cur_size += \
        #             self.human_list[self.human_list[12].ne_id1].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[12].ne_id1].max_size - \
        #                           self.human_list[self.human_list[12].ne_id1].cur_size
        #         self.human_list[self.human_list[12].ne_id1].cur_size = self.human_list[self.human_list[12].ne_id1].max_size
        #     view_node.append(26)
        #     pre_action.append(action)
        # elif action == 1 and len(pre_action) == 2 and pre_action[0] == 2 and pre_action[1] == 0:  # 201 27
        #     if (self.human_list[self.human_list[12].ne_id2].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[12].ne_id2].max_size -
        #          self.human_list[self.human_list[12].ne_id2].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[12].ne_id2].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[12].ne_id2].cur_size += \
        #             self.human_list[self.human_list[12].ne_id2].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[12].ne_id2].max_size - \
        #                           self.human_list[self.human_list[12].ne_id2].cur_size
        #         self.human_list[self.human_list[12].ne_id2].cur_size = self.human_list[self.human_list[12].ne_id2].max_size
        #     view_node.append(27)
        #     pre_action.append(action)
        # elif action == 0 and len(pre_action) == 2 and pre_action[0] == 2 and pre_action[1] == 1:  # 210 28
        #     if (self.human_list[self.human_list[13].ne_id1].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[13].ne_id1].max_size -
        #          self.human_list[self.human_list[13].ne_id1].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[13].ne_id1].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[13].ne_id1].cur_size += \
        #             self.human_list[self.human_list[13].ne_id1].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[13].ne_id1].max_size - \
        #                           self.human_list[self.human_list[13].ne_id1].cur_size
        #         self.human_list[self.human_list[13].ne_id1].cur_size = self.human_list[self.human_list[13].ne_id1].max_size
        #     view_node.append(28)
        #     pre_action.append(action)
        # elif action == 1 and len(pre_action) == 2 and pre_action[0] == 2 and pre_action[1] == 1:  # 211 29
        #     if (self.human_list[self.human_list[13].ne_id2].cos_speed * self.task_test.task_time) <= \
        #         (self.human_list[self.human_list[13].ne_id2].max_size -
        #          self.human_list[self.human_list[13].ne_id2].cur_size):
        #         base_action[0] -= self.human_list[self.human_list[13].ne_id2].cos_speed * self.task_test.task_time
        #         self.human_list[self.human_list[13].ne_id2].cur_size += \
        #             self.human_list[self.human_list[13].ne_id2].cos_speed * self.task_test.task_time
        #     else:
        #         base_action[0] -= self.human_list[self.human_list[13].ne_id2].max_size - \
        #                           self.human_list[self.human_list[13].ne_id2].cur_size
        #         self.human_list[self.human_list[13].ne_id2].cur_size = self.human_list[self.human_list[13].ne_id2].max_size
        #     view_node.append(29)
        #     pre_action.append(action)
        # # 26 27 28 29

###
        elif action == 3 and len(pre_action) == 0:  # 3
            if (self.task_test.task_time * self.human_list[3].cos_speed) <= (self.human_list[3].max_size -
                                                                             self.human_list[3].cur_size):
                base_action[0] -= self.task_test.task_time * self.human_list[3].cos_speed
                self.human_list[3].cur_size += self.task_test.task_time * self.human_list[3].cos_speed
            else:
                base_action[0] -= self.human_list[3].max_size - self.human_list[3].cur_size
                self.human_list[3].cur_size += self.human_list[3].max_size - self.human_list[3].cur_size
            view_node.append(3)
            pre_action.append(action)
            self.tru_value = 0.85
        elif action == 0 and len(pre_action) == 1 and pre_action[0] == 3:   # 30 16
            if (self.human_list[self.human_list[3].ne_id1].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[3].ne_id1].max_size -
                 self.human_list[self.human_list[3].ne_id1].cur_size):
                base_action[0] -= self.human_list[self.human_list[3].ne_id1].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[3].ne_id1].cur_size += \
                    self.human_list[self.human_list[3].ne_id1].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[3].ne_id1].max_size - \
                                  self.human_list[self.human_list[3].ne_id1].cur_size
                self.human_list[self.human_list[3].ne_id1].cur_size = self.human_list[self.human_list[3].ne_id1].max_size
            view_node.append(16)
            pre_action.append(action)
            self.tru_value = self.human_list[3].tru_id1
        elif action == 1 and len(pre_action) == 1 and pre_action[0] == 3:  # 31 17
            if (self.human_list[self.human_list[3].ne_id2].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[3].ne_id2].max_size -
                 self.human_list[self.human_list[3].ne_id2].cur_size):
                base_action[0] -= self.human_list[self.human_list[3].ne_id2].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[3].ne_id2].cur_size += \
                    self.human_list[self.human_list[3].ne_id2].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[3].ne_id2].max_size - \
                                  self.human_list[self.human_list[3].ne_id2].cur_size
                self.human_list[self.human_list[3].ne_id2].cur_size = self.human_list[self.human_list[3].ne_id2].max_size
            view_node.append(17)
            pre_action.append(action)
            self.tru_value = self.human_list[3].tru_id2
        elif action == 2 and len(pre_action) == 1 and pre_action[0] == 3:  # 32 18
            if (self.human_list[self.human_list[3].ne_id3].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[3].ne_id3].max_size -
                 self.human_list[self.human_list[3].ne_id3].cur_size):
                base_action[0] -= self.human_list[self.human_list[3].ne_id3].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[3].ne_id3].cur_size += \
                    self.human_list[self.human_list[3].ne_id3].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[3].ne_id3].max_size - \
                                  self.human_list[self.human_list[3].ne_id3].cur_size
                self.human_list[self.human_list[3].ne_id3].cur_size = self.human_list[self.human_list[3].ne_id3].max_size
            view_node.append(18)
            pre_action.append(action)
            self.tru_value = self.human_list[3].tru_id3
        elif action == 3 and len(pre_action) == 1 and pre_action[0] == 3:  # 33 19
            if (self.human_list[self.human_list[3].ne_id4].cos_speed * self.task_test.task_time) <= \
                (self.human_list[self.human_list[3].ne_id4].max_size -
                 self.human_list[self.human_list[3].ne_id4].cur_size):
                base_action[0] -= self.human_list[self.human_list[3].ne_id4].cos_speed * self.task_test.task_time
                self.human_list[self.human_list[3].ne_id4].cur_size += \
                    self.human_list[self.human_list[3].ne_id4].cos_speed * self.task_test.task_time
            else:
                base_action[0] -= self.human_list[self.human_list[3].ne_id4].max_size - \
                                  self.human_list[self.human_list[3].ne_id4].cur_size
                self.human_list[self.human_list[3].ne_id4].cur_size = self.human_list[self.human_list[3].ne_id4].max_size
            view_node.append(19)
            pre_action.append(action)
            self.tru_value = self.human_list[3].tru_id4
        elif action == 1 and len(pre_action) == 2 and pre_action[0] == 0 and pre_action[1] == 2:
            pre_action.append(action)
            flag = 0.2
        elif action == 2 and len(pre_action) == 2 and pre_action[0] == 0 and pre_action[1] == 2:
            pre_action.append(action)
            flag = 0.2
        elif action == 3 and len(pre_action) == 2 and pre_action[0] == 0 and pre_action[1] == 2:
            pre_action.append(action)
            flag = 0.2
        elif action == 1 and len(pre_action) == 3 and pre_action[0] == 0 and pre_action[1] == 2 and pre_action[2] == 0:
            pre_action.append(action)
            flag = 0.3
        elif action == 2 and len(pre_action) == 3 and pre_action[0] == 0 and pre_action[1] == 2 and pre_action[2] == 0:
            pre_action.append(action)
            flag = 0.3
        elif action == 3 and len(pre_action) == 3 and pre_action[0] == 0 and pre_action[1] == 2 and pre_action[2] == 0:
            pre_action.append(action)
            flag = 0.3
        elif action == 1 and len(pre_action) == 4 and pre_action[0] == 0 and pre_action[1] == 2 and \
                pre_action[2] == 0 and pre_action[3] == 0:
            pre_action.append(action)
            flag = 0.4
        elif action == 2 and len(pre_action) == 4 and pre_action[0] == 0 and pre_action[1] == 2 and \
                pre_action[2] == 0 and pre_action[3] == 0:
            pre_action.append(action)
            flag = 0.4
        elif action == 3 and len(pre_action) == 4 and pre_action[0] == 0 and pre_action[1] == 2 and \
                pre_action[2] == 0 and pre_action[3] == 0:
            pre_action.append(action)
            flag = 0.4
        # self.canvas.move(self.rect, base_action[0], base_action[1])  # move agent

        # next_coords = self.canvas.coords(self.rect)  # next state
        # reward function
        # print(view_node, pre_action, base_action[0])
        if base_action[0] < 0:
            base_action[0] = 0
            base_action[1] = 0
        if base_action[0] == 0 and (len(pre_action) == 2 or len(pre_action) == 1 or len(pre_action) == 3 or
                                    len(pre_action) == 4 or len(pre_action) == 5):
            reward = 1
            err_re = 0
            enter_hop = 0
            # print(pre_action)
            view_node.clear()
            pre_action.clear()
            if self.hop_count != 0:
                self.hop_collection2.append(self.hop_count * (1 / self.e_c) + 1)
                self.hop_count = 0
            done = True
        elif len(pre_action) == 2 and ((pre_action[0] == 0 and pre_action[1] == 3) or (pre_action[0] == 1 and pre_action[1] == 1)):
            reward = -1
            err_re = 1
            enter_hop = 0
            view_node.clear()
            pre_action.clear()
            if self.hop_count != 0:
                self.hop_collection2.append(self.hop_count * (1 / self.e_c) + 1)
                self.hop_count = 0
            done = True
        elif len(pre_action) == 2 and (pre_action[0] == 1 and pre_action[1] == 2):
            reward = 1
            err_re = 0
            enter_hop = 1
            view_node.clear()
            pre_action.clear()
            if self.hop_count != 0:
                self.hop_collection2.append(self.hop_count * (1 / self.e_c) + 1)
                self.hop_count = 0
            done = True
        elif base_action[0] > 0 and len(pre_action) == 2:
            reward = -1
            err_re = 0
            enter_hop = 0
            view_node.clear()
            pre_action.clear()
            if self.hop_count != 0:
                self.hop_collection2.append(self.hop_count * (1 / self.e_c) + 1)
                self.hop_count = 0
            done = True
        # elif base_action[0] > 0 and (len(pre_action) == 3 or len(pre_action) == 2) and\
        #         self.human_list[view_node[1]].flag == 1:
        #     reward = -1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif base_action[0] > 0 and len(pre_action) == 4 and self.human_list[view_node[2]].flag == 1:
        #     reward = -1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif base_action[0] > 0 and len(pre_action) == 5 and self.human_list[view_node[3]].flag == 1:
        #     reward = -1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif flag == 0:
        #     reward = -1
        #     flag = 1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif flag == 0.2:
        #     reward = 0.2
        #     flag = 1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif flag == 0.3:
        #     reward = 0.3
        #     flag = 1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        # elif flag == 0.4:
        #     reward = 0.4
        #     flag = 1
        #     view_node.clear()
        #     pre_action.clear()
        #     done = True
        else:
            self.hop_count += 1
            reward = 0
            err_re = 0
            enter_hop = 0
            done = False
        self.task_test.task_size = base_action[0]
        self.task_test.task_time = base_action[1]
        s_ = np.array([self.task_test.task_size, self.task_test.task_time, (1 - self.tru_value)])
        return s_, reward, done, {}, err_re, enter_hop

    def render(self):
        pass







