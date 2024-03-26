import airsim
import time
import numpy as np

uav_nums = 9
origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]       # 无人机初始位置
origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]

def get_UAV_pos(client, vehicle_name="SimpleFlight"):   # 获取UAV位置
    global origin_x
    global origin_y
    state = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    # Get Ground truth kinematics of the vehicle 获取车辆的地面实况运动学
    x = state.position.x_val
    y = state.position.y_val
    i = int(vehicle_name[3])    # UAVi 无人机列队
    x += origin_x[i - 1]        # 移动位置写死
    y += origin_y[i - 1]
    pos = np.array([[x], [y]])      # 返回array类型的pos
    return pos


client = airsim.MultirotorClient()  # connect to the AirSim simulator
for i in range(uav_nums):
    name = "UAV"+str(i+1)
    client.enableApiControl(True, name)     # 开启Api控制UAVi
    client.armDisarm(True, name)            # 解锁螺旋桨
    if i != uav_nums-1:                              # 起飞
        client.takeoffAsync(vehicle_name=name)
    else:
        client.takeoffAsync(vehicle_name=name).join()
        # Async methods returns Future.
        # Call join() to wait for task to complete.

for i in range(uav_nums):                           # 全部都飞到同一高度层3m
    name = "UAV" + str(i+1)
    if i != uav_nums-1:
        client.moveToZAsync(-3, 1, vehicle_name=name)
        # moveToZAsync(z, velocity, vehicle_name)
        # 是高度控制 API，第一个参数是高度，第二个参数是速度，第三个是控制的目标
    else:
        client.moveToZAsync(-3, 1, vehicle_name=name).join()    # .join()


# 参数设置
v_max = 2     # 无人机最大飞行速度
r_max = 20    # 邻居选择半径
k_sep = 7     # 控制算法系数
k_coh = 1
k_mig = 1
pos_mig = np.array([[25], [0]])   # 目标位置
v_cmd = np.zeros([2, 9])

for t in range(500):
    for i in range(uav_nums):   # 计算每个无人机的速度指令
        name_i = "UAV"+str(i+1)
        pos_i = get_UAV_pos(client, vehicle_name=name_i)
        r_mig = pos_mig - pos_i
        v_mig = k_mig * r_mig / np.linalg.norm(r_mig)
        v_sep = np.zeros([2, 1])
        v_coh = np.zeros([2, 1])
        N_i = 0
        for j in range(uav_nums):
            if j != i:
                N_i += 1
                name_j = "UAV"+str(j+1)
                pos_j = get_UAV_pos(client, vehicle_name=name_j)
                if np.linalg.norm(pos_j - pos_i) < r_max:
                    r_ij = pos_j - pos_i
                    v_sep += -k_sep * r_ij / np.linalg.norm(r_ij)
                    v_coh += k_coh * r_ij
        v_sep = v_sep / N_i
        v_coh = v_coh / N_i
        v_cmd[:, i:i+1] = v_sep + v_coh + v_mig

    for i in range(uav_nums):   # 每个无人机的速度指令执行
        name_i = "UAV"+str(i+1)
        client.moveByVelocityZAsync(v_cmd[0, i], v_cmd[1, i], -3, 0.1, vehicle_name=name_i)
        #  client.moveByVelocityZAsync(vx, vy, z, duration,
        #  drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = '')
        # (x轴速度,Y轴速度，Z轴，时长，控制对象) ，水平速度控制（指定高度）


# 循环结束
client.simPause(False)  # Pauses simulation
for i in range(uav_nums):
    name = "UAV"+str(i+1)
    if i != uav_nums-1:                                              # 降落
        client.landAsync(vehicle_name=name)
    else:
        client.landAsync(vehicle_name=name).join()
for i in range(uav_nums):
    name = "UAV" + str(i+1)
    client.armDisarm(False, vehicle_name=name)              # 锁定，关闭螺旋桨
    client.enableApiControl(False, vehicle_name=name)       # 释放控制权
