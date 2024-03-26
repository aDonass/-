import airsim
import time
import numpy as np


client = airsim.MultirotorClient()  # connect to the AirSim simulator
name = "UAV1"
client.enableApiControl(True, name)                     # 开启Api控制UAVi
client.armDisarm(True, name)                            # 解锁螺旋桨
client.takeoffAsync(vehicle_name=name)                  # 起飞

client.moveToZAsync(-5, 1, vehicle_name=name).join()    # 飞到5m高度层 (注意z轴+为下-为上)
    # moveToZAsync(z, velocity, vehicle_name)
    # 是高度控制 API，第一个参数是高度，第二个参数是速度，第三个是控制的目标
client.moveByVelocityAsync(2, -2, -1, 10, vehicle_name=name).join()   # 三轴飞行
    # moveByVelocityAsync(vx, vy, vz, duration, vehicle_name='')
client.moveByVelocityAsync(2, 0, 0, 15, vehicle_name=name).join()   # vy vz = 0 时，相当于x轴方向前进
client.moveByVelocityAsync(0, 2, 0, 6, vehicle_name=name).join()   # y轴方向前进
client.moveByVelocityAsync(0, 0, 1, 13, vehicle_name=name).join()   # z轴方向前进

# 结束
client.simPause(False)  # Pauses simulation
client.landAsync(vehicle_name=name).join()              # 降落
client.armDisarm(False, vehicle_name=name)              # 锁定，关闭螺旋桨
client.enableApiControl(False, vehicle_name=name)       # 释放控制权
