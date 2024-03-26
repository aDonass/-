import airsim
import time
import numpy as np
import datetime


client = airsim.MultirotorClient()  # connect to the AirSim simulator
name = "UAV1"
client.enableApiControl(True, name)                     # 开启Api控制UAVi
client.armDisarm(True, name)                            # 解锁螺旋桨
client.takeoffAsync(vehicle_name=name)                  # 起飞

client.moveToZAsync(-5, 1, vehicle_name=name).join()    # 飞到5m高度层 (注意z轴+为下-为上)
    # moveToZAsync(z, velocity, vehicle_name)
    # 是高度控制 API，第一个参数是高度，第二个参数是速度，第三个是控制的目标

client.moveByVelocityZAsync(1, 1, -5, 10, vehicle_name=name).join()     # 水平速度控制（指定高度）
    # client.moveByVelocityZAsync(vx, vy, z,vehicle_name = '')
    # (x轴速度,Y轴速度，Z轴，时长，控制对象)

client.moveByVelocityAsync(2, -2, -1, 10, vehicle_name=name).join()   # 三轴飞行
    # moveByVelocityAsync(vx, vy, vz, duration, vehicle_name='')
client.moveByVelocityAsync(2, 0, 0, 15, vehicle_name=name).join()   # vy vz = 0 时，相当于x轴方向前进
client.moveByVelocityAsync(0, 2, 0, 6, vehicle_name=name).join()   # y轴方向前进
client.moveByVelocityAsync(0, 0, 1, 5, vehicle_name=name).join()   # z轴方向前进

client.hoverAsync(vehicle_name=name).join()             # 悬停

responses = client.simGetImages([
    airsim.ImageRequest(0, airsim.ImageType.Scene, pixels_as_float=False, compress=True),           # 0
    airsim.ImageRequest(0, airsim.ImageType.Segmentation, pixels_as_float=False, compress=True),     # 5
    airsim.ImageRequest(0, airsim.ImageType.Infrared, pixels_as_float=False, compress=True)     # 5
], vehicle_name = name)

now = datetime.datetime.now()  # 时间
now_string = now.strftime('%Y_%m_%d_%H_%M_')

for i in range(3):
    image_name = now_string + "scene" + str(i) + ".png"
    f = open(image_name, 'wb')
    f.write(responses[i].image_data_uint8)
    f.close()

# 结束
client.simPause(False)  # Pauses simulation
client.landAsync(vehicle_name=name).join()              # 降落
client.armDisarm(False, vehicle_name=name)              # 锁定，关闭螺旋桨
client.enableApiControl(False, vehicle_name=name)       # 释放控制权



