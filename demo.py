# -*- coding: utf-8 -*-
# @Author  : zx jiang
# @Time    : 2025-02-03 12:34
# @File    : demo.py
# @Software: Cursor

import traci  
import os, sys
from loguru import logger
import time
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False # 坐标轴正常显示负号
import numpy as np

# configuration of SUMO
if 'SUMO_HOME' in os.environ:
  sumo_home = os.environ['SUMO_HOME']
  tools = os.path.join(sumo_home, 'tools')
  sys.path.append(tools)
else:
  sys.exit("Please declare environment variable 'SUMO_HOME'")
    
sumoBinary = os.path.join(sumo_home, "bin", "sumo-gui")
path = os.path.dirname(os.path.realpath(__file__))

# TODO: 打开SUMO接口
try:    
    sumoBinary = os.path.join(sumo_home, "bin", "sumo")
    sumocfg_path = path + '\\demo.sumocfg'  # sumocfg文件的位置
    sumocmd = [ sumoBinary, "-c", sumocfg_path, "--delay", "200" , "-S"]
    traci.start(sumocmd)    
except:
    logger.error('换一下路径')

# 获取所有车道ID
lane_ids = traci.lane.getIDList()
# print('车道ID:',lane_ids)

# 2、手写信号相位与具有通行权的车道（在sumo-netedit里对应找）
# TODO: 修改
phase_lane = {
    0: ['E3_0', 'E3_1', 'E3_2', 'E2_0', 'E1_0', 'E1_1', 'E1_2', 'E0_0'],
    1: ['E3_2', 'E1_2'],
    2: ['E3_2', 'E1_2'],
    3: [],
    4: ['E3_0', 'E2_0', 'E2_1', 'E2_2', 'E1_0', 'E0_0', 'E0_1', 'E0_2'],
    5: ['E2_2', 'E0_2'],
    6: ['E2_2', 'E0_2'],
    7: [],
}

# 获取具有通行权的车道
inter_lane = sorted(list(set([element for sublist in phase_lane.values() for element in sublist])))  # 使用set去重
inter_lane_change = inter_lane.copy()
for i in range(len(inter_lane)):
    inter_lane_change[i] = inter_lane[i].replace('_1', '_直行').replace('_2', '_左转').replace('_0', '_右转')
logger.info(inter_lane_change)

# 4、手写信号相位与对应的信号状态（可在sumo-netedit里复制）
# TODO: 修改
phase_timing = {
  0: 'GGgGrrGGgGrr',
  1: 'yygyrryygyrr',
  2: 'rrGrrrrrGrrr',
  3: 'rryrrrrryrrr',
  4: 'GrrGGgGrrGGg',
  5: 'yrryygyrryyg',
  6: 'rrrrrGrrrrrG',
  7: 'rrrrryrrrrry',
}

# 5、获取唯一的交通灯的ID
traffic_light_id = traci.trafficlight.getIDList()[0]
logger.warning("交通灯ID:{}".format(traffic_light_id))

# 6、展示一下仿真步长
stepLength = traci.simulation.getDeltaT() # 仿真步长为1.0
logger.warning("仿真步长:{}".format(stepLength))


# TODO 三、基于排队车辆数的实时信号控制算法
def phase_change( curr_phase, phase_duration, phase_duration_max, phase_lane_queue, queue_max):
    """
    Parameters：当前相位，相位持续时间，相位持续时间上限，各个相位最大排队长度，排队长度上限
    Returns：下个相位的序号
    """
    
    # 1、约束
    con1 = phase_duration <= phase_duration_min # 约束1：相位持续时间是否低于最小值
    con2 = phase_duration > phase_duration_min and phase_duration <= phase_duration_max # 约束2：相位持续时间是否正常
    con3 = phase_duration > phase_duration_max # 约束3：相位持续时间是否超出上限
    
    # 2、约束判断是否跳转相位
    if con1:
        next_phase = curr_phase
    
    if con2:
        next_phase = curr_phase
        if max(phase_lane_queue.values()) > queue_max:
            # 这里如果最大排队仍然是当前相位，可以继续给绿灯，只要不超过最大限制就行
            next_phase = max( phase_lane_queue, key = phase_lane_queue.get )
    
    if con3:
        sorted_keys = sorted( phase_lane_queue, key = phase_lane_queue.get, reverse=True)
        sorted_keys.remove(curr_phase)
        next_phase = sorted_keys[0] # 下一个相位一定不是当前相位
    
    return next_phase



#%% 四、仿真
# 1、参数设置
stop_time = 500 # 最大仿真时间
init_phase = 0 # 初始相位
phase_lane_queue = { key:0 for key,_ in phase_lane.items() } # 逐个相位下所有具有通行权的车道最大排队车辆数
queue_max = 10 # 排队车辆数上限

phase_duration = 0 # 相位持续时间
phase_duration_min = 10 # 最小相位持续时间
phase_duration_max = 60 # 最大相位持续时间
phase_shift = False # 相位切换时间

yellow_duration = 0 # 黄灯持续时间
yellow_duration_max = 3 # 最大黄灯持续时间
yellow_light = False # 黄灯期间

next_phase = 0 # 下一个相位
traci.trafficlight.setRedYellowGreenState( traffic_light_id, phase_timing[next_phase])  # 设置初始信号灯

ans = []  # 排队长度收集

logger.success("START!")

# 2、开始仿真
while traci.simulation.getTime() < stop_time:
    # traci.simulation.getTime() 表示
    # 仿真
    traci.simulationStep()
    phase_duration += 1   # 更新相位持续时间
    t = traci.simulation.getTime()  # 仿真时间更新
    if t % 30 == 0:
        logger.info( '当前仿真步长:{}'.format(t) )
    
    # 统计当前时间步的逐个相位下所有具有通行权的车道最大排队车辆数
    ans.append( [t, next_phase, phase_duration] + [ traci.lane.getLastStepHaltingNumber( laneID ) for laneID in inter_lane ] ) # 所有车道的（不含右转）
    for key, _ in phase_lane_queue.items():
        if len(phase_lane[key]) == 0:  # 对于空列表（黄灯相位）
            phase_lane_queue[key] = 0  # 直接设置为0
        else:
            phase_lane_queue[key] = max( [ traci.lane.getLastStepHaltingNumber( laneID ) for laneID in  phase_lane[key] ] )
    
    # 计算最佳相位
    curr_phase = next_phase # 当前相位
    next_phase = phase_change( curr_phase, phase_duration, phase_duration_max, phase_lane_queue, queue_max)  # 调用函数
    # logger.warning("当前相位:{}, 下一个相位:{}".format(curr_phase, next_phase))

    # 更新相位
    if next_phase != curr_phase:
        # 先执行黄灯，或者说红灯间隔相位
        for _ in range( yellow_duration_max ):
            traci.trafficlight.setRedYellowGreenState( traffic_light_id,  'rrrrrrrrrrrr')  # 设置信号灯
            traci.simulationStep() # 直接向下更新三个
        # 再换成下一个相位
        phase_duration = 0 # 相位持续时间清零
        traci.trafficlight.setRedYellowGreenState( traffic_light_id, phase_timing[next_phase] )  # 设置信号灯
    

#%% 五、数据可视化
def plot_ans(ans):
    new_ans = ans

    # 设置柱状图的宽度（调小一些以确保所有柱子都能显示）
    bar_width = 0.6
    
    # 创建一个新的图表
    plt.figure(figsize=(12, 6))  # 加大图表尺寸
    plt.pause(5)
    
    for i, row in enumerate(new_ans):
        tim, data = row[0], row[3:]
        # 计算每一组柱状图的x坐标
        x = np.arange(len(data))
        # 用bar函数绘制柱状图
        bars = plt.bar(x, data, width=bar_width, align='center', alpha=0.8, label=tim)
    
        # 添加数值标签
        for j, rect in enumerate(bars):
            plt.text(rect.get_x() + rect.get_width()/2, rect.get_height(), 
                    str(data[j]), ha='center', va='bottom')
                
        # 设置x轴的刻度位置和标签
        plt.xticks(x, inter_lane_change, rotation=45)  # 添加rotation让标签斜着显示，避免重叠
        
        # 调整x轴显示范围，确保所有柱子都能显示
        plt.xlim(-0.5, len(data)-0.5)
        plt.ylim(0, 35)
        
        # 调整布局，确保标签不被切掉
        plt.tight_layout()
        
        plt.title('仿真时间{}秒，'.format(int(tim)) + '相位{}, '.format(row[1]) + '相位持续时间{}秒'.format(row[2]))
        plt.ylabel('排队车辆数')
        
        plt.draw()
        plt.pause(0.5)
        if i < len(new_ans)-1:
            plt.clf()

    plt.ioff()
    plt.show()
    logger.success('DONE!')

plot_ans(ans)
traci.close() # 关闭接口
