import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取 CSV 文件
data = pd.read_csv('/home/ethan/swarm_robot/results/simu_results0.csv')

# 获取迭代次数
num_iters = 10

# 绘制每个数据的图像
plt.figure(figsize=(10, 8))

# 绘制 Cost Value Over Iterations
plt.subplot(3, 2, 1)
cost_values = [data[f'Cost Iter {i}'] for i in range(num_iters)]
plt.plot(range(num_iters), cost_values, label='Cost Value', marker='o')
plt.xlabel('Iteration')
plt.ylabel('Cost Value')
plt.title('Cost Value Over Iterations')
plt.legend()

# 绘制 Computation Time Over Iterations
plt.subplot(3, 2, 2)
computation_times = [data[f'Computation Time Iter {i}'] for i in range(num_iters)]
plt.plot(range(num_iters), computation_times, label='Computation Time', marker='o')
plt.xlabel('Iteration')
plt.ylabel('Computation Time (ms)')
plt.title('Computation Time Over Iterations')
plt.legend()

# 绘制 Trajectory Length Over Iterations
plt.subplot(3, 2, 3)
velocityx = [data[f'Velocity vx {i}'] for i in range(num_iters)]
plt.plot(range(num_iters), velocityx, label='vx', marker='o')
plt.xlabel('Iteration')
plt.ylabel('Velocity x (m/s)')
plt.title('Velocity vx Over Iterations')
plt.legend()

# plt.subplot(4, 2, 4)
# for i in range(data.shape[1] // 2):  # 假设每个速度值占两列
#     velocity = [data[f'Velocity {i} Iter {j}'].mean() for j in range(num_iters)]
#     plt.plot(range(num_iters), velocity, label=f'Velocity {i}', marker='o')
# plt.xlabel('Iteration')
# plt.ylabel('Velocity (m/s)')
# plt.title('Velocity Over Iterations')
# plt.legend()

# 绘制 Acceleration Over Iterations
# plt.subplot(4, 2, 5)
# for i in range(data.shape[1] // 2 - 1):  # 假设加速度值在速度值后面
#     acceleration = [data[f'Acceleration {i} Iter {j}'].mean() for j in range(num_iters)]
#     plt.plot(range(num_iters), acceleration, label=f'Acceleration {i}', marker='o')
# plt.xlabel('Iteration')
# plt.ylabel('Acceleration (m/s^2)')
# plt.title('Acceleration Over Iterations')
# plt.legend()

# 绘制 Constraint Violations Over Iterations
# plt.subplot(3, 2, 4)
# for c in range(1,3):
#     constraint_violations = [data[f'Constraint Violation Iter {i} Constraint {c}'].mean() for i in range(1,num_iters)]
#     plt.plot(range(1,num_iters), constraint_violations, label=f'Constraint {c}', marker='o')
# plt.xlabel('Iteration')
# plt.ylabel('Constraint Violation')
# plt.title('Constraint Violations Over Iterations')
# plt.legend()

# plt.tight_layout()


# table_data = []
# for i in range(min(num_iters, len(data))):
#     row = [data.iloc[i]['Cost Value'], data.iloc[i]['Computation Time'], data.iloc[i]['Trajectory Length']]
#     table_data.append(row)

# # 在新的子图上创建表格
# fig, ax = plt.subplots(figsize=(10, 10))
# ax.axis('tight')
# ax.axis('off')
# table = ax.table(cellText=table_data, colLabels=['Cost Value', 'Computation Time', 'Trajectory Length'], loc='center', cellLoc='center')
plt.show()
