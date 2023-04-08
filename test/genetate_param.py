import numpy as np
import pandas as pd


def random_list(start, stop, length):
    start, stop = (start, stop) if start <= stop else (stop, start)
    length = int(abs(length)) if length else 0
    random_list = []
    for i in range(length):
        random_list.append(round(np.random.rand()*(stop - start) + start,4))  # 确定保留多少位小数
    return random_list

'''
    生成误差参数
'''
# degree
delta_theta = random_list(-1/180*np.pi,1/180*np.pi,6)
# mm
delta_d = random_list(0,2,6)
delta_a = random_list(0,2,6)
delta_alpha = random_list(-1/180*np.pi,1/180*np.pi,6)
delta_beta = [0,0,0,0,0,0]

data = {'delta_theta':[i for i in delta_theta],
        'delta_d':[i for i in delta_d],
        'delta_a':[i for i in delta_a],
        'delta_alpha':[i for i in delta_alpha],
        'delta_beta':[i for i in delta_beta]}

df_delta = pd.DataFrame(data)

df_delta.to_excel('data\excel\dh_delta.xlsx',index=False)  # 不生成第一列的索引行

'''
生成随机点
'''
theta = [[0]]*6
theta[0] = random_list(-165,165,50)
theta[1] = random_list(-90,150,50)
theta[2] = random_list(-90,175,50)
theta[3] = random_list(-180,180,50)
theta[4] = random_list(-135,135,50)
theta[5] = random_list(-360,360,50)
df_theta_table = pd.DataFrame(theta).T
df_theta_table.to_excel('data/excel/theta_table.xlsx',index=False,header=None)

