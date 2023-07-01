#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt

def update_x(t, R, t_1, t_2, v_0):
    vel = 1/t_1 + 1/t_2
    neg_vel = 1/t_2 - 1/t_1
    return R * (t_1 + t_2) * np.sin((t**2 * neg_vel * v_0)/(4*R)) / (t_1 - t_2)

def update_y(t, R, t_1, t_2, v_0):
    vel = 1/t_1 + 1/t_2
    neg_vel = 1/t_2 - 1/t_1
    return 2 * R * (t_1 + t_2) * np.sin((t**2 * neg_vel * v_0)/(8*R))**2 / (t_1 - t_2)

def generate_path(t_List, R, t_1, t_2, v_0):
    x_list = [update_x(t, R, t_1, t_2, v_0) for t in t_List]
    y_list = [update_y(t, R, t_1, t_2, v_0) for t in t_List]
    return x_list, y_list

def plot_path(x_list, y_list):
    plt.rcParams['figure.figsize'] = (7,7)
    plt.plot(x_list, y_list)
    plt.scatter(x_list[0], y_list[0], color='green', label='Start')
    plt.scatter(x_list[-1], y_list[-1], color='red', label='End')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.savefig('assignment2_a.png', dpi=300)
    plt.show()
    
if __name__ == '__main__':
    t_List = np.linspace(0, 10, 1000)
    R = 0.12
    v_0 = 0.5
    t_1 = 10
    t_2 = 5

    x_list, y_list = generate_path(t_List, R, t_1, t_2, v_0)
    plot_path(x_list, y_list)

    print(x_list[0], y_list[0])
    print(x_list[-1], y_list[-1])


# In[ ]:




