#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.animation as anim
show_animation = True
writer = anim.FFMpegWriter(fps=20, metadata=dict(artist='Me'), bitrate=1800, extra_args=['-loglevel', 'error'])
plt.rcParams['animation.ffmpeg_path'] = '/usr/local/bin/ffmpeg'

open_list = []
closed_list = []
p_x, p_y = [], []


# The planner
def path_planner(start_x, start_y, goal_x, goal_y, resolution):
    c_x = start_x
    c_y = start_y
    global p_x, p_y
    path_x, path_y = [start_x], [start_y]
    p_x_prev = [0 for i in range(8)]
    p_y_prev = [0 for i in range(8)]
    while True:
        p1_x = p2_x = p3_x = round((c_x - resolution), 2)
        p4_x = p8_x = round(c_x, 2)
        p5_x = p6_x = p7_x = round((c_x + resolution), 2)
        p1_y = p8_y = p7_y = round((c_y - resolution), 2)
        p2_y = p6_y = round(c_y, 2)
        p3_y = p4_y = p5_y = round((c_y + resolution), 2)

        p_x = [p1_x, p2_x, p3_x, p4_x, p5_x, p6_x, p7_x, p8_x]
        p_y = [p1_y, p2_y, p3_y, p4_y, p5_y, p6_y, p7_y, p8_y]

        print("points_x : {}, points_y : {}".format(p_x, p_y))

        p_xy = [(p_x[i], p_y[i]) for i in range(len(p_x))]
        p_xy_prev = [(p_x_prev[i], p_y_prev[i]) for i in range(len(p_x_prev))]
        print("points_xy : {}".format(p_xy))
        print("points_xy_prev : {}".format(p_xy_prev))

        matched = [i for i in p_xy for j in p_xy_prev if i == j]
        matched = list(set(matched))
        print("matched : {}".format(matched))

        if matched:
            [p_xy.remove(matched[i]) for i in range(len(matched))]
        unvisited = p_xy

        print("unvisited : {}".format(unvisited))

        p_x = [unvisited[i][0] for i in range(len(unvisited))]
        p_y = [unvisited[i][1] for i in range(len(unvisited))]

        if show_animation:
            plt.plot(p_x, p_y, ".g", ms=0.75)
            plt.gcf().canvas.mpl_connect('exit_event', lambda event: [exit(0) if event.key == 'escape' else None])
            plt.pause(0.02)
            writer.grab_frame()

        hue = heuristics(p_x, p_y, goal_x, goal_y)
        print(hue)

        cost = actual_cost(p_x, p_y, start_x, start_y)
        print(cost)

        total_cost = np.add(hue, cost)
        print(total_cost)
        ind = np.argmin(total_cost)

        p_x_prev = np.append(p_x_prev, [*p_x, c_x])
        p_y_prev = np.append(p_y_prev, [*p_y, c_y])
        print("p_x_prev : {}".format(p_x_prev))
        print("p_y_prev : {}".format(p_y_prev))

        c_x = p_x[int(ind)]
        c_y = p_y[int(ind)]
        path_x.append(c_x)
        path_y.append(c_y)

        print("current point : {},{}\n".format(c_x, c_y))
        if c_x == goal_x and c_y == goal_y:
            break

    return path_x, path_y


# remaining traversal cost
def heuristics(pt_x, pt_y, gpt_x, gpt_y):
    h = [math.sqrt((pt_x[i] - gpt_x)**2 + (pt_y[i] - gpt_y)**2) for i in range(len(pt_x))]
    return h


# traversal incurred cost
def actual_cost(pt_x, pt_y, st_x, st_y):
    ac = [math.sqrt((pt_x[i] - st_x)**2 + (pt_y[i] - st_y)**2) for i in range(len(pt_x))]
    return ac


# the main function definition
def main():
    # position
    s_x = 1
    s_y = 0
    g_x = 26
    g_y = 15
    res = 0.5

    # create obstacles
    obs_x = []
    obs_y = []

    for i in range(-5, 31):
        obs_x.append(i)
        obs_y.append(-5)
    for i in range(-4, 31):
        obs_x.append(-5)
        obs_y.append(i)

    fig1 = plt.figure()

    if show_animation:
        with writer.saving(fig1, 'ASPP.mp4', dpi=200):
            plt.plot(obs_x, obs_y, ".k", ms=3)
            plt.plot(s_x, s_y, "db", ms=5)
            plt.plot(g_x, g_y, "*r", ms=5)
            plt.axis("equal")
            plt.xlabel("x-axis")
            plt.title("A-Star Path Planning Algorithm")
            writer.grab_frame()
            path_x, path_y = path_planner(s_x, s_y, g_x, g_y, res)
            plt.plot(path_x, path_y, "orange", ms=3)
            writer.grab_frame()
            plt.draw()
            plt.show()


if __name__ == '__main__':
    main()

