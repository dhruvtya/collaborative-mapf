#!/usr/bin/env python3
import argparse
from pathlib import Path
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import csv

Colors = ['green', 'blue', 'orange']


class Animation:
    def __init__(self, my_map, starts, goals, paths):
        self.my_map = np.flip(np.transpose(my_map), 1)
        self.starts = []
        for start in starts:
            self.starts.append((start[1], len(self.my_map[0]) - 1 - start[0]))
        self.goals = []
        for goal in goals:
            self.goals.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))

        aspect = len(self.my_map) / len(self.my_map[0])

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch

        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j] == -1:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))
                elif self.my_map[i][j] == 1:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray', alpha=0.2))

        # create agents:
        self.T = 0
        # draw goals first
        for i, goal in enumerate(self.goals):
            if(types[i] == 0):
                self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[0],
                                            edgecolor='black', alpha=0.5))
                # Add text of agent id at the goal
                self.agent_names[i] = self.ax.text(goal[0], goal[1], str(int(ids[i]/10)))
                self.agent_names[i].set_horizontalalignment('center')
                self.agent_names[i].set_verticalalignment('center')
    

        # Get number of helper agents
        num_helpers = 0
        for i in range(len(types)):
            if types[i] == 1:
                num_helpers += 1
        helper_count = 1

        for i in range(len(self.paths)):
            name = '-1'
            if(types[i] == 0):
                name = str(int(ids[i]/10))
                self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[0],
                                    edgecolor='black')
                self.agents[i].original_face_color = Colors[0]
            else:
                name = 'H' + str(helper_count)
                helper_count += 1
                self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor='yellow',
                                    edgecolor='black')
                self.agents[i].original_face_color = 'yellow'

            
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(paths[i]) - 1)
            self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])

        # Get movable obstacles
        movable_obstacles = []
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j] == 1:
                    mo = [i,j]
                    movable_obstacles.append(mo)

        # Draw movable obstacles only for the timesteps that no helper agents have been to the obstacle location
        longest_path = max([len(path) for path in paths])
                

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * 10,
                                                 interval=100,
                                                 blit=True)

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        for k in range(len(self.paths)):
            pos = self.get_state(t / 10, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos


def import_mapf_instance(filename):
    f = Path('maps/' + filename + '.txt')
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open('maps/' + filename + '.txt', 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(-1)
            elif cell == '.':
                my_map[-1].append(0)
            elif cell == '~':
                my_map[-1].append(1)
    f.close()
    return my_map


def import_results(filename):
    ids = []
    types = []
    starts = []
    goals = []
    paths = []

    with open('output/' + filename + '.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            ids.append(int(row[0]))
            types.append(int(row[1]))
            starts.append((int(row[2]), int(row[3])))
            goals.append((int(row[4]), int(row[5])))
            path = []
            for i in range(6, len(row), 2):
                path.append((int(row[i]), int(row[i+1])))
            paths.append(path)
    
    return ids, types, starts, goals, paths


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    
    args = parser.parse_args()
    file = args.instance
    my_map = import_mapf_instance(file)

    ids, types, starts, goals, paths = import_results(file)

    print("***Test paths on a simulation***")
    animation = Animation(my_map, starts, goals, paths)
    # animation.save("output.mp4", 1.0)
    animation.show()