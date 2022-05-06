from math import sqrt, radians, cos, sin, degrees, pow
import numpy as np
import heapq as hq


def euclidean_dist(node1_x, node1_y, node2_x, node2_y):
    return sqrt((node1_x - node2_x)**2 + (node1_y - node2_y)**2)


class Node():
    def __init__(self, x, y, l, g, task, t, parent=-1) -> None:
        self.x = x
        self.y = y
        self.l = l
        self.g = g
        self.t = t # start node is at t=0
        self.task = task
        self.h = self.compute_h()
        self.f = self.g + self.h
        self.parent = parent
        # self.p = [self.x, self.y]
    
    def compute_h(self):
        if self.l == 1:
            assert len(self.task) == 2
            self.h = euclidean_dist(self.x, self.y, self.task[0][0], self.task[0][1]) + \
                     euclidean_dist(self.task[0][0], self.task[0][1], self.task[1][0], self.task[1][1])
        else:
            assert self.l == 2
            assert len(self.task) == 1
            self.h = euclidean_dist(self.x, self.y, self.task[0][0], self.task[0][1])

class Path():
    def __init__(self) -> None:
        self.xs = []
        self.ys = []
        self.ts = []
        self.len = 0

    def add_node(self, x, y):
        self.xs.insert(0, x)
        self.ys.insert(0, y)
        self.len += 1
    
    def add_time(self):
        self.ts = [i for i in range(self.len)]

def collision_exists(curr_x, curr_y, curr_t, prev_x, prev_y, prev_t, paths):
    for other_ag_path in paths:
        if curr_x == other_ag_path.xs[curr_t] and curr_y == other_ag_path.ys[curr_t]:
            return True
        elif curr_x == other_ag_path.xs[prev_t] and curr_y == other_ag_path.ys[prev_t] and \
             prev_x == other_ag_path.xs[curr_t] and prev_y == other_ag_path.ys[curr_t]:
             return True
    return False
                


def generate_children(parent, nav_space, paths):
    children = []
    for child_x in [parent.x-1, parent.x, parent.x+1]:
        for child_y in [parent.y-1, parent.y, parent.y+1]:
            if child_x == parent.x and child_y == parent.y:
                continue
            if child_x < 0 or child_x >= nav_space.shape[1]:
                continue
            if child_y < 0 or child_y >= nav_space.shape[0]:
                continue
            if nav_space[nav_space.shape[0]-1-child_y, child_x] == 0: # if there's a static obstacle
                continue
            child_t = parent.t + 1
            if collision_exists(child_x, child_y, child_t, parent.x, parent.y, parent.t, paths):
                continue

            additional_g = 1
            child_g = parent.g + additional_g
            child_l = parent.l
            child_task = parent.task
            child_t = parent.t + 1
            #check if already generated
            children.append(Node(child_x, child_y, child_l, child_g, child_task, child_t, parent))

    return children

def backtrack(final_node):
    path = Path()
    curr_node = final_node
    while not isinstance(curr_node, int):
        path.add_node(curr_node.x, curr_node.y)
        curr_node = curr_node.parent
    path.add_time()
    return path

def mla_star(start, task, paths, nav_space): # returns a path object
    Q = []
    hq.heappush(Q, (start.f, start))
    hq.heapify(Q)
    while len(Q) != 0:
        curr_node = hq.heappop(Q)
        if curr_node.l == 1:
            assert len(task) == 2
            pick_x = task[0][0]
            pick_y = task[0][1]
            t_max = 1e5
            for other_ag_path in paths:
                for i in range(other_ag_path.len):
                    if other_ag_path.xs[i] == pick_x and other_ag_path.ys[i] == pick_y:
                        t_max = other_ag_path.ts[i]
                        break
                if t_max != 1e5:
                    break
            if curr_node.g > t_max:
                continue
        if curr_node.l == 1:
            if curr_node.x == task[0][0] and curr_node.y == task[0][1]:
                new_node = Node(curr_node.x, curr_node.y, 2, curr_node.g, [task[1]], curr_node, curr_node.t)
                hq.heappush(Q, (new_node.f, new_node))
                continue
        elif curr_node.l == 2:
            assert len(task) == 1
            if curr_node.x == task[0][0] and curr_node.y == task[0][1]:
                path = backtrack(curr_node)
                return path
        
        children = generate_children(curr_node, nav_space, paths)
        for child in children:
            hq.heappush(Q, (child.f, child))
        
    return None

def get_start_locations(num_agents):
    start_locs = [[1, 10], [1, 15], [1, 20]]
    #for i in num_agents:
    return start_locs

def get_tasks(num_agents):
    tasks = [[[4,10],[0,10]], [[4,15],[0,15]], [[4,20],[0,20]]]
    return tasks

if __name__ == '__main__':
    num_agents = 3
    starts = get_start_locations(num_agents)
    tasks = get_tasks(num_agents)
    paths = []
    nav_space = np.ones((20, 40)) # static obstacle points
    # nav_space is such that if it is displayed with cv2.imshow, it'll show the map as one would draw it
    # nav_space[0] corresponds to top-most row (corresponding to y=nav_space.shape[0]-1)
    # nav_space[:, 0] corresponds to left-most column (corresponding to x=0)
    # x varies from 0 to nav_space.shape[1]-1
    # y varies from 0 to nav_space.shape[0]-1
    for i in range(num_agents):
        task = tasks[i]
        start = starts[i]
        start = Node(start[0], start[1], 1, 0, task, 0)
        path = mla_star(start, task, paths, nav_space)
        if path is None:
            print('No path found for agent-'+str(i))
        paths.append(path)