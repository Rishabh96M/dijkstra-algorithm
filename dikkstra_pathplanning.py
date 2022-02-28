# dijkstra path planning
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Using dijkstra's algorith to find the optimum path from staring
# to goal poisiton

import heapq as heap
import matplotlib.pyplot as plt
from collections import defaultdict


def validPoint(point, map_len, map_bre):
    """
    Definition
    ---
    Method to check if point is valid on the map

    Parameters
    ---
    point : position to check for
    map_len : length of map
    map_bre : breadth of map

    Returns
    ---
    bool : True if point is valid, false if othervise
    """
    if (point[0] > map_len) or (point[1] > map_bre):
        return False
    elif (point[0] < 0) or (point[1] < 0):
        return False
    return True


def getAdjNodes(curr_node, map_len, map_bre):
    """
    Definition
    ---
    Method to generate all adjacent nodes for a given node

    Parameters
    ---
    curr_node : node of intrest
    map_len : length of map
    map_bre : breadth of map

    Returns
    ---
    adjNodes : list of adjacent nodes with cost from parent node
    """
    adjNodes = []
    if(validPoint((curr_node[0] + 1, curr_node[1]), map_len, map_bre)):
        adjNodes.append(((curr_node[0] + 1, curr_node[1]), 1))
    if(validPoint((curr_node[0], curr_node[1] + 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0], curr_node[1] + 1), 1))
    if(validPoint((curr_node[0] - 1, curr_node[1]), map_len, map_bre)):
        adjNodes.append(((curr_node[0] - 1, curr_node[1]), 1))
    if(validPoint((curr_node[0], curr_node[1] - 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0], curr_node[1] - 1), 1))
    if(validPoint((curr_node[0] + 1, curr_node[1] + 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0] + 1, curr_node[1] + 1), 1.4))
    if(validPoint((curr_node[0] - 1, curr_node[1] + 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0] - 1, curr_node[1] + 1), 1.4))
    if(validPoint((curr_node[0] + 1, curr_node[1] - 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0] + 1, curr_node[1] - 1), 1.4))
    if(validPoint((curr_node[0] - 1, curr_node[1] - 1), map_len, map_bre)):
        adjNodes.append(((curr_node[0] - 1, curr_node[1] - 1), 1.4))
    return adjNodes


def updateNode(new_node, curr_node, node_cost, queue, parent_map, cost):
    """
    Definition
    ---
    Method to update nodes based on cost and closed list of nodes

    Parameters
    ---
    new_node : node of intrest
    curr_node : parent node
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    cost : cost to get to new node from parent node

    Returns
    ---
    Reached : if new_node is goal node returns True othervise returns False
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    """
    new_cost = node_cost[curr_node] + cost
    temp_cost = node_cost.get(new_node)
    if not temp_cost or (temp_cost > new_cost):
        node_cost[new_node] = new_cost
        parent_map[new_node] = curr_node
        heap.heappush(queue, (new_cost, new_node))
    if new_node == goal:
        return True, node_cost, queue, parent_map
    return False, node_cost, queue, parent_map


def dijkstra_path(start, goal, map_len, map_bre):
    """
    Definition
    ---
    Method to get least cost path from starting to goal node using dijkstra's

    Parameters
    ---
    start : starting node
    goal : goal node
    map_len : length of map
    map_bre : breadth of map

    Returns
    ---
    Reached : if path is found True othervise False
    parent_map : dict of nodes mapped to parent node_cost
    node_cost : dict of all nodes mapped to costs
    """
    closed = []
    queue = []
    node_cost = defaultdict(lambda: float('inf'))
    node_cost = {}
    parent_map = {}
    reached = False

    node_cost[start] = 0
    heap.heappush(queue, (0, start))

    while not reached and queue:
        curr_cost, curr_node = heap.heappop(queue)
        closed.append(curr_node)
        adjNodes = getAdjNodes(curr_node, map_len, map_bre)
        for new_node, cost in adjNodes:
            if new_node in closed:
                continue
            # print('checking for node: ', new_node)
            flag, node_cost, queue, parent_map = updateNode(
                new_node, curr_node, node_cost, queue, parent_map, cost)
            if flag:
                reached = True
                break
    return reached, parent_map, node_cost


def getPath(parent_map, start, goal):
    """
    Definition
    ---
    Method to generate path using backtracking

    Parameters
    ---
    parent_map : dict of nodes mapped to parent node_cost
    start : starting node
    goal : goal node

    Returns
    ---
    path: list of all the points from starting to goal position
    """
    curr_node = goal
    parent_node = parent_map[goal]
    path = [curr_node]
    while not parent_node == start:
        curr_node = parent_node
        parent_node = parent_map[curr_node]
        path.append(curr_node)
    path.append(start)
    return path[::-1]


if __name__ == '__main__':
    map_len = 400
    map_bre = 250
    flag = False

    start = input("Input Staring Position in format: x,y\n")
    start = (float(start.split(',')[0]), float(start.split(',')[1]))
    if validPoint(start, map_len, map_bre):
        goal = input("Input Goal Position in format: x,y\n")
        goal = (float(goal.split(',')[0]), float(goal.split(',')[1]))
        if validPoint(goal, map_len, map_bre):
            print('performing')
            flag, parent_map, node_cost = dijkstra_path(
                start, goal, map_len, map_bre)
            if flag:
                print('Path Found')
                path = getPath(parent_map, goal, start)
                path_x = []
                path_y = []
                for point in path:
                    path_x.append(point[0])
                    path_y.append(point[1])
                plt.plot(goal[0], goal[1], 'ro')
                plt.plot(start[0], start[1], 'go')
                plt.plot(path_x, path_y, 'k-')
                plt.xlim([0, map_len])
                plt.ylim([0, map_bre])
                plt.show()
            else:
                print('Path not found')
                print(parent_map)
        else:
            print('Not a valid point')
    else:
        print('Not a valid point')
