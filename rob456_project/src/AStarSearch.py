import Queue
import numpy as np
import itertools

import math

# computes the distance in y plus the distance in x between two points
def manhattan_dist(pt1,pt2):
    return abs(pt1[0] - pt2[0]) + abs(pt1[1] - pt2[1])



def a_star_connection(world, starting_loc=(0,0), end_loc=(19,19)):
    print "Calculating A* path..."

    # holds all nodes we are already sure we know the shortest path to
    visited = set()

    # contains key value pairs where the key is the incoming node and the outgoing node
    path = {}

    # initialize the cost to get to all nodes to be inifinity
    cost = {node: np.inf for node in itertools.product(range(len(world)), range(len(world)))}

    # cost to get to the starting location is always zero
    cost[starting_loc] = 0
    
    # start with a priority queue only containing the starting node
    Q = Queue.PriorityQueue(maxsize=0)
    Q.put((0, starting_loc))

    # repeat until the queue is empty
    while not Q.empty():
        current_cost, current_loc = Q.get()

        # skip locations already in our visited list 
        # (e.g. locations we've already found the shortest path to)
        if current_loc in visited:
            continue

        # this node is the next shortest path we've found so far
        # so mark it as visited 
        visited.add(current_loc)

        # stop when we've reached the end location
        if current_loc == end_loc:
            break

        cur_row, cur_col = current_loc

        # consider all outgoing conections from the current location
        for next_loc in [(cur_row, cur_col+1), (cur_row, cur_col-1), \
                         (cur_row+1, cur_col), (cur_row-1, cur_col)]:
            # verify that we can reach the next location
            if world[next_loc] != 0 \
                or next_loc[0] >= world.shape[0] or next_loc[0] < 0 \
                or next_loc[1] >= world.shape[1] or next_loc[1] < 0:
                continue

            # add the distance to get to the next node
            cost_to_reach_next_loc = cost[current_loc] + 1

            # if this is the shortest path to this location that we have seen
            # mark this node as the shortest path that we have seen
            if cost_to_reach_next_loc < cost[next_loc]:
                cost[next_loc] = cost_to_reach_next_loc
                path[next_loc] = current_loc

            # priority is a combination of the true cost to reach the next node 
            # plus the estimated cost of reaching the goal node from the next node
            priority = cost_to_reach_next_loc + manhattan_dist(next_loc, end_loc) 

            # add next location to priority queue and place in
            # queue according to priority
            Q.put((priority, next_loc))
            
    return cost, path
            