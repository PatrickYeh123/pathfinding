from __future__ import print_function
#from heapq import * #Hint: Use heappop and heappush
import heapq

ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]

class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = [(0, self.grid.start)]
            self.explored = []
        elif self.type == "astar":
            # tuple of (F = G + H, G, node)
            self.frontier = [(self.get_h(self.grid.start, self.grid.goal), 0, self.grid.start)]
            self.explored = []

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True #This turns the color of the node to red
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    #Implement DFS
    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop()

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.explored.append(current)
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored and n not in self.frontier:
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

    #Implement BFS
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        current = self.frontier.pop(0)

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.explored.append(current)
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored and n not in self.frontier:
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

    #Implement UCS
    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        frontier_entry = heapq.heappop(self.frontier)
        current = frontier_entry[1]
        distance = frontier_entry[0]

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.explored.append(current)
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    distance_new = distance + self.grid.nodes[n].cost()
                    if n not in self.explored and not [x for x in self.frontier if n in x]:
                        self.previous[n] = current
                        heapq.heappush(self.frontier, (distance_new, n))
                        self.grid.nodes[n].color_frontier = True
                    elif [x for x in self.frontier if n in x]:
                        old_frontier_entry = [x for x in self.frontier if n in x][0]
                        # replace old entry if it is worse cost
                        if old_frontier_entry[0] > distance + self.grid.nodes[n].cost():
                            self.frontier.remove(old_frontier_entry)
                            self.previous[n] = current
                            heapq.heappush(self.frontier, (distance_new, n))
                            self.grid.nodes[n].color_frontier = True



    
    #Implement A*
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        frontier_entry = heapq.heappop(self.frontier)
        current = frontier_entry[2]
        g_current = frontier_entry[1]

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.explored.append(current)
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    g_new = g_current + self.grid.nodes[n].cost()
                    f_new = g_new + self.get_h(n, self.grid.goal)
                    if n not in self.explored and not [x for x in self.frontier if n in x]:
                        self.previous[n] = current
                        heapq.heappush(self.frontier, (f_new, g_new, n))
                        self.grid.nodes[n].color_frontier = True
                    elif [x for x in self.frontier if n in x]:
                        old_frontier_entry = [x for x in self.frontier if n in x][0]
                        # replace old entry if it is worse cost
                        if old_frontier_entry[0] > f_new:
                            self.frontier.remove(old_frontier_entry)
                            self.previous[n] = current
                            heapq.heappush(self.frontier, (f_new, g_new, n))
                            self.grid.nodes[n].color_frontier = True

    def get_h(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])
