#the code makes use of A Star algorithm alongside low level , high level CBS to find optimal path routing for multiple agents.
#the grid given below can be filled with 0 and 1 , where 0 is a valid block and 1 is a barrier.

import heapq

WIDTH = 10

agents = []
grid = [[0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0]]

def agent_append(agents, start, goal):
    agents.append((start, goal))

n_agents = int(input("How many agents do you want to have: "))
for i in range(n_agents):
    print(f"Enter x1, y1, and x2, y2 to specify start and end points of agent {i+1}:")
    x1 = int(input("Enter x1: "))
    y1 = int(input("Enter y1: "))
    x2 = int(input("Enter x2: "))
    y2 = int(input("Enter y2: "))
    agent_append(agents, (x1, y1), (x2, y2))

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) 

def add_node(x, y):
    return {'x': x, 'y': y, 'cost': float('inf'), 'constraints': []}

def findAstar(grid, start, goal):
    open_set = [(0, start)] 
    previous_from = {} 
    current_cost = {start: 0}
    finalpath = []  # Initialize the final path list here

    while open_set:
        currentcost, currentnode = heapq.heappop(open_set)
        if currentnode == goal:
            while currentnode is not None:
                finalpath.append(currentnode)
                currentnode = previous_from.get(currentnode)
            finalpath.reverse() 
            return finalpath

        for nextnode in neighbours(grid, currentnode):
            newcost = current_cost[currentnode] + 1
            if nextnode not in current_cost or newcost < current_cost[nextnode]:
                current_cost[nextnode] = newcost 
                heapq.heappush(open_set, (newcost + heuristic(nextnode, goal), nextnode))  
                previous_from[nextnode] = currentnode

def neighbours(grid, node):
    neighbors = []
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        x, y = node[0] + dx, node[1] + dy
        if (0 <= x < WIDTH) and (0 <= y < WIDTH) and (grid[x][y] != 1):
            neighbors.append((x, y))  
    return neighbors


def low_level(agents, agentno):
    start, goal = agents[agentno]
    path = findAstar(grid, start, goal)
    return path

def high_level_search(agents):
    open_nodes = [(0, [])]  
    while open_nodes:
        current_cost, current_solution = min(open_nodes, key=lambda x: x[0])   
        open_nodes.remove((current_cost, current_solution)) 
        if check(current_solution): 
           
            return current_solution
        else:
            for agent_index in range(len(agents)):  
                new_solution = current_solution.copy()  
                new_solution.append(low_level(agents, agent_index))  
                newcost = calculate_cost(new_solution)  
                open_nodes.append((newcost, new_solution)) 

def calculate_cost(solution):
    return len(solution)

def check(solution):
    if not solution:
        return False
    for time_step in range(max(len(path) for path in solution)): 
        positions = [path[time_step] if len(path) > time_step else path[-1] for path in solution] 
        if len(positions) != len(set(positions)):   
            return False
    return True


solution = high_level_search(agents) 
print("Solution:", solution) 
