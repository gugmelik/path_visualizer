from grid import *
from numpy.random import randint

def dijkstra(draw, grid, start, end):
    print("started")
    inf = 100000
    d = np.empty((len(grid), len(grid)))
    d.fill(inf)
    q = PriorityQueue()
    q.put((0, start))
    d[start.get_pos()[0]][start.get_pos()[1]] = 0
    came_from = {}
    while not q.empty():
            cur_d, v = q.get()
            if cur_d > d[v.get_pos()[0]][v.get_pos()[1]]:
                continue

            for to in v.neighbors:
                if d[v.get_pos()[0]][v.get_pos()[1]] + 1 < d[to.get_pos()[0]][to.get_pos()[1]]:
                    d[to.get_pos()[0]][to.get_pos()[1]] = d[v.get_pos()[0]][v.get_pos()[1]] + 1
                    came_from[to] = v
                    q.put((-d[to.get_pos()[0]][to.get_pos()[1]], to))
            draw()
            if v != start:
                v.make_closed()

    if d[start.get_pos()[0]][start.get_pos()[1]] == inf:
        return False
    else: 
        reconstruct_path(came_from, end, draw)
        end.make_end()
        return True

def dfs(draw, grid, start, end):
    used = np.zeros((len(grid), len(grid))) # check if pos in grid is used or not
    came_from = {} # path
    stack = [start] 
    while stack: 
        v = stack.pop()
        if v == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        for to in v.neighbors:
            if not used[to.get_pos()[0]][to.get_pos()[1]]:
                used[v.get_pos()[0]][v.get_pos()[1]] = 1
                came_from[to] = v
                stack.append(to)
        draw()

        if v != start:
            v.make_closed()
    return False

def bfs(draw, grid, start, end):
    #print(start)
    used = np.zeros((len(grid), len(grid)))
    d = np.empty((len(grid), len(grid)))
    q = Queue()
    q.put(start)
    used[start.get_pos()[0]][start.get_pos()[1]] = 1
    d[start.get_pos()[0]][start.get_pos()[1]] = 0
    came_from = {}
    while not q.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        v = q.get()
        if v == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        for to in v.neighbors:
            if not used[to.get_pos()[0]][to.get_pos()[1]]: 
                used[to.get_pos()[0]][to.get_pos()[1]] = 1
                q.put(to)
                #print(to.get_pos()[0])
                d[to.get_pos()[0]][to.get_pos()[1]] = d[v.get_pos()[0]][v.get_pos()[1]] + 1
                came_from[to] = v
        draw()

        if v != start:
            v.make_closed()
    return False

def algorithm(draw, grid, start, end): # A* search
    #print("pressed")
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start]=0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start]=h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current]+1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count +=1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    #neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()

    return False

def h(p1, p2): # heuristic
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1 - y2)

def maze_gen(draw, grid, start, end): # maze generation loop
    done = False
    last_pos = (0, 0)
    pos_history = []
    pos_history.append(last_pos)

    back_step = 0
    while not done:
        draw()
        last_pos, back_step, done = generate_step(grid, last_pos, pos_history, back_step)
        if last_pos not in pos_history:
            pos_history.append(last_pos)
        

def generate_step(grid, last_pos, pos_history, back_step):
    (x, y) = last_pos
    grid[x][y].make_barrier()
    
    grid_dim = (len(grid), len(grid))
    possible_steps = possible_next_steps(grid_dim, last_pos)
    #print(f"Position: {last_pos}")
    #print(f"Possible steps: {possible_steps}")
    
    valid_steps = []
    for step in possible_steps:
        (x1, y1) = step[0]
        (x2, y2) = step[1]
        
        not_barrier = (not grid[x1][y1].is_barrier()) & (not grid[x2][y2].is_barrier())
        not_start = (not grid[x1][y1].is_start()) & (not grid[x2][y2].is_start())
        not_end = (not grid[x1][y1].is_end()) & (not grid[x2][y2].is_end())
        
        if bool(not_barrier * not_start * not_end):
            valid_steps.append(step)
    
    #print(f"Valid steps: {valid_steps}")
    
    if (len(valid_steps) == 0): # if it is a dead end
        last_pos = pos_history[-2 - back_step]
        if last_pos == (0,0):
            print("finished")
            done = True
            return last_pos, back_step, done
        back_step += 1
        done = False
        return last_pos, back_step, done
    
    else:
        back_step = 0 # reset it
        # choose a valid step at random
        if (len(valid_steps) == 1):
            last_pos = valid_steps[0]
            (x1, y1) = last_pos[0]
            (x2, y2) = last_pos[1]
            grid[x1][y1].make_barrier()
            grid[x2][y2].make_open()
            last_pos = last_pos[1]
            done = False
            return last_pos, back_step, done
        else:
            index = randint(0, len(valid_steps))
            # print(f"valid: {len(valid_steps)}, chose {index}")
            last_pos = valid_steps[index]
            (x1, y1) = last_pos[0]
            (x2, y2) = last_pos[1]
            grid[x1][y1].make_barrier()
            grid[x2][y2].make_open
            last_pos = last_pos[1]
            done = False
            return last_pos, back_step, done

def possible_next_steps(grid_dim, last_pos):
    """
    Parameters
    ----------
    grid_dim : tuple of 2 ints
        dimensions of the grid
    last_pos : tuple of 2 ints
        x, y coordinates of current position
    Returns
        possible_steps: list of list of tuples (x,y) denoting the
        next 2 movements possible in every direction possible
    """
    x_pos, y_pos = last_pos # unroll coordinates
    
    possible_steps = []
    operations_1 = [(0,1), (0,-1), (1,0), (-1,0)]
    operations_2 = [(0,2), (0,-2), (2,0), (-2,0)]
    num_operations = len(operations_1)
    
    for i in range(num_operations):
        op1_x, op1_y = operations_1[i]
        op2_x, op2_y = operations_2[i]
        
        if (is_in_map((x_pos + op1_x, y_pos + op1_y), grid_dim)) and (is_in_map((x_pos + op2_x, y_pos + op2_y), grid_dim)):
            possible_steps.append([(x_pos + op1_x, y_pos + op1_y), (x_pos + op2_x, y_pos + op2_y)])
    return possible_steps

def is_in_map(pos, grid_dim):
    """
    Parameters
    ----------
    pos : tuple of 2 ints 
        x, y coordinates in the grid system of current position
    grid_dim : tuple of ints
        x, y dimension of the grid system
    Returns
        true if pos in map
        false if not in map
    """
    (max_x, max_y) = grid_dim # unroll the dimensions
    (x, y) = pos # unroll the position coordinates
        
    x_in = (x < max_x) & (x >= 0) # logical x in map
    y_in = (y < max_y) & (y >= 0) # logical y in map
    return bool(x_in*y_in) # only true if both true