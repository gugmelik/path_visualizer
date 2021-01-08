from grid import *

def dijkstra(draw, grid, start, end):
    pass

def dfs(draw, grid, start, end):
    pass

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

def algorithm(draw, grid, start, end):
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
                    neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()

    return False

def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1 - y2)