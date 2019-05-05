

def make_path(checkpoints, res):
    '''
    Creates the path from the starting position to a position facing the waiter crossing
    '''
    path = [checkpoints[0]]
    for i in range(1, len(checkpoints)):
        DX = checkpoints[i][0] - checkpoints[i-1][0]
        DY = checkpoints[i][1] - checkpoints[i-1][1]
        steps = int(max(abs(DX), abs(DY)) / res)
        dx, dy = DX/steps, DY/steps
        pos = checkpoints[i-1]
        for j in range(steps):
            pos = (pos[0]+dx, pos[1]+dy)
            path.append(pos)
        path.append(checkpoints[i])
    return path


#Global Paths
PATH_A = make_path([ (2.0,0.4) , (2.0,1.1) , (1.0,2.0) , (0.75, 2.0), (0.75,1.8),(0.9, 1.0), (1.0, 0.35), (0.3, 0.35) ], 0.05)

PATH_AR = PATH_A[::-1]
