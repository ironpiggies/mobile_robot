

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
PATH_A = make_path([ (2.0,0.4) , (2.0,1.1) , (1.3,1.95) , (0.9, 1.95), (0.7,1.8),(0.8, 1.0), (1.0, 0.9),(1.0, 0.4), (0.2, 0.4) ], 0.05)

PATH_AR = PATH_A[::-1]

PATH_B = make_path([(0.3, 0.4),(1.15, 0.4)], 0.05)

PATH_C = make_path([(1.1, 0.4),(0.95, 1.0),(0.8,1.2),(0.7,1.8), (0.65, 1.95), (2.0, 1.95)], 0.05)

PATH_0 = make_path([(2.0,0.4), 
                    (2.0,1.0),
                    (1.4,1.7),
                    (1.3,2.0),
		    (1.2,2.0),
		    (0.7,1.8)], 0.05)

PATH_1 = [0.7, 1.8]

PATH_2A = make_path([(0.7,1.8),
		     (0.8,1.0),
		     (1.1,0.7),
		     (1.1,0.4),
		     (0.2,0.4)], 0.05)

PATH_2B = make_path([(0.7,1.8),
		     (0.5,1.5),
		     (0.5,1.2),
		     (0.8,1.0),
		     (1.1, 0.7),
		     (0.2,0.4)], 0.05)


PATH_3 = make_path([(0.2,0.4),
		    (1.25,0.35)], 0.05)

PATH_4 = make_path([(1.25,0.35),
		    (1.1,0.4),
		    (1.2,0.8)], 0.05)

PATH_5A = make_path([(1.2,0.8),
		     (0.8,1.1),
		     (0.8,2.05),
		     (1.8,2.05)], 0.05)

PATH_5B = make_path([(1.2,0.8),
		     (0.3,0.8),
		     (0.3,2.05),
		     (1.8,2.05)], 0.05)

PATH_6 = make_path([(1.8,2.05),
		    (0.5,2.0)], 0.05)



PATH_3 = make_path([(1.8,2.05),
		    (0.6,2.0)], 0.05)

PATH_4 = make_path([(0.6,2.0),
		    (0.8,2.0),
		    (0.8,1.7),
		    (0.8,1.0),
		    (1.1,0.7),
		    (1.1,0.4),
		    (0.2,0.4)], 0.05)

PATH_STOP = []
