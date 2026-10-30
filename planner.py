import numpy as np
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_, trajectory="parabola"):

        self.type=type_
        self.trajectory = trajectory
    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        if self.trajectory == "parabola":
            xs = np.linspace(0.0, 1, 60)
            ys = xs ** 2
        elif self.trajectory == "sigmoid":
            xs = np.linspace(0.0, 2.5, 50)
            ys = 2.0 / (1.0 + np.exp(-2.0 * xs)) - 1.0
        else:
            raise ValueError("trajectory must be 'parabola' or 'sigmoid'")

        # list of [x, y] lists (not a numpy array), as decisions.py expects
        return [[float(x), float(y)] for x, y in zip(xs, ys)]

