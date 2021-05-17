#!/usr/bin/env python
from path_planning.Bezier import Bezier
from path_planning.BezierSpline import BezierSpline
from path_planning.CubicSpline import CubicSpline
from path_planning.Dubins import Dubins
from path_planning.Line import Line
from path_planning.Ellipse import Ellipse

DUBINS_POINTS = [[0,0],[1,2]]
POINTS = [[0,0],[1,2],[10.5, 4.5],[5,6]]
ELLIPSE_POINTS = [[0,0],[1,2],[3,1]]
CIRCLE_POINTS  = [[0,0],[1,2],[2,1]]
ORIENTATION = [[180, 90]]

def prepare_desired_path(trajectory_type):
    '''
    Prepare desired path
    '''
    if trajectory_type == "dubins":
        shape = Dubins(DUBINS_POINTS,ORIENTATION,2)
    
    if trajectory_type == "circle":
        shape = Ellipse(CIRCLE_POINTS)

    if trajectory_type == "bezier":
        shape = Bezier(POINTS, True)

    if trajectory_type == "ellipse":
        shape = Ellipse(ELLIPSE_POINTS)

    if trajectory_type == "cubic_spline":
        shape = CubicSpline(POINTS)

    if trajectory_type == "bezier_spline":
        shape = BezierSpline(POINTS, order=3)

    samples, yaw_samples = shape.sample_points()
    return samples, yaw_samples

'''
if __name__ == '__main__':
    path = prepare_desired_path("circle")
    print path
'''