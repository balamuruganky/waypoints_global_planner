#!/usr/bin/env python
from path_planning.Bezier import Bezier
from path_planning.BezierSpline import BezierSpline
from path_planning.CubicSpline import CubicSpline
from path_planning.Dubins import Dubins
from path_planning.Line import Line
from path_planning.Ellipse import Ellipse

DUBINS_POINTS = [[0,0],[3,5]]
POINTS = [[0,0],[1,-2],[10.5,-4.5],[5,6],[10,15],[25,30]]
ELLIPSE_POINTS = [[0,0],[1,2],[3,1]]
CIRCLE_POINTS  = [[4,4],[6,9],[9,6]]
ORIENTATION = [[180, 90]]
LINE_POINTS = [[0,0],[7,-12]]

def prepare_desired_path(trajectory_type):
    '''
    Prepare desired path
    '''
    if trajectory_type == "dubins":
        shape = Dubins(DUBINS_POINTS,ORIENTATION,5)
    
    if trajectory_type == "circle":
        shape = Ellipse(CIRCLE_POINTS)

    if trajectory_type == "bezier":
        shape = Bezier(POINTS, False)

    if trajectory_type == "ellipse":
        shape = Ellipse(ELLIPSE_POINTS)

    if trajectory_type == "cubic_spline":
        shape = CubicSpline(POINTS)

    if trajectory_type == "bezier_spline":
        shape = BezierSpline(POINTS, order=3, is_periodic=False)

    if trajectory_type == "line":
        shape = Line(LINE_POINTS, is_periodic=False)

    samples, yaw_samples = shape.sample_points()
    return samples, yaw_samples

'''
if __name__ == '__main__':
    path = prepare_desired_path("circle")
    print path
'''