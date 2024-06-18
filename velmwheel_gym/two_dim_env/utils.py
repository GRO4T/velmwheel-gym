""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar/blob/main/robot2d/utils.py"""

from math import cos, pi, pow, sin, sqrt


def sgn(x):
    if x < 0:
        return -1
    else:
        return 1


def obtain_intersection_points(x1, y1, x2, y2, xc, yc, rc):
    # https://mathworld.wolfram.com/Circle-LineIntersection.html
    x1n = x1 - xc
    x2n = x2 - xc

    y1n = y1 - yc
    y2n = y2 - yc

    dx = x2n - x1n
    dy = y2n - y1n

    dr = sqrt(dx * dx + dy * dy)
    D = x1n * y2n - x2n * y1n

    dr_pow = dr * dr

    delta = rc * rc * dr_pow - D * D

    if delta >= 0:
        sqrt_delta = sqrt(delta)
        xi1 = (D * dy + sgn(dy) * dx * sqrt_delta) / dr_pow
        yi1 = -(D * dx - abs(dy) * sqrt_delta) / dr_pow

        xi2 = (D * dy - sgn(dy) * dx * sqrt_delta) / dr_pow
        yi2 = -(D * dx + abs(dy) * sqrt_delta) / dr_pow

        dis1 = sqrt(((xi1 + xc - x1) ** 2) + ((yi1 + yc - y1) ** 2))
        dis2 = sqrt(((xi2 + xc - x1) ** 2) + ((yi2 + yc - y1) ** 2))

        if dis2 > dis1:
            return True, [xi1 + xc, yi1 + yc]
        else:
            return True, [xi2 + xc, yi2 + yc]
    else:
        return False, -1


def validate_point(x, y, xo, yo, th, max_range):
    # Check if it is the same direction
    if sgn(y) != sgn(sin(th)):
        return False
    elif sgn(x) != sgn(cos(th)):
        return False
    # Check it is within the range
    elif sqrt(x**2 + y**2) > max_range:
        return False

    elif sqrt(x**2 + y**2) > sqrt(xo**2 + yo**2):
        return False
    else:
        return True


def clip_angle(th):
    try:
        while th <= -pi:
            th = th + 2 * pi
        while th > pi:
            th = th - 2 * pi
        return th
    except:
        raise Exception("th angle was not a number")


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y
