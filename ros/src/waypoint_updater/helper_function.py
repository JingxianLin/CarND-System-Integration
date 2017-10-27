import math


def dist_two_points(x1, y1, x2, y2):
    """Find distance between two points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def closest_waypoint(x0, y0, map_x, map_y):
    """Find nearest waypoint"""
    if len(map_x) == 0 or len(map_y) == 0:
        return None

    try:
        closest_wp_dist = 999999.9
        closest_wp = 0
        for i in range(min(len(map_x), len(map_y))):
            dist = dist_two_points(x0, y0, map_x[i], map_y[i])
            if dist < closest_wp_dist:
                closest_wp_dist = dist
                closest_wp = i

        return closest_wp
    except IndexError:
        return None