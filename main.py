# 2) b.

import matplotlib.pyplot as plt
from math import hypot
import random


class Node:
    def __init__(self, position: (), parent: ()):
        self.position = tuple(position)
        self.parent = parent
        self.g = 0  # Distance to start node
        self.h = 0  # Distance to goal node
        self.f = 0  # Total cost

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __repr__(self):
        return f'{self.position}, {self.f}'


def heuristic(n, e):
    d = hypot(e[0] - n[0], e[1] - n[1])
    return d


def on_segment(p, q, r):
    if max(p[0], q[0]) >= r[0] >= min(p[0], q[0]) and max(p[1], q[1]) >= r[1] >= min(p[1], q[1]):
        return True
    return False


def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    return 1 if val > 0 else -1


def intersects(seg1, seg2):
    p1 = seg1[0][0], seg1[1][0]
    q1 = seg1[0][1], seg1[1][1]
    p2 = seg2[0][0], seg2[1][0]
    q2 = seg2[0][1], seg2[1][1]

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 == 0 and on_segment(p1, q1, p2):
        return False
    if o2 == 0 and on_segment(p1, q1, q2):
        return False
    if o3 == 0 and on_segment(p2, q2, p1):
        return False
    if o4 == 0 and on_segment(p2, q2, q1):
        return False

    if o1 != o2 and o3 != o4:
        return True

    return False


def make_line(p1, p2):
    return (p1[0], p2[0]), (p1[1], p2[1])


def get_polygon_edges(polygon_list):
    edges = [(make_line(polygon_list[j].get_xy()[i], polygon_list[j].get_xy()[i + 1]))
             for j in range(len(polygon_list))
             for i in range(len(polygon_list[j].get_xy()) - 1)]

    for poly in range(len(polygon_list)):
        for diag1 in range(len(polygon_list[poly].get_xy()) - 1):
            for diag2 in range(diag1 + 2, len(polygon_list[poly].get_xy()) - 1):
                diagonal = make_line(polygon_list[poly].get_xy()[diag1], polygon_list[poly].get_xy()[diag2])
                edges.append(diagonal)
    return edges


def a_star(start, end, poly_list):
    to_visit = []
    visited = []

    start_node = Node(start, None)
    goal_node = Node(end, None)

    to_visit.append(start_node)
    while len(to_visit) > 0:

        to_visit.sort()
        current_node = to_visit.pop(0)
        visited.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            distance = []
            while current_node != start_node:
                path.append(current_node.position)
                distance.append(current_node.g)
                current_node = current_node.parent
            distance.append(start_node.g)
            path.append(start_node.position)
            for point in range(len(path) - 1):
                line = make_line(path[point], path[point + 1])
                plt.plot(line[0], line[1])
            # Return reversed path
            return path[::-1], distance[::-1]

        neighbours = []

        edges = get_polygon_edges(poly_list)
        # for each polygon
        for poly in poly_list:
            # for each point in polygon
            for i in range(len(poly.get_xy()) - 1):
                visible = True
                # line from current point to each point in polygon
                line = make_line(current_node.position, poly.get_xy()[i])
                middle = sum(line[0]) / 2, sum(line[1]) / 2
                if poly.get_path().contains_point(middle):
                    visible = False
                else:
                    # for each edge of all the polygons
                    for edge in edges:
                        # if the line from the current point to a corner of the polygon intersects an edge of any poly
                        # then that point is not visible
                        if intersects(line, edge) is True:
                            visible = False
                            break

                if visible:
                    neighbours.append(poly.get_xy()[i])
                    # print(poly.get_xy()[i])  # the points we can see from the current point
                    # plt.plot(line[0], line[1])  # drawing line to the points we can see from the current point

        visible = True
        line_to_goal = make_line(current_node.position, goal_node.position)

        # for each edge of all the polygons
        for edge in edges:
            # if the line from the current point to a corner of the polygon intersects an edge of any poly
            # then that point is not visible
            if intersects(line_to_goal, edge) is True:
                visible = False
                break
        if visible:
            neighbours.append(goal_node.position)
            # print(poly.get_xy()[i])  # the points we can see from the current point

        # Loop neighbors
        for neighbour in neighbours:
            # Create a neighbor node
            neighbour_node = Node(neighbour, current_node)

            # Check if the neighbor is in the closed list
            if neighbour_node in visited:
                continue

            # Generate heuristics
            neighbour_node.g = heuristic(current_node.position, neighbour_node.position) + current_node.g
            neighbour_node.h = heuristic(neighbour_node.position, goal_node.position)
            neighbour_node.f = neighbour_node.g + neighbour_node.h

            # Check if neighbor is in open list and if it has a lower f value
            if add_to_open(to_visit, neighbour_node):
                to_visit.append(neighbour_node)

    return None


# Check if a neighbor should be added to open list
def add_to_open(to_visit, neighbor):
    for node in to_visit:
        if neighbor == node and neighbor.f >= node.f:
            return False
    return True


def draw_polygons(poly_coords):
    poly_list = []
    fig, ax = plt.subplots()

    for coord in poly_coords:
        poly = plt.Polygon(coord)
        poly_list.append(poly)
        ax.add_patch(poly)

    return poly_list


def main():
    poly_coords = [
        [(1, -1), (10, -1), (10, 1), (1, 1), (1, -1)],
        [(11, 0), (13, 1), (10, 3), (11, 0)],
        [(14, 0), (16, -1), (18, 0), (18, 1), (16, 2), (14, 1), (14, 0)],
        [(0, 3), (4, 2), (6, 5), (3, 8), (-1, 6), (0, 3)],
        [(6, 2), (9, 2), (7, 6), (6, 2)],
        [(9, 4), (12, 7), (10, 8), (9, 7), (9, 4)],
        [(13, 3), (15, 3), (15, 8), (13, 8), (13, 3)],
        [(16, 7), (18, 8), (20, 6), (18, 2), (16, 7)]
    ]
    poly_coords_copy = [[], [], [], [], [], [], [], []]
    for i in range(len(poly_coords)):
        for j in poly_coords[i]:
            j_list = list(j)
            j_list[0] *= 40
            j_list[1] *= 40
            j = tuple(j_list)
            poly_coords_copy[i].append(j)

    poly_coords = poly_coords_copy

    poly_list = draw_polygons(poly_coords)

    end = (800, 310)
    plt.plot(end[0], end[1], 'ro')
    plt.annotate("Goal", end)

    total_points = 0
    i = 1
    while i <= 10:
        ok = True
        x, y = random.randint(-30, 700), random.randint(-60, 300)
        start = (x, y)
        for poly in poly_list:
            if poly.get_path().contains_point(start):
                ok = False
                break
        if ok is False:
            continue
        plt.plot(start[0], start[1], 'bo')
        plt.annotate(f"Start{i}", start)
        path, distance = a_star(start, end, poly_list)
        points_lost = distance[-1]
        total_points += 1000 - points_lost
        # print("Points: ", 1000 - points_lost)

        i += 1
    print(f'Total points: {total_points}')

    plt.show()


if __name__ == '__main__':
    main()
