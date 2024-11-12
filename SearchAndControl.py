import random
import heapq
import copy
import math

class Node:
    def __init__(self, x, y, g, h, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent = parent
        self.f = g + h


def heuristic(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def valid_space(x, y, n, m):
    return 0 <= x < n and 0 <= y < m

def show_grid(grid):
    for row in grid:
        for col in row:
            print(col, end=" ")
        print()


def search(grid, start, goal):
    moves = [[-1, 0], [1,0], [0, -1], [0, 1]]
    visited = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    startNode = Node(start[0], start[1], 0, heuristic(start[0], start[1], goal[0], goal[1]), None)
    pq = []
    heapq.heappush(pq, (startNode.f, 0, startNode))
    count = 1

    while pq:
        _, _, current = heapq.heappop(pq)
        if [current.x, current.y] == goal:
            # Recreate path to goal
            path = []
            while current:
                path.append([current.x, current.y])
                current = current.parent
            path.reverse()
            return path
        for move in moves:
            new_x = current.x + move[0]
            new_y = current.y + move[1]
            if valid_space(new_x, new_y, len(grid), len(grid[0])) and grid[new_x][new_y] != 1 and visited[new_x][new_y] != 1:
                visited[new_x][new_y] += 1
                neighbor = Node(new_x, new_y, current.g + 1, heuristic(new_x, new_y, goal[0], goal[1]), current)
                heapq.heappush(pq, (neighbor.f, count, neighbor))
                count += 1

    # No path found
    return None


def smooth(path, weight_data=0.1, weight_smooth=0.1, tolerance=0.000001):
    if not path:
        return None
    spath = copy.deepcopy(path)
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path) - 1):
            for j in range(len(path[0])):
                aux = spath[i][j]
                spath[i][j] += weight_data * (path[i][j] - spath[i][j])
                spath[i][j] += weight_smooth * (spath[i - 1][j] + spath[i + 1][j] - (2.0 * spath[i][j]))
                if i >= 2:
                    spath[i][j] += 0.5 * weight_smooth * ((2.0 * spath[i - 1][j]) - spath[i - 2][j] - spath[i][j])
                if i < len(path) - 3:
                    spath[i][j] += 0.5 * weight_smooth * ((2.0 * spath[i + 1][j]) - spath[i + 2][j] - spath[i][j])
                change += abs(aux - spath[i][j])

    return spath


class Robot:
    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.steering_drift = 0.0
        self.distance_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions = 0
        self.num_steps = 0

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * math.pi)

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    def set_steering_drift(self, new_drift):
        self.steering_drift = new_drift

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = math.sqrt((self.x - float(i)) ** 2 + (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return True
        return False

    def check_goal(self, goal, threshold=1.0):
        dist = math.sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=math.pi/4.0):
        # Make a new robot
        res = Robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions = self.num_collisions
        res.num_steps = self.num_steps + 1


        self.num_steps += 1
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        # Apply noise and drift to movement
        steering2 = random.gauss(steering, self.steering_noise)
        steering2 += self.steering_drift
        distance2 = random.gauss(distance, self.distance_noise)
        if distance2 < 0:
            distance2 = 0
        turn = math.tan(steering2) * distance2 / self.length
        if abs(turn) < tolerance:
            # approximate by straight line motion
            res.x = self.x + (distance2 * math.cos(self.orientation))
            res.y = self.y + (distance2 * math.sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * math.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (math.sin(self.orientation) * radius)
            cy = self.y + (math.cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * math.pi)
            res.x = cx + (math.sin(res.orientation) * radius)
            res.y = cy - (math.cos(res.orientation) * radius)
        return res

    def __str__(self):
        return (f'[x={self.x:.5f} y={self.y:.5f} orient={self.orientation:.5f}]')

    def sense(self):
        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    def measurement_prob(self, measurement):
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = (math.exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0)
                 / math.sqrt(2.0 * math.pi * (self.measurement_noise ** 2)))
        error *= (math.exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0)
                  / math.sqrt(2.0 * math.pi * (self.measurement_noise ** 2)))
        return error
class Particle:
    # Create set based on initial position
    def __init__(self, x, y, theta,
                 steering_noise, distance_noise, measurement_noise, N=100):
        self.N = N
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

        self.data = []
        for i in range(self.N):
            r = Robot()
            r.set(x, y, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)

    # Estimate position from particle set
    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0
        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. Normalize
            # around the first particle to be more robust
            orientation += (((self.data[i].orientation - self.data[0].orientation + math.pi) % (2.0 * math.pi))
                            + self.data[0].orientation - math.pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # Move all particles
    def move(self, steer, speed):
        new_data = []
        for i in range(self.N):
            r = self.data[i].move(steer, speed)
            new_data.append(r)
        self.data = new_data

    # Sense and resample
    def sense(self, Z):
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        p2 = []
        index = int(random.random() * self.N)
        beta = 0.0
        max_w = max(w)
        for i in range(self.N):
            beta += random.random() * 2.0 * max_w
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p2.append(self.data[index])
        self.data = p2


def run(grid, start, goal, spath, params, timeout=1000, print_flag=False):
    myRobot = Robot()
    myRobot.set(start[0], start[1], 0)
    # 2 degree steering bias
    myRobot.set_steering_drift(2.0 * (math.pi / 180.0))
    myRobot.set_noise(0.5 * (math.pi / 180.0), 0.05, 0.1)
    filter = Particle(myRobot.x, myRobot.y, myRobot.orientation,myRobot.steering_noise,
                      myRobot.distance_noise, myRobot.measurement_noise)

    speed = 0.1
    err = 0.0
    N = 0
    crosstrack_error = 0.0
    int_crosstrack_error = 0.0
    index = 0
    while not myRobot.check_goal(goal) and N < timeout:

        estimate = filter.get_position()
        #estimate = [myRobot.x, myRobot.y]

        diff_crosstrack_error = -crosstrack_error
        # Determine line segments
        dx = spath[index + 1][0] - spath[index][0]
        dy = spath[index + 1][1] - spath[index][1]
        rx = estimate[0] - spath[index][0]
        ry = estimate[1] - spath[index][1]
        # Robot postion projected onto line segment
        u = ((rx * dx) + (ry * dy)) / ((dx * dx) + (dy * dy));
        crosstrack_error = ((ry * dx) - (rx * dy)) / ((dx * dx) + (dy * dy))
        # Moved past current path segment
        if u > 1.0:
            index += 1
        diff_crosstrack_error += crosstrack_error
        int_crosstrack_error += crosstrack_error
        # Apply PID control
        steer = -params[0] * crosstrack_error - params[1] * diff_crosstrack_error - params[2] * int_crosstrack_error

        # Move and sense environment
        myRobot = myRobot.move(steer, speed)
        filter.move(steer, speed)
        Z = myRobot.sense()
        filter.sense(Z)

        err += crosstrack_error ** 2
        N += 1
        if myRobot.check_collision(grid):
            print("### Collision ###")
        if print_flag:
            print(myRobot)

    print(f"Reached goal: {myRobot.check_goal(goal)}, # Collisions: {myRobot.num_collisions}, # steps: {myRobot.num_steps}")


if __name__ == '__main__':
    n = 5
    m = 6

    # 30% chance of obstacle at any space
    grid = [[(1 if random.random() < 0.3 else 0) for col in range(m)] for row in range(n)]

    test = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 1],
            [0, 1, 0, 1, 0, 0]]
    #grid = test

    start = [0, 0]
    goal = [n - 1, m - 1]
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0
    show_grid(grid)

    path = search(grid, start, goal)
    if not path:
        print("No path found")
        exit()
    spath = smooth(path)
    params = [2.0, 6.0, 0.0]
    run(grid, start, goal, spath, params)



