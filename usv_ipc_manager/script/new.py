import matplotlib.pyplot as plt
import numpy as np
import heapq

class Node:
    def __init__(self, x, y, theta, cost, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def heuristic(node, goal):
    return np.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)

def a_star(start, goal, obstacles, grid_size, step_size, theta_step, max_theta):
    open_list = []
    closed_list = set()
    all_nodes = []
    heapq.heappush(open_list, (0, start))

    while open_list:
        _, current = heapq.heappop(open_list)
        closed_list.add((current.x, current.y, current.theta))
        all_nodes.append(current)

        if np.sqrt((current.x - goal.x)**2 + (current.y - goal.y)**2) < grid_size:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1], all_nodes

        for dtheta in [-theta_step, 0, theta_step]:
            theta = current.theta + dtheta
            # 限制theta角的最大值
            if theta > max_theta:
                theta -= 2 * np.pi
            elif theta < -max_theta:
                theta += 2 * np.pi

            dx = step_size * np.cos(theta)
            dy = step_size * np.sin(theta)
            x = current.x + dx
            y = current.y + dy

            if (x, y) in obstacles:
                continue

            neighbor = Node(x, y, theta, current.cost + step_size, current)
            if (neighbor.x, neighbor.y, neighbor.theta) in closed_list:
                continue

            heapq.heappush(open_list, (neighbor.cost + heuristic(neighbor, goal), neighbor))

    return None, all_nodes

def plot_path(path, all_nodes, obstacles, start, goal):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    for obs in obstacles:
        ax.plot(obs[0], obs[1], 'ks')

    x_all_nodes = [node.x for node in all_nodes]
    y_all_nodes = [node.y for node in all_nodes]
    ax.plot(x_all_nodes, y_all_nodes, 'g.', markersize=2, label='Generated Nodes')

    x_path = [node.x for node in path]
    y_path = [node.y for node in path]
    ax.plot(x_path, y_path, 'b-', linewidth=2, label='Path')

    ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
    ax.plot(goal.x, goal.y, 'ro', markersize=10, label='Goal')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('A* Path Planning with Dynamics')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    start = Node(0, 0, 0, 0)
    goal = Node(10, 5, 0, 0)
    obstacles = {(3, 3), (6, 3), (8, 5), (5, 5)}
    grid_size = 0.5
    step_size = 0.5
    theta_step = np.pi / 4
    max_theta = np.pi

    path, all_nodes = a_star(start, goal, obstacles, grid_size, step_size, theta_step, max_theta)
    if path:
        plot_path(path, all_nodes, obstacles, start, goal)
    else:
        print("No path found")