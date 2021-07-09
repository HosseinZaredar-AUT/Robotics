import numpy as np
import math

def generate_ellipse(n=120):

    ellipse = np.array([[1], [3]])

    degree = (2 * np.pi) / n 
    rotation_matrix = np.array([
        [math.cos(degree), -math.sin(degree)],
        [math.sin(degree), math.cos(degree)]
    ])

    points = []

    current = np.array([[ellipse[0, 0]], [0]])
    for _ in range(n + 1):
        points.append(current * ellipse)
        current = rotation_matrix @ current

    return points


def generate_spiral(n=100, cycles=3):

    growth_factor = 0.1
    degree = (2 * np.pi) / n
    theta = 0

    points = []
    for _ in range(n * cycles + 1):
        r = growth_factor * theta
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        points.append(np.array([[x], [y]]))
        theta += degree

    return points
