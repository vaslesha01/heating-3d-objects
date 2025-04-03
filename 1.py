from vedo import *
import heapq
from collections import deque
import numpy as np

# Load 3d bunny
bunny = Mesh("bunny.obj")
bunny.color("white")

# Creating a neighbors dictionary
bunny_vertices = {}
for cell in bunny.cells:
    for i in range(len(cell)):
        vertex = cell[i]
        neighbors_set = set(cell) - {vertex}
        if vertex not in bunny_vertices:
            bunny_vertices[vertex] = set()
        bunny_vertices[vertex].update(neighbors_set)

# Implementation of Breadth-First Search (BFS) with depth fixation
def brf(start, graph, points, max_distance):
    distances = {start: 0.0}
    pq = [(0.0, start)]
    while pq:
        d, vertex = heapq.heappop(pq)
        # If the current distance is greater than the specified maximum, skip
        if d > max_distance:
            continue
        for neighbor in graph.get(vertex, []):
            # Calculate the Euclidean distance between vertices
            edge_length = np.linalg.norm(points[vertex] - points[neighbor])
            new_distance = d + edge_length
            if new_distance < distances.get(neighbor, np.inf) and new_distance < max_distance:
                distances[neighbor] = new_distance
                heapq.heappush(pq, (new_distance, neighbor))
    return distances

# Mouse handler to create the "heat transfer" effect
def on_click(event):

    if event.actor != bunny:
        return
    # Get the 3D position of the click
    pos = event.picked3d
    pts = bunny.points
    # Find the closest vertex to the click point
    dists = np.linalg.norm(pts - np.array(pos), axis=1)
    nearest_vertex = int(np.argmin(dists))

    # Set the maximum search distance
    max_distance_param = 1.0
    distances = brf(nearest_vertex, bunny_vertices, pts, max_distance_param)

    # Distance normalization for gradient coloring
    max_distance_found = max(distances.values()) if distances else 1.0
    new_colors = np.ones((len(pts), 4), dtype=int) * 255
    for v, d in distances.items():
        norm = d / max_distance_found  # normalize the distance from 0 to 1
        col = colors.color_map(norm, 'hot', 0, 1)
        col = [int(255 * x) for x in col] + [255]
        new_colors[v] = col

    bunny.pointcolors = new_colors  # updating vertex colors
    plt.render()  # redraw plt


plt = Plotter(bg='black')
plt.add(bunny)
plt.add_callback("mouse click", on_click)

plt.show().close()