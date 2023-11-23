import shapely.geometry as sg
import numpy as np
import matplotlib.pyplot as plt

def generate_non_intersecting_polygons(n, xlim, ylim, max_attempts=100):
    polygons = []

    for _ in range(n):
        attempt = 0
        intersecting = True

        while intersecting and attempt < max_attempts:
            num_vertices = np.random.randint(3, 8)
            vertices = np.random.uniform(xlim[0], xlim[1], size = (num_vertices, 2))
            polygon = sg.Polygon(vertices)
            intersecting = any(existing_polygon.intersects(polygon) for existing_polygon in polygons)

            attempt  = attempt + 1

        if attempt < max_attempts:
            polygons.append(polygon)
        else:
            print("Max attempts reached for polygon,Skipping.")

    return polygons

fig, ax = plt.subplots()
xlim = ax.set_xlim([0, 100])
ylim = ax.set_ylim([0, 100])

n  = int(input("Enter number of polygons = "))
polygons_list = generate_non_intersecting_polygons(n,xlim,ylim)

for polygon in polygons_list:
    exterior_coords = list(zip(polygon.exterior.xy[0], polygon.exterior.xy[1]))
    ax.add_patch(plt.Polygon(exterior_coords, facecolor="black", edgecolor="black"))

plt.show()
