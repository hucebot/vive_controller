#!/usr/bin/env python3

import csv, yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

config_file = "/ros_ws/src/ros1_vive_controller/config/config.yaml"

def remove_outliers_zscore(points, z_threshold):
    mean = np.mean(points, axis=0)
    std = np.std(points, axis=0)
    std[std == 0] = 1e-9
    z_scores = np.abs((points - mean) / std)
    mask = (z_scores < z_threshold).all(axis=1)
    
    filtered_points = points[mask]
    outliers = points[~mask]
    return filtered_points, outliers

def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def plot_bounding_box(points, ax, color='green', alpha=0.2):
    min_x, max_x = np.min(points[:,0]), np.max(points[:,0])
    min_y, max_y = np.min(points[:,1]), np.max(points[:,1])
    min_z, max_z = np.min(points[:,2]), np.max(points[:,2])

    corners = np.array([
        [min_x, min_y, min_z],
        [min_x, min_y, max_z],
        [min_x, max_y, min_z],
        [min_x, max_y, max_z],
        [max_x, min_y, min_z],
        [max_x, min_y, max_z],
        [max_x, max_y, min_z],
        [max_x, max_y, max_z]
    ])

    faces = [
        [0,1,3,2],
        [4,5,7,6],
        [0,1,5,4],
        [2,3,7,6], 
        [1,3,7,5], 
        [0,2,6,4] 
    ]

    configurations = read_yaml(config_file)
    if 'workspace' not in configurations:
        configurations['workspace'] = {}
    
    configurations['workspace']['x_min'] = round(float(min_x), 2)
    configurations['workspace']['x_max'] = round(float(max_x), 2)
    configurations['workspace']['y_min'] = round(float(min_y), 2)
    configurations['workspace']['y_max'] = round(float(max_y), 2)
    configurations['workspace']['z_min'] = round(float(min_z), 2)
    configurations['workspace']['z_max'] = round(float(max_z), 2)

    write_yaml(config_file, configurations)

    poly3d = [[corners[idx] for idx in face] for face in faces]
    collection = Poly3DCollection(poly3d, facecolors=color, alpha=alpha, edgecolors='k')
    ax.add_collection3d(collection)

def read_yaml(path):
    with open(path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return {}

def write_yaml(path, data):
    with open(path, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

def plot_convex_hull(points, outliers=None):
    hull = ConvexHull(points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if outliers is not None and len(outliers) > 0:
        ax.scatter(
            outliers[:,0], 
            outliers[:,1], 
            outliers[:,2], 
            color='black', 
            alpha=0.5, 
            label='Outliers'
        )

    ax.scatter(points[:,0], points[:,1], points[:,2], color='blue', alpha=0.5, label='Filtrados')

    for simplex in hull.simplices:
        xs = points[simplex, 0]
        ys = points[simplex, 1]
        zs = points[simplex, 2]

        xs = np.append(xs, xs[0])
        ys = np.append(ys, ys[0])
        zs = np.append(zs, zs[0])
        ax.plot(xs, ys, zs, color='red')

    plot_bounding_box(points, ax, color='green', alpha=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    set_axes_equal(ax)
    plt.legend()
    plt.show()

def main():
    csv_file = read_yaml(config_file)['general']['csv_path'] +  'points.csv'
    points_list = []
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            points_list.append([x, y, z])
    points = np.array(points_list)

    z_threshold = read_yaml(config_file)['general']['z_threshold']
    filtered, outliers = remove_outliers_zscore(points, z_threshold=z_threshold)


    if len(filtered) > 3:
        plot_convex_hull(filtered, outliers)
    else:
        print("Not enough points to compute the Convex Hull")

if __name__ == "__main__":
    main()
