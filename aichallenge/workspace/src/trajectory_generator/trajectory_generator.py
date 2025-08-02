import xml.etree.ElementTree as ET
import numpy as np
import csv
import math
from scipy.optimize import minimize

MU = 1.0
G = 9.81

def yaw_to_quaternion(yaw):
    return(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def compute_centerline(left_pts, right_pts):
    return (left_pts + right_pts) / 2.0

def main():
    tree = ET.parse("course.osm")
    root = tree.getroot()

    nodes = load_nodes(root)
    ways = load_way_nodes(root)
    lanelets = extract_lanelets(root)

    all_waypoints = []

    for ll in lanelets:
        left_way = ways[ll["left"]]
        right_way = ways[ll["right"]]

        left_pts = np.array([nodes[nid] for nid in left_way])
        right_pts = np.array([nodes[nid] for nid in right_way])
        centerline = compute_centerline(left_pts, right_pts)
    
        normals = []
        for i in range(len(centerline)):
            prev = centerline[i-1][:2]
            next = centerline[(i+1)%len(centerline)][:2]
            tangent = next - prev
            normal = np.array([-tangent[1], tangent[0]])
            normal = normal / np.linalg.norm(normal)
            normals.append(normal)
        normal = np.array(normals)
    

if __name__ == "__main__":
    main()

