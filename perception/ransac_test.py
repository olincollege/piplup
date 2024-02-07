import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pyransac3d as pyrsc
from pydrake.all import *
from pydrake.geometry import Meshcat, StartMeshcat, Box, Cylinder, Sphere
import time


def fit_box(meshcat, cloud, visuals=False):
    cuboid_fit = pyrsc.Cuboid()

    if visuals:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.visualization.draw_geometries([pcd])

    print("-----Fitting box-----")
    eqs, inliers = cuboid_fit.fit(cloud, thresh=0.01, maxIteration=1000)
    print("-----Fit Complete-----")

    print(len(inliers))

    A = eqs[:, :-1]
    b = -eqs[:, -1]
    intersection = np.linalg.solve(A, b)
    shifted_pts = cloud[inliers] - intersection
    dots = np.matmul(A, np.transpose(shifted_pts))
    dots_abs = np.abs(dots)
    dims = dots_abs.max(axis=1)
    dims_idx = np.argmax(dots_abs, axis=1)
    normal_signs = np.sign([dots[i, dims_idx[i]] for i in range(3)])
    limits = cloud[inliers][dims_idx]

    normals = np.transpose(A)
    box_center = intersection
    for i in range(3):
        box_center = np.add(
            box_center,
            A[i, :] / np.linalg.norm(A[i, :]) * dims[i] / 2 * normal_signs[i],
        )

    box_fit = Box(dims)
    meshcat.SetObject("box_fit", box_fit)
    meshcat.SetTransform(
        "box_fit", RigidTransform(R=RotationMatrix(normals), p=box_center)
    )

    if visuals:
        plane = pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
        not_plane = pcd.select_by_index(inliers, invert=True)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
        o3d.visualization.draw_geometries([mesh, plane, not_plane])

        intersection_pcd = o3d.geometry.PointCloud()
        intersection_pcd.points = o3d.utility.Vector3dVector([intersection, box_center])
        intersection_pcd = intersection_pcd.paint_uniform_color([0, 1, 0])
        fit_pcd = o3d.geometry.PointCloud()
        fit_pcd.points = o3d.utility.Vector3dVector(limits)
        fit_pcd = fit_pcd.paint_uniform_color([0, 0, 1])
        o3d.visualization.draw_geometries([mesh, intersection_pcd, fit_pcd])


def fit_cylinder(meshcat, cloud, visuals=False):
    cylinder_fit = pyrsc.Cylinder()

    if visuals:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.visualization.draw_geometries([pcd])

    print("-----Fitting cylinder-----")
    center, axis, radius, inliers = cylinder_fit.fit(
        cloud, thresh=0.05, maxIteration=1000
    )
    print("-------Fit complete-------")

    print(len(inliers))

    shifted_pts = cloud[inliers] - center
    dots = np.matmul(axis, np.transpose(shifted_pts))
    dots_abs = np.abs(dots)
    cyl_length = dots_abs.max(axis=0) * 2

    cylinder_mesh = Cylinder(radius=radius, length=cyl_length)
    meshcat.SetObject("cylinder_fit", cylinder_mesh)
    meshcat.SetTransform(
        "cylinder_fit",
        RigidTransform(theta_lambda=AngleAxis(angle=0, axis=axis), p=center),
    )

    if visuals:
        plane = pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
        not_plane = pcd.select_by_index(inliers, invert=True)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
        o3d.visualization.draw_geometries([mesh, plane, not_plane])


def fit_sphere(meshcat, cloud, visuals=False):
    sphere_fit = pyrsc.Sphere()

    if visuals:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.visualization.draw_geometries([pcd])

    print("-----Fitting sphere-----")
    center, radius, inliers = sphere_fit.fit(cloud, thresh=0.01, maxIteration=1000)
    print("-------Fit complete-------")

    print(len(inliers))

    sphere_mesh = Sphere(radius=radius)
    meshcat.SetObject("sphere_fit", sphere_mesh)
    meshcat.SetTransform(
        "sphere_fit",
        RigidTransform(theta_lambda=AngleAxis(angle=0, axis=[0, 0, 1]), p=center),
    )

    if visuals:
        plane = pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
        not_plane = pcd.select_by_index(inliers, invert=True)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
        o3d.visualization.draw_geometries([mesh, plane, not_plane])


def main():
    point_cloud = np.transpose(
        np.load("/home/ali1/code/piplup/test_data/box_point_cloud.npy")
    )
    filtered_cloud = point_cloud[~np.any(np.isinf(point_cloud), axis=1)]
    filtered_cloud = filtered_cloud[
        (0.2 < filtered_cloud[:, 0]) & (filtered_cloud[:, 0] < 0.7)
    ]
    filtered_cloud = filtered_cloud[
        (-0.1 < filtered_cloud[:, 1]) & (filtered_cloud[:, 1] < 0.1)
    ]

    meshcat: Meshcat = StartMeshcat()

    fit_box(meshcat, filtered_cloud, visuals=False)
    fit_cylinder(meshcat, filtered_cloud, visuals=False)
    fit_sphere(meshcat, filtered_cloud, visuals=False)

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
