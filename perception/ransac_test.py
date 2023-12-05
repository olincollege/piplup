
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

def main():
    point_cloud = np.transpose(np.load('/home/ali1/code/piplup/box_point_cloud.npy'))
    filtered_cloud = point_cloud[~np.any(np.isinf(point_cloud), axis=1)]
    filtered_cloud = filtered_cloud[(0.2 < filtered_cloud[:,0]) & (filtered_cloud[:,0] < 0.7)]
    filtered_cloud = filtered_cloud[(-0.1 < filtered_cloud[:,1]) & (filtered_cloud[:,1] < 0.1)]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(filtered_cloud)
    # o3d.visualization.draw_geometries([pcd])

    cuboid_fit = pyrsc.Cuboid()

    best_eq, best_inliers = cuboid_fit.fit(filtered_cloud, thresh=0.02, maxIteration=5000)
    plane = pcd.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
    not_plane = pcd.select_by_index(best_inliers, invert=True)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([mesh, plane, not_plane])

    x_lims = (min(filtered_cloud[:,0]), max(filtered_cloud[:,0]))
    y_lims = (min(filtered_cloud[:,1]), max(filtered_cloud[:,1]))
    z_lims = (min(filtered_cloud[:,2]), max(filtered_cloud[:,2]))
    com = [np.average(filtered_cloud[:,0]), np.average(filtered_cloud[:,1]), np.average(filtered_cloud[:,2])]
    x = np.linspace(x_lims[0], x_lims[1], 100)
    y = np.linspace(y_lims[0], y_lims[1], 100)
    xx, yy = np.meshgrid(x,y)

    zs = []
    for eq in best_eq:
        print(eq)
        zs.append((-xx*eq[0] - yy*eq[1] - eq[3])/eq[2])

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # for z in zs:
    #     z_filtered = z
    #     # z_filtered[(z < z_lims[0]) & (z > z_lims[1])] = np.nan

    #     ax.plot_surface(x, y, z_filtered)
    ax.plot_surface(x,y,zs[0])

    for eq in best_eq:
        ax.plot([com[0],com[0]+eq[0]], [com[1],com[1]+eq[1]], [com[2],com[2]+eq[2]])

    # ax.scatter(filtered_cloud[:,0], filtered_cloud[:,1], filtered_cloud[:,2])
    ax.set_xlim([x_lims[0] - 0.1, x_lims[1] + 0.1])
    ax.set_ylim([y_lims[0] - 0.1, y_lims[1] + 0.1])
    ax.set_zlim([z_lims[0] - 0.1, z_lims[1] + 0.1])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

if __name__ == "__main__":
    main()