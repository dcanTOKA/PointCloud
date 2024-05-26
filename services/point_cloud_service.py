import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import silhouette_score
import open3d as o3d

from services.interfaces.point_cloud_interface import IPointCloudProcessorInterface


class PointCloudProcessorService(IPointCloudProcessorInterface):
    def __init__(self, settings):
        self.settings = settings
        self.point_cloud = None

    def load_point_cloud(self):
        self.point_cloud = o3d.io.read_point_cloud(self.settings.input_path)
        print("Point cloud loaded from:", self.settings.input_path)
        print(f"Total point numbers: {len(self.point_cloud.points)}")

    def save_point_cloud(self):
        o3d.io.write_point_cloud(self.settings.output_path, self.point_cloud)
        print("Point cloud saved to:", self.settings.output_path)

    def apply_filter(self, filter_type):
        if filter_type == 'voxel_downsample':
            voxel_size = self.settings.voxel_downsample.voxel_size
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size=voxel_size)
            print(f"Total point numbers after voxel_downsample with {voxel_size}: {len(self.point_cloud.points)}")
        elif filter_type == 'statistical_outlier_removal':
            nb_neighbors = self.settings.statistical_outlier_removal.nb_neighbors
            std_ratio = self.settings.statistical_outlier_removal.std_ratio
            self.point_cloud, _ = self.point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                                                              std_ratio=std_ratio)
        else:
            raise ValueError(f"Filter type {filter_type} not supported.")
        print(f"Filter {filter_type} applied with settings: {self.settings}")

    def apply_clustering(self, distance_threshold, min_points_for_nb, log_mode=False):
        labels = np.array(self.point_cloud.cluster_dbscan(eps=distance_threshold, min_points=min_points_for_nb,
                                                          print_progress=log_mode))
        max_label = labels.max()
        print(f"Point cloud has been clustered into {max_label + 1} clusters.")

        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([self.point_cloud])

    def divide_ply(self, num_parts, axis='y'):
        # Noktaları ve renkleri numpy dizisine çevir
        points = np.asarray(self.point_cloud.points)
        colors = np.asarray(self.point_cloud.colors) if self.point_cloud.has_colors() else None
        normals = np.asarray(self.point_cloud.normals) if self.point_cloud.has_normals() else None

        # Seçilen eksene göre noktaları sırala
        indices = np.argsort(points[:, 'xyz'.index(axis)])
        points = points[indices]
        if colors is not None:
            colors = colors[indices]
        if normals is not None:
            normals = normals[indices]

        # Parçaların boyutunu hesapla ve bölgelere ayır
        part_size = len(points) // num_parts

        for i in range(num_parts):
            start_index = i * part_size
            end_index = start_index + part_size if i < num_parts - 1 else len(points)

            # Nokta bulutu parçasını oluştur
            part_pcd = o3d.geometry.PointCloud()
            part_pcd.points = o3d.utility.Vector3dVector(points[start_index:end_index])

            if colors is not None:
                part_pcd.colors = o3d.utility.Vector3dVector(colors[start_index:end_index])

            if normals is not None:
                part_pcd.normals = o3d.utility.Vector3dVector(normals[start_index:end_index])

            # Parçayı .ply formatında kaydet
            output_path = f"{self.settings.output_path}_part_{i}.ply"
            o3d.io.write_point_cloud(output_path, part_pcd)

        print(f"Nokta bulutu {axis} ekseni üzerinden {num_parts} parçaya bölündü ve kaydedildi.")

    def visualize(self, notebook_mode=False):
        if self.point_cloud:
            print("Visualizing point cloud...")
            if notebook_mode:
                o3d.visualization.draw_plotly([self.point_cloud])
            else:
                o3d.visualization.draw_geometries([self.point_cloud])
        else:
            print("No point cloud to visualize. Please load a point cloud first.")
