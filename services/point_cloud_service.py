import numpy as np

from services.interfaces.point_cloud_interface import IPointCloudProcessorInterface
import open3d as o3d


class PointCloudProcessorService(IPointCloudProcessorInterface):
    def __init__(self, settings):
        self.settings = settings
        self.point_cloud = None

    def load_point_cloud(self):
        self.point_cloud = o3d.io.read_point_cloud(self.settings.input_path)
        print("Point cloud loaded from:", self.settings.input_path)

    def save_point_cloud(self):
        o3d.io.write_point_cloud(self.settings.output_path, self.point_cloud)
        print("Point cloud saved to:", self.settings.output_path)

    def apply_filter(self, filter_type):
        if filter_type == 'voxel_downsample':
            voxel_size = self.settings.voxel_downsample.voxel_size
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size=voxel_size)
        elif filter_type == 'statistical_outlier_removal':
            nb_neighbors = self.settings.statistical_outlier_removal.nb_neighbors
            std_ratio = self.settings.statistical_outlier_removal.std_ratio
            self.point_cloud, _ = self.point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        else:
            raise ValueError(f"Filter type {filter_type} not supported.")
        print(f"Filter {filter_type} applied with settings: {self.settings}")

    def visualize(self):
        if self.point_cloud:
            print("Visualizing point cloud...")
            o3d.visualization.draw_geometries([self.point_cloud])
        else:
            print("No point cloud to visualize. Please load a point cloud first.")
