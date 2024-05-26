from config.settings import PointCloudSettings, VoxelDownsampleSettings, StatisticalOutlierRemovalSettings
from services.point_cloud_service import PointCloudProcessorService


def main():
    settings = PointCloudSettings()

    processor = PointCloudProcessorService(settings)
    processor.load_point_cloud()
    # processor.divide_ply(5)
    processor.apply_filter("voxel_downsample")
    # processor.visualize()
    processor.apply_clustering(distance_threshold=2.02, min_points_for_nb=500, log_mode=True)


if __name__ == "__main__":
    main()
