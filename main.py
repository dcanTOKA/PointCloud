from config.settings import PointCloudSettings, VoxelDownsampleSettings, StatisticalOutlierRemovalSettings
from services.point_cloud_service import PointCloudProcessorService


def main():
    settings = PointCloudSettings()

    processor = PointCloudProcessorService(settings)
    processor.load_point_cloud()
    processor.visualize()


if __name__ == "__main__":
    main()
