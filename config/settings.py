from dataclasses import dataclass
from decouple import config


@dataclass
class VoxelDownsampleSettings:
    voxel_size: float = 0.02


@dataclass
class StatisticalOutlierRemovalSettings:
    nb_neighbors: int = 20
    std_ratio: float = 2.0


@dataclass
class PointCloudSettings:
    input_path: str = config("INPUT_POINT_CLOUD_PATH")
    output_path: str = config("OUTPUT_POINT_CLOUD_PATH")
    voxel_downsample: VoxelDownsampleSettings = VoxelDownsampleSettings()
    statistical_outlier_removal: StatisticalOutlierRemovalSettings = StatisticalOutlierRemovalSettings()
