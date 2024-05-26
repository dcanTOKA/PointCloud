from dataclasses import dataclass
from decouple import config


@dataclass
class VoxelDownsampleSettings:
    voxel_size: float = float(config("VOXEL_SIZE"))


@dataclass
class StatisticalOutlierRemovalSettings:
    nb_neighbors: int = int(config("NB_NEIGHBORS"))
    std_ratio: float = float(config("STD_RATIO"))


@dataclass
class PointCloudSettings:
    input_path: str = config("INPUT_POINT_CLOUD_PATH")
    output_path: str = config("OUTPUT_POINT_CLOUD_PATH")
    voxel_downsample: VoxelDownsampleSettings = VoxelDownsampleSettings()
    statistical_outlier_removal: StatisticalOutlierRemovalSettings = StatisticalOutlierRemovalSettings()
