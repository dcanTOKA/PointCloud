from abc import ABCMeta, abstractmethod


class IPointCloudProcessorInterface(metaclass=ABCMeta):
    @abstractmethod
    def load_point_cloud(self):
        raise NotImplemented

    @abstractmethod
    def save_point_cloud(self):
        raise NotImplemented

    @abstractmethod
    def apply_filter(self, filter_type):
        raise NotImplemented

    @abstractmethod
    def apply_clustering(self, distance_threshold, min_points_for_nb, log_mode=False):
        raise NotImplemented

    @abstractmethod
    def visualize(self, notebook_mode=False):
        raise NotImplemented
