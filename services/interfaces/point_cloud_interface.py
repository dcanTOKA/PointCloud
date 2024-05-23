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
