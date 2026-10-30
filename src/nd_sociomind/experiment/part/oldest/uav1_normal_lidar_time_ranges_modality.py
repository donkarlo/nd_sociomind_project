import numpy as np


class Uav1NormalLidarTimeRangesModality:
    def __new__(cls, slc: slice = None):
        instance = super().__new__(cls)
        return instance

    def __init__(self, slc: slice = None):
        self._slice = slc
        self._time_position_file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_lidar_normal.npz"

    def get_np_time_ranges(self) -> np.ndarray:
        with np.load(self._time_position_file_path) as loaded_file:
            time_ranges = loaded_file["data"].copy()

        if self._slice is not None:
            time_ranges = time_ranges[self._slice]

        return time_ranges