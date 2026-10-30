import numpy as np

from nd_math.number.kind.real.interval.unit.open_unit_interval import OpenUnitInterval
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.decorator.storaged.storaged import \
    Storaged
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.part.explicit.part.long_term.part.episodic.part.shared.episode.modality.kind.time_positions import \
    TimePositions
from nd_utility.data.kind.dic.dic import Dic
from nd_utility.data.storage.kind.file.numpi.multi_valued import MultiValued as NpMultiValued
from nd_utility.os.file_system.file.file import File as OsFile
from nd_utility.os.file_system.path.file import File as FilePath


class Uav1300kNormalTimePositionModality(TimePositions, Storaged):
    def __new__(cls, slc: slice = None):
        instance = super().__new__(cls)
        return instance

    def __init__(self, slc: slice = None):
        self._slice = slc

        self._time_position_file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_normal_300k_time_positions_modality.npz"

        file_path = FilePath(self._time_position_file_path)
        os_file = OsFile.init_from_path(file_path)
        storage = NpMultiValued(os_file, False)

        dic = Dic({
            "labels": ["time", "x", "y", "z"],
            "values": None
        })

        TimePositions.__init__(self, dic)
        Storaged.__init__(self, self, storage)

        self._origin_class_name = self.__class__.__name__
        self._np_time_positions = None
        self._periods = None
        self._dic = dic

    def get_np_time_positions(self) -> np.ndarray:
        if self._np_time_positions is None:
            self.load()
            ram = self.get_storage().get_ram()

            if self._slice is not None:
                self._np_time_positions = ram[self._slice]
            else:
                self._np_time_positions = ram

            self._dic.add_key_value("values", self._np_time_positions, True)

        return self._np_time_positions

    def get_np_positions(self) -> np.ndarray:
        return self.get_np_time_positions()[:, 1:]

    def get_time_position_file_path(self) -> str:
        return self._time_position_file_path

    def get_periods(self):
        ...

    def sample_periods(self, rate: OpenUnitInterval):
        ...

    def get_train_test_sample(self, one_period_members_count, rate: OpenUnitInterval):
        ...
