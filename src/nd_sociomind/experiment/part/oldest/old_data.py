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


class Uav1300kNormalTimePositionModality(TimePositions,
                                         Storaged):
    """ This class returns a Storaged(TimePositions()) """


    def __new__(cls):
        slc = slice(0, 300000)
        time_position_file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_normal_300k_time_positions_modality.npz"
        file_path = FilePath(time_position_file_path)
        os_file = OsFile.init_from_path(file_path)
        storage = NpMultiValued(os_file, False)
        dic = Dic({"labels": ["time", "x", "y", "z"], "values": None})
        time_positions = TimePositions(dic)
        storaged_time_positions = Storaged(time_positions, storage)
        storaged_time_positions._origin_class_name = cls.__name__
        storaged_time_positions._slice = slc
        storaged_time_position_file_path = time_position_file_path
        storaged_time_positions._time_position_file_path = storaged_time_position_file_path
        storaged_time_positions._np_time_positions = None
        storaged_time_positions._periods = None
        storaged_time_positions._dic = dic
        return storaged_time_positions


    def __init__(self, slc: slice):
        pass


    def get_np_time_positions(self) -> np.ndarray:
        if self._np_time_positions is None:
            self.load()
        ram = self.get_storage().get_ram()
        if self._slice is not None:
            self._np_time_positions = ram[self._slice]
        else:
            self._np_time_positions = ram
        self._dic["values"] = self._np_time_positions
        return self._np_time_positions


    def get_np_positions(self) -> np.ndarray:
        return self.get_np_time_positions()[:, 1:]


    def get_time_position_file_path(self) -> str:
        return self._time_position_file_path


    def get_periods(self): ...


    def sample_periods(self, rate: OpenUnitInterval): ...


    def get_train_test_sample(self, one_period_members_count, rate: OpenUnitInterval):
        # positions_sequence = self._time_positions_composite_memory.get_np_positions() # # usable_len = (len(positions_sequence) // one_period_members_count) * one_period_members_count # ram = positions_sequence[:usable_len] # partition_count = len(ram) // one_period_members_count # # # shape = (partition_count, one_period_members_count, 3) # partitioned_population = ram.reshape(partition_count, one_period_members_count, ram.shape[1]) # # Random # # # print(partitioned_population[0].shape) # training_input_target_pairs = np.vstack( # [partitioned_population[0], partitioned_population[1], partitioned_population[2], partitioned_population[3], # partitioned_population[4], partitioned_population[5], partitioned_population[6], # partitioned_population[7]]) # testing_input_target_pairs = np.vstack([partitioned_population[8], partitioned_population[9]]) pass