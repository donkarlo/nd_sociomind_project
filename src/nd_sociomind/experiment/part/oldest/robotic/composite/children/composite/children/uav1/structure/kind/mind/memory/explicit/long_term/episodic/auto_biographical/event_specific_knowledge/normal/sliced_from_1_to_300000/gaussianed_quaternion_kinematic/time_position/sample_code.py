from nd_math.probability.statistic.population.sampling.sampler.kind.countable.finite.members_mentioned.numbered.sequence.sliding_window.generator import \
    Generator as SlidingWindowGenerator
from nd_math.probability.statistic.population.sampling.sampler.kind.countable.finite.members_mentioned.numbered.sequence.sliding_window.sliding_window import \
    SlidingWindow
from nd_robotic_ai.robot.structure.kind.mind.cognition.process.kind.memory.composite.action.kind.intra.segregating.segregating import \
    Segregating
from nd_robotic_ai.robot.structure.kind.mind.cognition.process.kind.memory.composite.component import Component
from nd_utility.data.storage.kind.file.numpi.multi_valued import MultiValued as NpMultiValued
from nd_utility.os.file_system.file.file import File as OsFile
from nd_utility.os.file_system.path.file import File as FilePath
import numpy as np
from typing import Tuple


class AutoCorrelating(Segregating):
    """
    To break a periodicity sequence into its periods
    """

    def __init__(self, source_memory_component: Component, period_length: int, with_time: bool):
        self._period_length = 24450

        file_path = FilePath(
            "/home/donkarlo/Dropbox/repo/nd_robotic_lab_project/data/oldest/robotic_group/config/config/members/uav1/grouping/members/mind/memory/explicit/long_term/episodic/normal/gaussianed_quaternion_kinematic/time_position/time_position.npz"
        )
        os_file = OsFile.init_from_path(file_path)
        storage = NpMultiValued(os_file, False)
        storage.load()
        # removing the time_based
        if with_time:
            self._traces = storage.get_ram()[:, :]  # (T, 3)
        else:
            self._traces = storage.get_ram()[:, 1:]

    def _build_segregated_components(self, sliding_window: SlidingWindow) -> Tuple[np.ndarray, np.ndarray]:
        usable_len = (len(self._traces) // self._one_period_members_count) * self._one_period_members_count
        usable_ram = self._traces[:usable_len]

        partition_count = len(usable_ram) // self._one_period_members_count

        # shape = (partition_count, one_period_members_count, 3)
        partitioned_population = usable_ram.reshape(partition_count, self._one_period_members_count,
                                                    usable_ram.shape[1])
        training_partitions = np.vstack(
            [partitioned_population[0], partitioned_population[1], partitioned_population[2], partitioned_population[3],
             partitioned_population[4], partitioned_population[5], partitioned_population[6],
             partitioned_population[7]])
        testing_partitions = np.vstack([partitioned_population[8], partitioned_population[9]])

        training_input_traget_pairs = SlidingWindowGenerator(training_partitions,
                                                             sliding_window).get_input_output_pairs()
        testing_input_traget_pairs = SlidingWindowGenerator(testing_partitions, sliding_window).get_input_output_pairs()

        return (training_input_traget_pairs, testing_input_traget_pairs)