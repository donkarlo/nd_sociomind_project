from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.action.kind.intra.segregating.kind.slicing.kind.by_feature.kind.ros_bag_yaml_message_segragator import \
    RosBagYamlMessageSegragating
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.composition.leaf.leaf import \
    Group as TraceGroup
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.decorator.storaged.storaged import \
    Storaged
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.part.explicit.part.long_term.part.episodic.part.shared.episode.episode import \
    Episode
from nd_utility.data.kind.dic.dic import Dic
from nd_utility.data.storage.factory.uni_kinded_multi_valued_yaml_file import \
    UniKindedMultiValuedYamlFile as UniKindedMultiValuedYamlFileStorage


# it might be an episodic
class Uav1300kNormalEpisode(Episode):
    def __init__(self):
        Episode.__init__(self)
        self._file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_normal_episode.yaml"

        slc = slice(0, 300000)

        self._storage = UniKindedMultiValuedYamlFileStorage(self._file_path, slc, False)

        trace_group = TraceGroup(Dic({}))
        storaged_trace_group = Storaged(trace_group, self._storage)

        segregating_action = RosBagYamlMessageSegragating(storaged_trace_group, slc)

