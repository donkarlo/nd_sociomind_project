from robotic_nd.robot.robot import Segregatored
from robotic_nd.robot.robot import FromNameDictionary as CompositeMemoryDic
from robotic_nd.robot.robot import \
    RosBagYamlMessageSegragator
from robotic_nd.robot.robot import Structure
from robotic_nd.robot.robot import RosMultiModalYamlMessages

from nd_robotic_group_project.group import Group as RobotGroup
from nd_robotic_group_project.experiment.experiment import Experiment
from robotic_nd.robot.robot import Composite as CompositeMemory
from robotic_nd.robot.robot import Storaged as StoragedTraceGroup
from robotic_nd.robot.robot import Group as TraceGroup
from robotic_nd.robot.robot import Group as PotentialFilledTraceKinds
from nd_utility.data.storage.factory.uni_kinded_multi_valued_yaml_file import UniKindedMultiValuedYamlFile
from nd_utility.data.kind.dic.dic import Dic
from nd_utility.os.file_system.path.path import Path


class Oldest(Experiment):
    """
    """

    def __init__(self):
        print("Oldest experiment is initializing ...")


        # TODO: All this classification must be moved to schema.yaml configuration file

        # loading robotic_group - this classification is just like enliven or waking up

        # TODO: call robotic_group mind set_mode

        # slices to load
        slc = slice(1, 300000)

        path_sep = Path.get_os_path_separator()
        #sample_dir_path = "/home/donkarlo/Dropbox/phd/robotic_group/experiements/oldest/robots/uav1/grouping/mind/memory/explicit/long_term/episodic/experiences/normal"
        shared_path = "/data/experiements/oldest/robots/"
        structure_tree = Structure().get_structure()

        #this path goes through mind ... memory
        shared_path_to_expisode = structure_tree.get_unique_shortest_path_from_top_to_bottom("grouping", "episodic", path_sep)+path_sep

        # each file is named all_modalities.schema.yaml
        all_mixed_memory_traces_for_single_episode = "mixed_traces.schema.yaml"

        robots_props = [Dic({"title": "uav1"})]
        # robots_props.append(Dic({"title": "uav2"}))

        #episodic here is the experience title
        episode_props = [Dic({"title": "normal"})]
        # experience_props.append(Dic({"title": "follow"}))
        # experience_props.append(Dic({"title": "next_to"}))

        modality_props = [Dic({"title": PotentialFilledTraceKinds.gaussianed_quaternion_kinematic.title}),
                          Dic({"title": PotentialFilledTraceKinds.lidar_scan_ranges.title})]


        # builder the robots and loading their mind
        # TODO: consider mental state
        for robot_prop in robots_props:
            robot_name = robot_prop["title"]

            # Building memory
            composit_memmory_tree = CompositeMemoryDic(structure_tree)

            ## we have ros files for each experience independently so we send None here
            robot_episode_composit_mem_root = composit_memmory_tree.get_root_composite()

            robot_shared_episode_path = shared_path + robot_name + path_sep + shared_path_to_expisode

            #Go through the episodic of each robotic_group
            for episode_prop in episode_props:
                episode_name = episode_prop["title"]
                robot_episode_dir_path = robot_shared_episode_path+ path_sep + episode_name + path_sep
                robot_episode_all_mod_file_path = robot_episode_dir_path + path_sep + all_mixed_memory_traces_for_single_episode



                uniformatted_multi_valued_yaml_file = UniKindedMultiValuedYamlFile(robot_episode_all_mod_file_path,
                                                                                   slc, False)

                robot_episode_trace_group = StoragedTraceGroup(TraceGroup.init_by_traces_and_kind_and_name(None, RosMultiModalYamlMessages(), all_mixed_memory_traces_for_single_episode), uniformatted_multi_valued_yaml_file)

                #Todo:  is segregation possible? the robotic_group should ask itself and decide base on relatned nodes
                #TODO: the robotic_group should understand if it needs segregation according to anomally level or not?
                #TODO: is resegregation necessary?
                #builder mixed modality which should be segregated to be able to segregate each episodic
                composite_memory_episode = CompositeMemory(robot_episode_trace_group, episode_name)
                robot_segregatble_episode_composit_memory = Segregatored(composite_memory_episode, RosBagYamlMessageSegragator(composite_memory_episode, slc))

                # add normal episodic for example and then follow and next_to
                robot_episode_composit_mem_root.get_child_by_name("episodic").add_child(robot_segregatble_episode_composit_memory)

                # if normal is added, now segregate it
                # TODO: this should be automated. For example the robotic_group should try to find out if segregation helps it with better reducing the suprise or free enrgy
                # Todo: if the the internal_storage memory already exists, this must be stopped
                robot_segregatble_episode_composit_memory.create_segregated_componnets_as_children()

                robot_episode_composit_mem_root.draw()
                print(robot_episode_composit_mem_root.get_tree())
                # Klass.init_from_object(robot_segregatble_episode_composit_memory).draw_inherited_classes()



        # now that we have the mind aqnd memory tree lets builder the the Robot Finite
        # human_group = RobotGroup(Human())
        # robot_group = LeaderFollowers(TarotT650(""), [TarotT650("")])
        # self._composite_robot_group = CompositeRobotGroup()
        # self._composite_robot_group.add_child(robot_group)
        # self._composite_robot_group.add_child(human_group)

        # builder scenarios
        # learning_scenarios = [NormalScenario()]
        # test_scenarios = [FollowScenario(), NextToScenario()]
        # super().__init__(learning_scenarios, test_scenarios)

    def get_robot_group(self) -> RobotGroup:
        return self._composite_robot_group

    def get_robot_mind(self, robot):
        pass

if __name__ == "__main__":
    experiment = Oldest()