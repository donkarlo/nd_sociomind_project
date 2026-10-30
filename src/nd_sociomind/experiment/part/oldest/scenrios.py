from functools import cache
from robotic_nd.structure.mind.process.kind.memory.stack.layer.layer import Layer as ExperienceLevel
from robotic_nd.structure.mind.process.kind.memory.stack.stack import Stack as ExperienceLevelStack
from robotic_nd.robot.robot import Goal
from nd_sociomind_project.experiment.kind.oldest.goal.synced_turning_arround_corridor import SyncedTurningAroundCorridor
from robotic_nd.robot.robot import Composite
from typing import Tuple
from robotic_nd.robot.robot import TarotT650Oldest
from nd_utility.data.storage.factory.uni_kinded_multi_valued_yaml_file import UniKindedMultiValuedYamlFile
from nd_utility.data.storage.factory.single_yaml_file import SingleYamlFile
from robotic_nd.robot.robot import Experience
from robotic_nd.robot.robot import Group as ExperienceGroup
from nd_physics.quantity.kind.dynamic.kinematic.pose.position.position import Position
from robotic_nd.robot.robot import PositionToleranceCriterion
from robotic_nd.robot.robot import Factory as ActionCollectionGenerator
from robotic_nd.robot.robot import Goal
from nd_physics.dimension.unit.unit import Unit





class Scenrios:
    @staticmethod
    @cache
    def get_distance_unit():
        scenario_configes = Scenrios.get_scnario_configs()
        unit_name = scenario_configes["unit"]["distance"]
        unit = Unit(unit_name)
        return unit

    @staticmethod
    @cache
    def get_distance_time_unit():
        scenario_configes = Scenrios.get_scnario_configs()
        unit_name = scenario_configes["unit"]["zeit"]
        unit = Unit(unit_name)
        return unit


    @staticmethod
    @cache
    def get_robots() -> Tuple[TarotT650Oldest, TarotT650Oldest]:
        """
        Create the two UAVs only once and reuse them forever.
        """
        scenario_configs_dict = Scenrios.get_scnario_configs()


        # uav1 or leader experience
        uav1_name = "uav1"
        uav1_mission = Scenrios.get_mission(uav1_name)
        uav1_normal_plan = Scenrios.get_scnario_plan(uav1_name, "normal")
        uav1_normal_memeory_level_stack = Scenrios.get_normal_experience_memory_level_stack(uav1_name, "normal")
        uav1_normal_experience = Experience(uav1_mission, uav1_normal_plan, uav1_normal_memeory_level_stack, "normal")
        uav1_experience_group = ExperienceGroup([uav1_normal_experience])
        uav1 = TarotT650Oldest(uav1_experience_group, uav1_name)

        # uav2 or follower experience
        uav2_name = "uav1"
        uav2_mission = Scenrios.get_mission(uav2_name)
        uav2_normal_plan = Scenrios.get_scnario_plan(uav2_name, "normal")
        uav2_normal_memeory_level_stack = Scenrios.get_normal_experience_memory_level_stack(uav2_name, "normal")
        uav2_normal_experience = Experience(uav2_mission, uav2_normal_plan, uav2_normal_memeory_level_stack, "normal")
        uav2_experience_group = ExperienceGroup([uav2_normal_experience])

        uav2 = TarotT650Oldest(uav2_experience_group, uav2_name)

        return (uav1, uav2)

    @staticmethod
    @cache
    def get_scnario_configs()->dict:
        scenarios_configs = SingleYamlFile("../../configs/scenarios.schema.yaml")
        scenario_configs_dict = scenarios_configs.get_ram_dict()
        return scenario_configs_dict

    @staticmethod
    @cache
    def get_mission(robot_name:str)->Goal:
        scenario_configs_dict = Scenrios.get_scnario_configs()
        goal_states = scenario_configs_dict["initial_mission"]["goals"]["robots"][robot_name]["states"]

        mission = SyncedTurningAroundCorridor(goal_states)

        return mission

    @staticmethod
    @cache
    def get_scnario_plan(robot_name: str, scanario_name:str)->Composite:
        scenario_configs_dict = Scenrios.get_scnario_configs()
        uav_plan = scenario_configs_dict["structured_group"][scanario_name]["robots"][robot_name]["pre_plan"]
        uav_plan_states_file = uav_plan["actions"]["goals"]["states"]["file"]
        vec_sep = uav_plan_states_file["vector_sep"]
        component_sep = uav_plan_states_file["vector_components_sep"]
        uav_status_plan_file_path = uav_plan_states_file["str_path"]
        action_name = uav_plan["actions"]["_kind"]

        
        if action_name == "goto":
            from robotic_nd.robot.robot import GoTo
            sample_goal_state = Position(0,0,0)
            ptc_val = uav_plan["actions"]["goals"]["acceptance_criterion"]["tolerance"]
            sample_goal_acceptance = PositionToleranceCriterion(ptc_val)
            sample_goal = Goal(sample_goal_state, sample_goal_acceptance)
            sample_action = GoTo(sample_goal)
            action_collection = ActionCollectionGenerator.from_vecs_file_path(sample_action, uav_status_plan_file_path, vec_sep, component_sep)
        return Composite(action_collection)

    @staticmethod
    @cache
    def get_normal_experience_memory_level_stack(robot_name:str, scenario_name:str)-> ExperienceLevelStack:
        scnario_configs = Scenrios.get_scnario_configs()

        rosbag_shared_dir = scnario_configs["ros"]["bag"]["dir"]
        normal_scenario_ros_bag = scnario_configs["structured_group"][scenario_name]["ros"]["bag"]
        normal_scenario_sor_bag_dir = normal_scenario_ros_bag["dir"]
        rosbag_shared_file_name = normal_scenario_ros_bag["robots"][robot_name]["file"]["_kind"]

        noramal_file_storage_path = rosbag_shared_dir+ normal_scenario_sor_bag_dir + rosbag_shared_file_name
        file = UniKindedMultiValuedYamlFile(noramal_file_storage_path)

        level_0 = ExperienceLevel(file)
        level_stack = ExperienceLevelStack([level_0])


        return level_stack




