from nd_physics.world.obstacle import Obstacle
from nd_robotic_group_project.experiment.scenario import Scenario as MrsScenario
from nd_sociomind_project.experiment.kind.oldest.world.hollow_nested_cubes import HollowNestedCubes

from nd_robotic_ai.experiment.kind.core.kind import Kind

from nd_sociomind.experiment.kind.oldest.oldest_old import Oldest
from nd_utility.data.storage.storage import Storage


class Normal(Oldest):
    def __init__(self):
        self.__name = Scenrios.get_scnario_configs()["structured_group"]["normal"]["_kind"]
        self._experience_name = self.__name
        world = HollowNestedCubes([Obstacle()])

        self._robots = Scenrios.get_robots()

        # pose points from normal scenario points
        self._uav1 = self._robots[0]
        self._uav1_plan = Scenrios.get_scnario_plan(self._uav1.get_name(), self.__name)
        self._uav1_mission = Scenrios.get_mission(self._uav1.get_name())

        self._uav2 = self._robots[1]
        self._uav2_plan = Scenrios.get_scnario_plan(self._uav2.get_name(), self.__name)
        self._uav2_mission = Scenrios.get_mission(self._uav2.get_name())

        robots_missions_plans = []
        robots_missions_plans.append([self._uav1, self._uav1_mission, self._uav1_plan])
        robots_missions_plans.append([self._uav2, self._uav2_mission, self._uav2_plan])



        # for the moment we use single robotic_group scenario since follower is totally dependent on the leader during running
        super().__init__(robots_missions_plans, world, self.__name)


    def remember(self)->None:
        uav1_lowest_normal_experience_level_storage:Storage = self._uav1.remember_experience_by_name(self._experience_name).get_lowest_layer().get_trace_group()

        trace_group = TraceGroupLevel(self._uav1_mission, self._uav1_plan)
        uav1_lowest_normal_experience_level_storage.attach_add_value_observer(trace_group)

        slc = slice(1,100)
        ram = uav1_lowest_normal_experience_level_storage.get_values_by_slice(slc)

        traces = trace_group._set_traces()
        print(ram)