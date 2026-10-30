from nd_math.linear_algebra.tensor.vector.vector import Vector
from nd_physics.quantity.kind.dynamic.kinematic.pose.kind.quaternion_pose import QuaternionPose
from nd_physics.world.obstacle import Obstacle
from experiment.scenario import Scenario
from robotic_nd.robot.robot import Group
from robotic_nd.robot.robot import GoTo
from nd_sociomind_project.experiment.kind.oldest.world.hollow_nested_cubes_narrowed_on_a_side import HollowNestedCubesNarrowedOnASide
from nd_sociomind_project.experiment.kind.oldest.experience.plan.next_to import NextTo as NextToPlan
from nd_sociomind_project.experiment.kind.oldest.goal.synced_turning_arround_corridor import SyncedTurningAroundCorridor
from nd_sociomind_project.experiment.kind.oldest.scenrios import Scenrios


class NextTo(Scenario):
    def __init__(self):
        self._world = HollowNestedCubesNarrowedOnASide([Obstacle()])
        self._plan = NextToPlan(Group([GoTo(QuaternionPose(Vector([1, 2, 3]), Vector([1, 2, 3, 4])))]))  # pose points from normal scenario points
        self._mission = SyncedTurningAroundCorridor()

        self._robots = Scenrios.get_robots()

        super().__init__(self._robots, self._mission, self._world, self._plan, "next_to")

    def learn(self) -> None:
        print("I am in NextTo scenario Learning")