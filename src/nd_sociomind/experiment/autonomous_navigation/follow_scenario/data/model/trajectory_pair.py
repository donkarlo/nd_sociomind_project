from dataclasses import dataclass

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory


@dataclass
class TrajectoryPair:
    leader: Trajectory
    follower: Trajectory
    # Complete recorded follower loop for visualization, when available.
    # Playback continues to use the synchronized leader/follower samples.
    follower_full_route: Trajectory | None = None
