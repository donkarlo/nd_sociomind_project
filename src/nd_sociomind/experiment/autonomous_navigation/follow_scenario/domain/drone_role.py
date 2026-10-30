from enum import Enum


class DroneRole(Enum):
    LEADER = "leader"
    FOLLOWER = "follower"

    def other(self) -> "DroneRole":
        if self == DroneRole.LEADER:
            return DroneRole.FOLLOWER
        return DroneRole.LEADER
