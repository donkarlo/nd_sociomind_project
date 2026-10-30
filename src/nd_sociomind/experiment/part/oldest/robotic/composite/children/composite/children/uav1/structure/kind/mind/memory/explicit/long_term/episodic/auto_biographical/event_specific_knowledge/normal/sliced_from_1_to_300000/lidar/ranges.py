from nd_robotic_ai.robot.structure.kind.mind.cognition.process.kind.memory.composite.composite import \
    Composite as CompositeMemory


class Lidar(CompositeMemory):
    def __init__(self):
        self._lidar_file_path = "/data/oldest/robotic_group/classification/classification/members/uav1/grouping/members/mind/memory/explicit/long_term/episodic/normal/lidar/lidar.pkl"