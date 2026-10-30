from nd_robotic_ai.robot.kind.uav.kind.multi_copter.quad_copter.quad_copter import QuadCopter


class TarotT650Oldest(QuadCopter):

    def __init__(self):
        QuadCopter.__init__(self)

        # rotor = Rotor()
        # rotor_composite = RotorComposite
        # sensor_set = Train([RpA2Lidar(), GpsOdomSensor()])
        # memory = Working(Memorizing(), Remembering(), experience_group)
        # mind = Mind(memory, Learning())


    def build_from_urdf(self, urdf: str) -> None:
        QuadCopter.build_from_urdf(self, urdf)

