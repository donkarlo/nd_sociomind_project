from nd_robotic_ai.robot.kind.human.human import Human
from nd_robotic_ai.robot.kind.environment.environment import Environment
from nd_robotic_ai.robot.part.mind.part.cognition.part.object_level.process.part.memory.part.explicit.part.long_term.part.episodic.part.autobiographic.part.event_specific_knowledge.event_specific_knowledge import \
    EventSpecificKnowledge
from nd_robotic_ai.society.experiment.experiment import Experiment
from nd_robotic_ai.society.relationship.kind.containership import Containership
from nd_robotic_ai.society.relationship.kind.leadership import Leadership
from nd_robotic_ai.society.relationship.kind.tele_controlship import TeleControlship
from nd_robotic_ai.society.society import Group as CompositeRobotSociety
from nd_sociomind.experiment.part.oldest.robotic.composite.children.composite.prototype.tarot_t650_oldest import \
    TarotT650Oldest
from nd_sociomind.experiment.part.oldest.uav1_300k_normal_episode import Uav1300kNormalEpisode
from nd_sociomind.experiment.part.oldest.uav1_300k_normal_time_position_modality import Uav1300kNormalTimePositionModality
from nd_utility.oop.design_pattern.creational.prototype.factory import Factory as PrototypeFactory
from nd_utility.oop.design_pattern.structural.composition.decoration.map_to_directory import MapToDirectory


class Oldest(Experiment):
    def __init__(self):
        """
        environment <-contains-> ( human <-controls-> ( uav1 <-leads-> (uav2,uav3,uav4) ) )
        """
        # Build the robots

        # society
        society = CompositeRobotSociety()
        society.set_name("Society")

        # environment
        environment = Environment()
        environment.set_name("environment")

        #  Human
        human = Human()
        human.set_name("human")

        # creating UAVs
        uav_society = CompositeRobotSociety()
        uav_society.set_name("uav_society")

        tarrot_oldest_prototype_factory = PrototypeFactory(TarotT650Oldest())

        # uav1
        uav1 = tarrot_oldest_prototype_factory.get_clone()
        uav1.set_name("uav1")
        uav1_noarmal_episode = Uav1300kNormalEpisode()
        uav1_noarmal_episode.add_child(Uav1300kNormalTimePositionModality())
        uav1.get_child(EventSpecificKnowledge).add_child(uav1_noarmal_episode)


        #followers composition (uav2 and uav3)
        followers_society = CompositeRobotSociety()
        followers_society.set_name("followers_society")

        uav2 = tarrot_oldest_prototype_factory.get_clone()
        uav2.set_name("uav2")

        uav3 = tarrot_oldest_prototype_factory.get_clone()
        uav3.set_name("uav3")

        followers_society.add_child(uav2)
        followers_society.add_child(uav3)

        uav_society.add_child(uav1)
        uav_society.add_child(followers_society)

        uav_society.add_relation(Leadership(uav1, followers_society))

        # human UAV composition
        human_uav_society = CompositeRobotSociety()
        human_uav_society.set_name("human_uav_society")
        human_uav_society.add_child(human)
        human_uav_society.add_child(uav_society)
        human_uav_society.add_relation(TeleControlship(human, uav_society))

        # environment human uav composition
        society.add_child(environment)
        society.add_child(human_uav_society)
        society.add_relation(Containership(environment, human_uav_society))

        # draw
        society.draw()

        #create directory
        directory_decorator = MapToDirectory(society, "/home/donkarlo/Dropbox/repo/nd_sociomind_project/data/experiment/kind/oldest/robots", "conditions", "npz")
        # directory_decorator.export()
    def run(self) ->None:
        pass

if __name__ == "__main__":
    oldest_experiment = Oldest()
    oldest_experiment.run()