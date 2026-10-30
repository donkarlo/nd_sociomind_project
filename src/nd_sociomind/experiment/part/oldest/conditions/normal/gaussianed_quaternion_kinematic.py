from robotic_nd.robot.robot import \
    GaussianedQuaternionKinematic


class GaussianedQuaternionKinematic(GaussianedQuaternionKinematic):
    """
    This is a mind process and the result will be saved in memory priming
    this process is to check if it reduces free energy
    """
    pass

if __name__ == "__main__":
    str_path = "/home/donkarlo/Dropbox/phd/data/experiements/oldest/robots/uav1/structure/mind/memory/explicit/long_term/episodic/normal/gaussianed_quaternion_kinematic/uav1_300k_normal_gaussianed_quaternion_kinematic_modality.pkl"
    gps_position_vecs = GaussianedQuaternionKinematic(str_path)
    position_list = gps_position_vecs.get_time_position_sequence_trace_group()
    print("finished")


