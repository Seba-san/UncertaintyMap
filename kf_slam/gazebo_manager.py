import rospy
from std_srvs.srv import Empty

class GazeboSimulationController:
    def __init__(self):
        #rospy.init_node('gazebo_simulation_controller', anonymous=True)
        self.pause_simulation_service = '/gazebo/pause_physics'
        self.unpause_simulation_service = '/gazebo/unpause_physics'
        rospy.wait_for_service(self.pause_simulation_service)
        rospy.wait_for_service(self.unpause_simulation_service)
        self.pause_sim_srv = rospy.ServiceProxy(self.pause_simulation_service, Empty)
        self.unpause_sim_srv = rospy.ServiceProxy(self.unpause_simulation_service, Empty)

    def pause_simulation(self):
        """
        Pausa el motor fisico  en Gazebo
        """
        rospy.loginfo("Pausing Gazebo simulation...")
        try:
            self.pause_sim_srv()
            rospy.loginfo("Gazebo simulation paused.")
        except rospy.ServiceException as e:
            rospy.logerr("Error al pausar la simulación: %s" % e)

    def unpause_simulation(self):
        """
        Le da play al motor fisico de gazebo
        """
        rospy.loginfo("Unpausing Gazebo simulation...")
        try:
            self.unpause_sim_srv()
            rospy.loginfo("Gazebo simulation unpaused.")
        except rospy.ServiceException as e:
            rospy.logerr("Error al despausar la simulación: %s" % e)

if __name__ == '__main__':
    try:
        controller = GazeboSimulationController()

        # Pausar la simulación
        controller.pause_simulation()
        rospy.sleep(2)  # Puedes ajustar el tiempo que desees pausar la simulación

        # Despausar la simulación
        controller.unpause_simulation()
        rospy.sleep(2)  # Puedes ajustar el tiempo que desees que la simulación se ejecute

    except rospy.ROSInterruptException:
        rospy.logerr("Error al interactuar con ROS.")