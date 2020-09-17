import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

class ControllerManager:
    def __init__(self, numOfCamera):
        """ Init essential publisher and subscribers """
        self.num = numOfCamera
        self.yawPubs = [0] * numOfCamera
        self.pitchPubs = [0] * numOfCamera
        for i in range(numOfCamera):
            # Note that ***Pubs[id] corresponds to the i+1 robot (coz there are no robot0 in the simulation)
            topicYaw = "/robot" + str(i + 1) + "/yaw_joint_velocity_controller/command"
            self.yawPubs[i] = rospy.Publisher(topicYaw, Float64, queue_size=1)
            topicPitch = "/robot" + str(i + 1) + "/pitch_joint_velocity_controller/command"
            self.pitchPubs[i] = rospy.Publisher(topicPitch, Float64, queue_size=1)

        self.jointSubs = [0] * numOfCamera
        for i in range(numOfCamera):
            topicName = "/robot" + str(i + 1) + "/joint_states"
            self.jointSubs[i] = rospy.Subscriber(topicName, JointState, self.callback, (i))

        self.jointStates = np.zeros(shape=(numOfCamera,2))

    def callback(self, data, arg):
        cameraID = arg
        self.jointStates[cameraID] = data.position

    def manipulate(self, speeds, CameraID):
        """ send speeds [pitch, yaw] command to camera (specified by CameraID) """
        if len(speeds) != 2:
            rospy.logerr(
                "The length of speeds is not 2 (yaw and pitch repectively) in ControllerManager's manipulate function.")
        else:
            self.yawPubs[CameraID].publish(speeds[1])
            self.pitchPubs[CameraID].publish(speeds[0])

    def multimanipulate(self, speeds):
        """send speeds command to multi cameras"""
        length = len(speeds)
        for i in range(length):
            self.manipulate(speeds[i], i)

    def getState(self, cameraID):
        """ get pitch,yaw of camera """
        state = self.jointStates[cameraID]
        return state

    def getImageState(self, cameraID):
        sum_prob = []
        return sum_prob


def stateSet(ctrlManager,id, state):
    """ Test the manipulate function: rotate four cameras in turn. 
    NOTE that this function is only used for test"""
    state_m = ctrlManager.getState(id)
    rate = rospy.Rate(100)
    k = 5
    tor=0.0001
    while not (np.linalg.norm((state_m-state)) <= tor or rospy.is_shutdown()):
        try:
            state_m = ctrlManager.getState(id)
            speed = np.array([ k*(state[0]-state_m[0]), k*(state[1]-state_m[1])])
            ctrlManager.manipulate(speed, id)
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")
    ctrlManager.manipulate(np.zeros(shape=(2,1)), id)

def stateReset(ctrlManager):
    """ Function that reset all the joint to initial pose """
    id =0
    print("reset ", id + 1, "th robot")
    stateSet(ctrlManager, id, state=np.array([1, 1 * (np.pi / 2)]))
    id = 1
    print("reset ", id + 1, "th robot")
    stateSet(ctrlManager, id, state=np.array([1, 3 * (np.pi / 2)]))
    id = 2
    print("reset ", id + 1, "th robot")
    stateSet(ctrlManager, id, state=np.array([1, 2 * (np.pi / 2)]))
    id = 3
    print("reset ", id + 1, "th robot")
    stateSet(ctrlManager, id, state=np.array([1, 0 * (np.pi / 2)]))



if __name__ == "__main__":
    rospy.init_node("camera_controller")
    numOfCamera = 4
    ctrlManager = ControllerManager(numOfCamera)
    rate = rospy.Rate(1)

    print("Test start")
    # Sleep 100 ms. Because ControllerManager need some time to use callback to get the correct states of joints.
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            # To test either manipulate function or getState function,
            # please comment the other

            # Test stateControl()
            # stateReset(ctrlManager)

            # Test getState function
            id = 1
            print("Getting ", id, "State")
            ctrlManager.manipulate([0.2, -0.2], id)
            state = ctrlManager.getState(id)
            print(state)
            rate.sleep()

        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("ROS Interrupt Exception, trying to shut down node")