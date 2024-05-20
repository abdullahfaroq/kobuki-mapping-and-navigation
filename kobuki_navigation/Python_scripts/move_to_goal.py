import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import time

class MoveBaseRelative:
    def __init__(self):
        rospy.init_node('move_base_relative_goal_python')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def wait_for_current_pose(self, timeout=5.0):
        start_time = time.time()
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for current pose...")
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                rospy.logerr("Timeout waiting for current pose.")
                return False
        return True

    def get_current_pose(self):
        if self.current_pose is None:
            rospy.logerr("Current pose not yet received.")
            return None
        return self.current_pose

    def move_to_relative_goal(self, relative_x, relative_y, relative_yaw):
        if not self.wait_for_current_pose():
            rospy.logerr("Could not obtain current pose.")
            return

        current_pose = self.current_pose

        current_orientation = current_pose.orientation
        current_position = current_pose.position

        _, _, current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        new_yaw = current_yaw + relative_yaw
        new_orientation_quat = quaternion_from_euler(0, 0, new_yaw)

        new_x = current_position.x + relative_x
        new_y = current_position.y + relative_y

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(new_x, new_y, 0)
        goal.target_pose.pose.orientation = Quaternion(*new_orientation_quat)

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
        else:
            return self.client.get_result()

if __name__ == '__main__':
    try:
        mover = MoveBaseRelative()
        # Adjust the values below as per your requirement
        mover.move_to_relative_goal(-1.0, 0.5, math.radians(90))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
