import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import tkinter as tk
import math

DESTINATIONS = {"AR_ROBOTICS": (-1, 0, 0), "SMART HELMET": (-2, 0, 90), "5 DOF ARM": (-3, 0, 180),
                "HEALTH CARE": (0, 0, 0), "VIBRATIONAL ANALYSIS": (0, 0, 180), "ROBOT FAULT ": (0, 0, 270)}

def move_to_goal(x_goal, y_goal, orientation):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not ac.wait_for_server(rospy.Duration.from_sec(5)):
        rospy.loginfo("Failed to connect to move_base server")
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.z = math.sin(math.radians(orientation)/2)
    goal.target_pose.pose.orientation.w = math.cos(math.radians(orientation)/2)

    rospy.loginfo("Sending goal location: (%f, %f) with orientation: %f", x_goal, y_goal, orientation)
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))

    if ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("Failed to reach the destination")
        return False

def navigate_to_destination(destination):
    if destination in DESTINATIONS:
        x_goal, y_goal, orientation = DESTINATIONS[destination]
        if move_to_goal(x_goal, y_goal, orientation):
            label_status.config(text=f"Navigation to {destination} successful", fg="#009688")
        else:
            label_status.config(text=f"Navigation to {destination} failed", fg="#FF5722")
    else:
        rospy.loginfo("Invalid destination")

if __name__ == "__main__":
    rospy.init_node('map_navigation', anonymous=False)

    root = tk.Tk()
    root.title("Robot Navigation")

    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    root.geometry(f"{screen_width}x{screen_height}")

    # Background color
    root.configure(bg="#FAFAFA")

    # Left Frame for A, B, C
    left_frame = tk.Frame(root, bg="#FAFAFA")
    left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

    for destination, coordinates in DESTINATIONS.items():
        if destination in ["AR_ROBOTICS", "SMART HELMET", "5 DOF ARM"]:
            button = tk.Button(left_frame, text=destination, command=lambda dest=destination: navigate_to_destination(dest), width=20, height=3, bg="#B2DFDB", fg="#212121", font=("Arial", 12), relief="raised")
            button.pack(pady=10, fill=tk.X)

    # Right Frame for D, E, F
    right_frame = tk.Frame(root, bg="#FAFAFA")
    right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

    for destination, coordinates in DESTINATIONS.items():
        if destination in ["HEALTH CARE", "VIBRATIONAL ANALYSIS", "ROBOT FAULT "]:
            button = tk.Button(right_frame, text=destination, command=lambda dest=destination: navigate_to_destination(dest), width=20, height=3, bg="#FFCCBC", fg="#212121", font=("Arial", 12), relief="raised")
            button.pack(pady=10, fill=tk.X)

    label_status = tk.Label(root, text="", bg="#FAFAFA", fg="#424242", font=("Arial", 14, "bold"))
    label_status.pack(pady=20)

    root.mainloop()
