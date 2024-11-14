#!/usr/bin/env python3
import rospy 
import csv
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Vector3


class CollectingDataNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('collect_data_node')
        rospy.loginfo('collect_data_node successfully initialized!')
        
        # Initialize the controls' publisher
        self.steering_gain = rospy.get_param('steering/gain')
        self.steering_offset = rospy.get_param('steering/offset')
        self.throttle_gain = rospy.get_param('throttle/gain')
        
        # Initialize the path of the csv file
        self.csv_file_path = rospy.get_param('csv_file_path')

        # Initialize the subscriber to the Optitrack
        rospy.Subscriber("/vrpn_client_node/steering_car/pose", PoseStamped, self.__callback_vrpn)
        rospy.Subscriber("/orientation/euler", Vector3, self.__callback_orientation)
        rospy.Subscriber("/steering", Float32, self._callback_steering)
        rospy.Subscriber("/throttle", Float32, self._callback_throttle)
        
        # Initialize with zeros the state and control quantities
        self.x_k = 0.0
        self.y_k = 0.0
        self.theta_k = 0.0
        
        self.Delta_k = 0.0
        self.delta_k = 0.0
        
    def __callback_vrpn(self, msg):
        self.x_k = msg.pose.position.x
        self.y_k = -msg.pose.position.z
    
    def __callback_orientation(self, msg):
        self.theta_k = msg.y
        
    def _callback_throttle(self, msg):
        # https://www.waveshare.com/wiki/JetRacer_AI_Kit                        
        # y = a * x 
        throttle = self.throttle_gain * msg.data
        self.Delta_k = (1/100) * throttle
    
    def _callback_steering(self, msg):
        # https://www.waveshare.com/wiki/JetRacer_AI_Kit 
        # y = a * x + b -> x = (y - b) / a
        self.delta_k = self.steering_gain * msg.data + self.steering_offset
        
    def run(self):
                
        # Main loop of the node
        rate_Hz = 100 # Hz
        rate = rospy.Rate(rate_Hz)
        
        # Initialize the CSV with headers or reset at startup
        with open(self.csv_file_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            # Write header row
            csv_writer.writerow(["x_k", "y_k", "theta_k", "Delta_k", "delta_k"])
        
        print(self.csv_file_path)
        
        while (not rospy.is_shutdown()):
            
            kth_row = [self.x_k, self.y_k, self.theta_k, self.Delta_k, self.delta_k]
            
            # Write the current data to the CSV file
            with open(self.csv_file_path, mode='a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(kth_row)  # Write the row to the CSV
            
            # Sleep for the next time instant
            rate.sleep()


if __name__ == "__main__":
    collectData_main_node = CollectingDataNode()
    collectData_main_node.run()