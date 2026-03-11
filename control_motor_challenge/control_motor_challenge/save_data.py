import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv

class MotorLogger(Node):
    def __init__(self):
        super().__init__('save_data')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'motor_output',
            self.callback,
            10)

        self.file = open('motor_data.csv','w',newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["time","setpoint","control","velocity"])

    def callback(self,msg):
        t = msg.data[0]
        sp = msg.data[1]
        u = msg.data[2]
        vel = msg.data[3]
        self.writer.writerow([t,sp,u,vel])

def main():
    rclpy.init()
    node = MotorLogger()
    rclpy.spin(node)
main()
