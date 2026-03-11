# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')
        
        self.declare_parameter('signal_type', 'sine')
        self.signal_type = self.get_parameter('signal_type').value
        
        # Retrieve sine wave parameters
        self.amplitude = 1.0
        self.omega  = 1.0
        self.timer_period = 0.01 #seconds

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)    ## CHECK FOR THE NAME OF THE TOPIC
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("SetPoint Node Started \U0001F680")
        

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        if self.signal_type == 'sine':
            signal = self.amplitude * np.sin(self.omega * elapsed_time)
            
        elif self.signal_type == 'square':
            signal = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        
        elif self.signal_type == 'triangle':
            signal = (2 * self.amplitude / np.pi) * np.arcsin(np.sin(self.omega * elapsed_time))
            
        elif self.signal_type == 'step':
            if elapsed_time >= 2.0:
                signal = self.amplitude
            else:
                signal = 0.0
        
        else:
            self.get_logger().warn("Invalid signal_type. Using 0.0")
            signal = 0.0
        # Generate the signal
        self.signal_msg.data = float(signal)
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
    
    #Parameter callback
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'signal_type':
                self.signal_type = param.value
        return SetParametersResult(successful=True)
#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
