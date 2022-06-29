import rclpy
import numpy as np
from rclpy.node import Node
from canfd_msgs.msg import OpenCyphalMessage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
""" 
Conversion of a uORB pca_pwm message in PX4 to OpenCyphal single frame message. 
Tested on Ubuntu 22.04 for virtual CAN use. Physical CANFD tested on a NavQPlus 
connected over CAN to an NXP UCANS32K146 connected over i2c to an NXP PCA9685 board.
uORB message definition of pca_pwm:
    uint64 timestamp        # time since system start (microseconds)
    uint16 pwm_period       # PWM period in microseconds (us) [roof((1s/1526hz)*10^7us/s)=656us:floor((1s/24hz)*10^7us/s)=41666us]
    uint16[16] pulse_width  # pulse width in microseconds (us) [656:41666]
"""

class ROS2CyphalMessagePublisherTest(Node):

    def __init__(self):
        super().__init__('ros2_cyphal_message_pca_publisher_test')
        self.TestPeriod = 20000
        self.OnesArray = np.ones((16,), dtype=np.uint16)
        self.InitTime = int(round(self.get_clock().now().nanoseconds/1000.0))
        self.CounterCyphalMsg = 0
        self.PubCyphal = self.create_publisher(OpenCyphalMessage, 'CyphalTransmitFrame', 0)
        self.CmdVelPub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.SubCmdVel = self.create_subscription(Twist, '/cmd_vel', self.PublishPCAPWMValues, 10)
        bogusCmdVel = Twist()
        bogusCmdVel.linear.x = 0.0
        bogusCmdVel.angular.z = 0.0
        self.CmdVelPub.publish(bogusCmdVel)

    def PublishPCAPWMValues(self, ReceivedMsg):
        VelocitySetpointUS = np.uint16((ReceivedMsg.linear.x * 57) + 1500)
        TurnSetpointUS = np.uint16((ReceivedMsg.angular.z * 150) + 1500)
        if(VelocitySetpointUS > 1590):
            VelocitySetpointUS = 1590
        if(VelocitySetpointUS < 1410):
            VelocitySetpointUS = 1410
        if(TurnSetpointUS > 1650):
            TurnSetpointUS = 1650
        if(TurnSetpointUS < 1350):
            TurnSetpointUS = 1350
        TestPulseWidthArray = self.OnesArray*np.uint16(1500)
        TestPulseWidthArray[0] = VelocitySetpointUS
        TestPulseWidthArray[1] = TurnSetpointUS
        TestPWMPeriod=self.TestPeriod
        msg = OpenCyphalMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.priority = int(4)
        msg.is_annonymous = False
        msg.subject_id = int(500)
        msg.source_node_id = int(96)
        msg.data = self.ConvertDataPCAPWM(TestPWMPeriod,TestPulseWidthArray)
        msg.crc= int(224+(self.CounterCyphalMsg%32))
        self.PubCyphal.publish(msg)
        self.CounterCyphalMsg += 1
    

    def ConvertDataPCAPWM(self, PWMPeriod, PulseWidthArray):
        DataArray=np.array([], dtype=np.uint8)
        if PWMPeriod > 41666:
            PWMPeriod = 41666
        elif PWMPeriod < 656:
            PWMPeriod = 656
        TimeSinceInit = int(round(self.get_clock().now().nanoseconds/1000.0))-self.InitTime
        for i in range(8):
            DataArray = np.append(DataArray,
                    [np.uint8((TimeSinceInit >> i*8) & 255)], 
                    axis=0)
        DataArray = np.append(DataArray,
                            [np.uint8(PWMPeriod & 255),
                             np.uint8(PWMPeriod >> 8)], axis=0)
        if len(PulseWidthArray) <= 16:
            for PulseWidthVal in PulseWidthArray:
                DataArray = np.append(DataArray, 
                               [np.uint8(PulseWidthVal & 255),
                                np.uint8(PulseWidthVal >> 8)], axis=0)
        while len(DataArray) < 63:
            DataArray = np.append(DataArray, 
                               [np.uint8(0)], axis=0)
        return DataArray
                
            
if __name__ == '__main__':
    rclpy.init()
    R2CMPT = ROS2CyphalMessagePublisherTest()
    rclpy.spin(R2CMPT)
    R2CMPT.destroy_node()
    rclpy.shutdown()
