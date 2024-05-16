"""
    Jason Hughes
    April 2024
    ROS-Node to interface with xbee radio via serial
    and change flight mode to AUTO.LOITER.
"""

import serial
import rospy
from mavros_msgs.srv import SetMode, SetModeRequest

NORMAL = "7e000b8801495300010002000000d7"
ESTOP  = "7e000b8801495300010002000002d5"
QUERY = "7e0004080149535a"
RESET = "7e000508014431047d"
RESET_SUCCESS = "7e0005880144310001"

class EStopNode:

    def __init__(self, dev, baud):

        self.conn_ = self.connect(dev, baud)
        self.service_id_ = "/mavros/set_mode"

        self.radio_check()
        self.mode_client_ = self.service_init()

        self.set_mode_ = SetModeRequest()
        self.set_mode_.custom_mode = "AUTO.LOITER"

    def service_init(self):
        """
        Initialize the connection to the ROS service to 
        change flight modes.
        """
        rospy.loginfo("[ESTOP] Waiting for service id %s" %self.service_id_)
        rospy.wait_for_service( self.service_id_ )
        rospy.loginfo("[ESTOP] Found service %s" %self.service_id_)
    
        return rospy.ServiceProxy(self.service_id_, SetMode)


    def connect(self, dev, baud):
        """
        function to open a serial connection to xbee reciever
        """
        try:
            connection = serial.Serial(dev, baud)
            rospy.loginfo("[ESTOP] connected to radio at %s with baudrate %s" %(dev, baud))
        except Exception as e:
            rospy.logerr("[ESTOP] couldn't connect to radio at %s with baudrate %s, %s" %(dev, baud, e))
            exit()

        return connection

    def radio_check(self):
        """
        Check to see if the radio is configured properly 
        on start-up, if its not reset it.
        """
        res = self.conn_.write(bytes.fromhex(QUERY))
        raw_bytes = self.conn_.read(15)

        if raw_bytes.hex() != NORMAL:
            self.reset()

    def reset(self):
        """
        reset the radio according to DARPA guidelines
        """
        res = self.conn_.write(bytes.fromhex(RESET))
        raw_bytes = self.conn_.read(9)

        if raw_bytes.hex == RESET_SUCCESS:
            rospy.loginfo("[ESTOP] reset successful")
        else:
            rospy.logerr("[ESTOP] unable to reset xbee radio")
            exit()

    def loop(self):

        while (not rospy.is_shutdown()):
            res = self.conn_.write(bytes.fromhex(QUERY))
            raw_bytes = self.conn_.read(15)

            if raw_bytes.hex() != NORMAL:
                rospy.loginfo("[ESTOP] E-stop detected, changing mode to AUTO.LOITER")
                if (self.mode_client_.call(self.set_mode_).mode_sent == True):
                    rospy.loginfo("[ESTOP] AUTO.LOITER mode set")
                else:
                    rospy.loginfo("[ESTOP] unable to change flight mode, retrying")
                    continue


if __name__ == "__main__":
    rospy.init_node("xbee_reciever_mavros")

    baud = 9600
    dev  = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30DN9UC-if00-port0"
    #dev = "/dev/ttyUSB0"
    nh = EStopNode(dev, baud)
    nh.loop()
