import rospy
from geometry_msgs.msg import Twist
import can
import struct
import time

class CANVelocityController:
    def __init__(self):
        self.node_id_1 = 0
        self.node_id_2 = 1
        self.bus = can.interface.Bus("can0", bustype="socketcan")

        self.enter_closed_loop_control(self.node_id_1)
        self.enter_closed_loop_control(self.node_id_2)
        
        # Wait for a short duration to ensure the motors have entered closed loop control
        time.sleep(1)

        # Subscribe to cmd_vel topic
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback)

    def enter_closed_loop_control(self, node_id):
        self.bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
            data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
        ))

    def set_motor_velocity(self, node_id, velocity):
        self.bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d),  # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', velocity, 0.0),  # velocity: velocity setpoint, 0.0: torque feedforward
            is_extended_id=False
        ))

    def twist_callback(self, msg):
        forward_velocity = msg.linear.x
        turning_velocity = msg.angular.z

        velocity_motor_1 = forward_velocity + turning_velocity
        velocity_motor_2 = forward_velocity - turning_velocity

        self.set_motor_velocity(self.node_id_1, velocity_motor_1)
        self.set_motor_velocity(self.node_id_2, velocity_motor_2)

def main():
    rospy.init_node('can_vel_controller')
    controller = CANVelocityController()
    rospy.spin()

if __name__ == '__main__':
    main()