 #!/usr/bin/env python
import rospy
from rosserial_msgs.msg import ServoPositions
from rosserial_msgs.msg import Moves
from src.logger.log import LoggerService

#logger = LoggerService.get_instance()

class ControlCore:
   
   #move tuple example: (("a5", "PICK"),("a5", "YEET") ("a1", "PICK"), ("a5", "PLACE"))
    def send_move(self, move):
        print("RECEIVED SEND MOVE")
        control_move = [0, 0, 0, 0, 0, 0]
        self.pub.publish(control_move[0], control_move[1], control_move[2], control_move[3], control_move[4], control_move[5])
        print(f"Moving robot to {move.square}")
        pass

    def __init__(self):
        try:
            print(f'Launching Control Core')
            self.pub = rospy.Publisher('/control/arm', ServoPositions, queue_size=10)
            self.sub = rospy.Subscriber('/control_core/move', Moves, self.send_move)
            rospy.init_node('controller')
            self.rate = rospy.Rate(10)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller = ControlCore()
