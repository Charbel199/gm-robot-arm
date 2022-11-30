 #!/usr/bin/env python
import rospy
from rosserial_msgs.msg import ServoPositions
from rosserial_msgs.msg import Moves
from std_msgs.msg import Bool
from logger.log import LoggerService
import time
from utils.move_utils import MOVE_DICT, SAFE_POSE, YEET_POSE

#logger = LoggerService.get_instance()

class ControlCore:
    
    def __init__(self):
        try:
            print(f'Launching Control Core')
            self.pub = rospy.Publisher('/control/arm', ServoPositions, queue_size=10)
            self.move_sub = rospy.Subscriber('/control/move', Moves, self.send_move)
            self.done_sub = rospy.Subscriber('/control/move_done', Bool, self.move_done)
            self.done = False
            self.GRIPPER_CLOSE = 180
            self.GRIPPER_OPEN = 130
            rospy.init_node('controller')
            self.rate = rospy.Rate(10)
            self.go_to_safe_pose()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
   
   #move tuple example: (("a5", "PICK"),("a5", "YEET") ("a1", "PICK"), ("a5", "PLACE"))

    def move_done(self, done):
        self.done = done.data
        print(f"Setting move done to {self.done}")

    def wait_for_move(self):
        while self.done==False:
           pass
        self.rate.sleep()
        self.done = False
        #time.sleep(3)
        pass

    def go_to_safe_pose(self):
        self.pub.publish(self.GRIPPER_OPEN, SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4])
        pass

    def move_sequence(self, control_move):
        for i in range(6):
            if i==0:
                self.pub.publish(self.GRIPPER_CLOSE if control_move[0]==self.GRIPPER_OPEN else self.GRIPPER_OPEN, SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")
            if i==1:
                self.pub.publish(self.GRIPPER_CLOSE if control_move[0]==self.GRIPPER_OPEN else self.GRIPPER_OPEN, SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], control_move[4], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")
            if i==2:
                self.pub.publish(self.GRIPPER_CLOSE if control_move[0]==self.GRIPPER_OPEN else self.GRIPPER_OPEN, SAFE_POSE[0], SAFE_POSE[1], control_move[3], control_move[4], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")
            if i==3:
                self.pub.publish(self.GRIPPER_CLOSE if control_move[0]==self.GRIPPER_OPEN else self.GRIPPER_OPEN, SAFE_POSE[0], control_move[2], control_move[3], control_move[4], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")
            if i==4:
                self.pub.publish(self.GRIPPER_CLOSE if control_move[0]==self.GRIPPER_OPEN else self.GRIPPER_OPEN, control_move[1], control_move[2], control_move[3], control_move[4], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")
            if i==5:
                self.pub.publish(control_move[0], control_move[1], control_move[2], control_move[3], control_move[4], control_move[5])
                self.wait_for_move()
                print(f"Sequence {i} completed")

        for i in range(5):
            # if i==0:
            #     self.pub.publish(control_move[0], control_move[1], control_move[2], control_move[3], control_move[4],SAFE_POSE[4])
            #     self.wait_for_move()
            # if i==1:
            #     self.pub.publish(control_move[0], control_move[1], control_move[2], control_move[3], SAFE_POSE[3], SAFE_POSE[4])
            #     self.wait_for_move()
            # if i==2:
            #     self.pub.publish(control_move[0], control_move[1], control_move[2], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4])
            #     self.wait_for_move()
            # if i==3:
            #     self.pub.publish(control_move[0], control_move[1], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4])
            #     self.wait_for_move()
            # if i==4:
            #     self.pub.publish(control_move[0], SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4])
            #     self.wait_for_move()
            if i==0:
                self.pub.publish(control_move[0], SAFE_POSE[0], control_move[2], control_move[3], control_move[4],control_move[5])
                self.wait_for_move()
            if i==1:
                self.pub.publish(control_move[0], SAFE_POSE[0], SAFE_POSE[1], control_move[3], control_move[4],control_move[5])
                self.wait_for_move()
            if i==2:
                self.pub.publish(control_move[0], SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], control_move[4],control_move[5])
                self.wait_for_move()
            if i==3:
                self.pub.publish(control_move[0],  SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[4],control_move[5])
                self.wait_for_move()
            if i==4:
                self.pub.publish(control_move[0], SAFE_POSE[0], SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4])
                self.wait_for_move()
        current_counter = rospy.get_param('/control/move_complete_counter')
        rospy.set_param('/control/move_complete_counter', current_counter-1)

    def send_move(self, move):
        print(f"RECEIVED SEND MOVE {move.type} TO {move.square}")
        move_type = move.type
        if move_type == 'YEET':
            control_move = YEET_POSE
        else:
            control_move = MOVE_DICT[move.square].copy()
            print(f"BEFORE INSERT {control_move}")
        if move_type == 'PICK':
            control_move.insert(0,self.GRIPPER_CLOSE)
        else:
            control_move.insert(0,self.GRIPPER_OPEN)
        
        print(f"AFTER INSERT Control move: {control_move}")
        self.move_sequence(control_move)           
        print(f"Executed move.")
        pass

    

if __name__ == "__main__":
    controller = ControlCore()
