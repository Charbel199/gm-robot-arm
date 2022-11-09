import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
counter = 0


def callback(data):
    global counter
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Azure Kinect image", cv_image)

    if cv2.waitKey(33) == ord('p'):
        cv2.imwrite(f"./move{counter}.jpg", cv_image)
        print(f"Saved ./move{counter}.jpg")
        counter += 1

    if cv2.waitKey(33) == ord('q'):
        rospy.signal_shutdown('Manual Shutdow')


def listener():
    rospy.init_node('image_listener')
    rospy.Subscriber("/rgb/image_raw", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
