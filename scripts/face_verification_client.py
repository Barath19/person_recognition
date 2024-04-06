#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from person_recognition.srv import FaceVerification, FaceVerificationRequest

def image_callback(msg):
    # Call the face verification service
    rospy.wait_for_service("face_verification")
    face_verification = rospy.ServiceProxy("face_verification", FaceVerification)

    # Choose the operation ("add" or "verify")
    operation = "verify"
    name = "barath"  # Replace with the actual name
    favorite_drink = "tea"  # Replace with the favorite drink

    try:
        response = face_verification(msg, operation, name, favorite_drink)
        if response.success:
            rospy.loginfo(response.message)
        else:
            rospy.logwarn(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
    
    rospy.signal_shutdown("Image processed, exiting...")

def face_verification_client():
    rospy.init_node("face_verification_client")
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    face_verification_client()