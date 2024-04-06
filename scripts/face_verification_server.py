#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from person_recognition.srv import FaceVerification, FaceVerificationResponse
from cv_bridge import CvBridge
import cv2
from deepface import DeepFace
import sqlite3
import numpy as np

# Publisher for the verification images
pub = None

def extract_face_embedding(cv_image):
    try:
        embedding = DeepFace.represent(cv_image, model_name="VGG-Face", enforce_detection=False)
        return embedding
    except ValueError as e:
        rospy.logerr(f"Error: {e}")
    return None

def extract_face_info(cv_image):
    try:
        face_analysis = DeepFace.analyze(img_path=cv_image, actions=['age', 'gender'], enforce_detection=False)
        if face_analysis:
            face_info = face_analysis[0]
            age = face_info['age']
            gender = face_info['dominant_gender']
            return age, gender
    except ValueError as e:
        rospy.logerr(f"Error: {e}")
    return None, None

def handle_face_verification(req):
    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

    if req.operation == "add":
        # Extract face embedding and information
        try:
            embedding = extract_face_embedding(cv_image)
            age, gender = extract_face_info(cv_image)
            if embedding is not None and age is not None and gender is not None:
                name = req.name
                favorite_drink = req.favorite_drink
                embedding_array = np.array(embedding)
                image_data = cv2.imencode('.jpg', cv_image)[1].tobytes()  # Convert image to JPEG format
                c.execute("INSERT INTO face_embeddings (embedding, name, image, favorite_drink, age, gender) VALUES (?, ?, ?, ?, ?, ?)", (embedding_array.tobytes(), name, image_data, favorite_drink, age, gender))
                conn.commit()
                return FaceVerificationResponse(True, f"Face embedding, image, favorite drink, age, and gender added for {name}")
            else:
                return FaceVerificationResponse(False, "Failed to extract face information")
        except ValueError as e:
            return FaceVerificationResponse(False, str(e))

    elif req.operation == "verify":
        # Publish the image for verification
        pub.publish(req.image)

        # Perform face verification
        try:
            c.execute("SELECT image, name, favorite_drink, age, gender FROM face_embeddings")
            results = c.fetchall()
            for row in results:
                image_data, stored_name, favorite_drink, age, gender = row
                img1 = cv2.imdecode(np.frombuffer(image_data, np.uint8), cv2.IMREAD_COLOR)
                result = DeepFace.verify(img1_path=img1, img2_path=cv_image, distance_metric="cosine")
                if result["verified"]:
                    return FaceVerificationResponse(True, f"Face verified: {stored_name}, Favorite Drink: {favorite_drink}, Age: {age}, Gender: {gender}")
            return FaceVerificationResponse(False, "Face not recognized")
        except ValueError as e:
            return FaceVerificationResponse(False, str(e))

    else:
        return FaceVerificationResponse(False, "Invalid operation")

def face_verification_server():
    rospy.init_node("face_verification_server")
    service = rospy.Service("face_verification", FaceVerification, handle_face_verification)

    # Create a publisher for the verification images
    global pub
    pub = rospy.Publisher("verification_images", Image, queue_size=10)

    rospy.loginfo("Face verification server is running")
    rospy.spin()

if __name__ == "__main__":
    bridge = CvBridge()

    # Connect to the SQLite database and create a table
    conn = sqlite3.connect('face_embeddings.db', check_same_thread=False)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS face_embeddings
                 (id INTEGER PRIMARY KEY AUTOINCREMENT, embedding BLOB, name TEXT, image BLOB, favorite_drink TEXT, age INTEGER, gender TEXT)''')

    face_verification_server()