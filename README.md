# person_recognition

## Get Started

`cd ros_ws/src`

`git clone https://github.com/Barath19/person_recognition.git`

`cd .. && catkin build`

### Start the server

`rosrun person_recognition face_verification_server.py`

![barath](https://github.com/Barath19/person_recognition/blob/main/assets/server.png)

### To add a person
change line 13 `operation = "add"` and provide name and favorite drink
`rosrun person_recognition face_verification_client.py`

![barath](https://github.com/Barath19/person_recognition/blob/main/assets/barath.png)

![barath](https://github.com/Barath19/person_recognition/blob/main/assets/dhoni.png)

This will be saved in the `face_embedding.db`


### To verify the person from database

change line 13 `operation = "verify"` 


![barath](https://github.com/Barath19/person_recognition/blob/main/assets/verified.png)
