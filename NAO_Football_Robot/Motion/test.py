from naoqi import ALProxy

ip = "192.168.1.100"
port = 9559
proxy = ALProxy("ALTextToSpeech", ip, port)
pose_proxy = ALProxy("ALRobotPosture", ip, port)
pose_proxy.goToPosture("Stand", 0.5)
# proxy.say("I wanna be a real boy!")