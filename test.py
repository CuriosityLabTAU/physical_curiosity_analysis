from naoqi import ALProxy


robotIP = '192.168.0.100'
port = 9559
motionProxy = ALProxy("ALMotion", robotIP, port)
postureProxy = ALProxy("ALRobotPosture", robotIP, port)

# postureProxy.goToPosture("StandInit", 0.1)
# motionProxy.setStiffnesses("Body", 1.0)
# motionProxy.setCollisionProtectionEnabled('Arms', True)

motionProxy.rest()

poses = pickle.load(open('data_after_analysis', 'rb'))

sections = ['basic', 'learn', 'task1', 'task2', 'task3']

pNames = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
          'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
#
# for key in poses:
#     print key
#     for section in sections:
#         print section
#         pos_n=1
#         for pos in poses[key][section]:
#             print pos_n
#             pTargetAngles = pos.tolist()[1:]
#             print pTargetAngles
#             time.sleep(3)
#             motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, 0.1)
#             pos_n+=1
#








