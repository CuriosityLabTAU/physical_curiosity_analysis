from naoqi import ALProxy
import pickle
import time

#data:
poses = pickle.load(open('data_after_analysis_17', 'rb'))
poses_16 = pickle.load(open('poses_16', 'rb'))


#conect to robot:

# robotIP = '192.168.0.100'
# port = 9559
# motionProxy = ALProxy("ALMotion", robotIP, port)
# postureProxy = ALProxy("ALRobotPosture", robotIP, port)
#
# postureProxy.goToPosture("StandInit", 0.1)
# motionProxy.setStiffnesses("Body", 1.0)
# motionProxy.setCollisionProtectionEnabled('Arms', True)

# motionProxy.rest()



pNames = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
          'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']

#run subject 11 and then 44
#run task1,task2,task3
# for subject_id, sections in poses.items():
#     if subject_id==11.0:
#         for section_id, section in sections.items():
#             if section_id =='task1':
#                 for i, d in enumerate(section['time']):
#                     pose = section['skeleton'][i]
#                     print pose
#                     pTargetAngles = pose.tolist()[0:]
#                     print pTargetAngles
#                     time.sleep(3)
#                     motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, 0.2)

#run 16 poses:

# for pos in poses_16:
#     pTargetAngles = pos.tolist()[0:]
#     print pTargetAngles
#     time.sleep(3)
#     motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, 0.2)



