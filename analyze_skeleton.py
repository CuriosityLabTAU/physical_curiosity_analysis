import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import medfilt
import seaborn as sns
import scipy.optimize

median_filter_window = 31
movement_threshold = 0.002

###For switch angles matrix:
def switch_angles(angle_name_0, angle_name_1):
    matrix = np.eye(8)
    angle_0 = pNames.index(angle_name_0)
    angle_1 = pNames.index(angle_name_1)
    matrix[angle_0, angle_0] = 0
    matrix[angle_1, angle_1] = 0
    matrix[angle_0, angle_1] = 1
    matrix[angle_1, angle_0] = 1
    return matrix


base_matrices = np.eye(8)
pNames = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
          'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
base_matrices = {}
base_matrices['basic'] = np.eye(8)
base_matrices['LShoulderPitch-RShoulderRoll'] = switch_angles('LShoulderPitch', 'RShoulderRoll')
base_matrices['LShoulderRoll-RShoulderPitch'] = switch_angles('LShoulderRoll', 'RShoulderPitch')


def get_poses(angles):
    t = angles[:, 0]
    # plt.plot(t, angles[:,1:])

    f_angles = medfilt(angles[:, 1:], [median_filter_window, 1])
    # plt.plot(t, f_angles)

    d_angles = np.gradient(f_angles, axis=0)
    total_derivative = np.sum(d_angles ** 2, axis=1)
    # plt.plot(t, total_derivative)
    # plt.show()

    # binarize to movement/no-movement
    bin = np.argwhere(total_derivative < movement_threshold)
    bin = np.array([x[0] for x in bin])
    # print(bin)
    d_bin = bin[1:] - bin[:-1]
    # print(d_bin)

    # get start/stop of no movement
    start_stop = np.argwhere(d_bin > 1)
    no_movement_bins_start = np.array([x[0] for x in bin[start_stop[:-1]+1]])
    no_movement_bins_stop = np.array([x[0] for x in bin[start_stop[1:]]])
    # print(no_movement_bins_start, no_movement_bins_stop)
    # get angles of middle of no-movement section
    middle_bin = [(no_movement_bins_start[i] + no_movement_bins_stop[i])/2.0 for i in range(no_movement_bins_start.shape[0])]
    middle_bin = [int(x) for x in middle_bin]
    # print(middle_bin)

    pose = angles[middle_bin,:]
    return pose ,middle_bin



# for delay in range(0,50):
#     print delay
#     data = pickle.load(open('raw_data', 'rb'))
#     # data[id][section] = array(dict{skeleton, robot, time})
#     poses = {}
#     for subject_id, sections in data.items():           # go over subject
#         poses[subject_id] = {}
#         for section_id, section in sections.items():    # go over sections
#             if 'trans' not in section_id:               # not 'transformation'
#
#                 # print(section_id)
#                 time_stamp = np.zeros([len(section['data']), 1])
#                 skeleton_angles = np.zeros([len(section['data']), 8])
#                 robot_angles=np.zeros([len(section['data']), 8])
#                 for i, d in enumerate(section['data']): # go over time-steps
#                     time_stamp[i,0] = d['time']
#
#                     skeleton_angles[i, :] = np.array([float(x) for x in d['skeleton'].split(',')])
#
#                     robot_angles[i, :] = np.array([float(x) for x in d['robot'].split(';')[1].split(',')])
#
#                 skeleton_poses, pose_bins = get_poses(skeleton_angles)
#                 skeleton_poses=skeleton_poses[:-3]
#                 pose_bins=pose_bins[:-3]
#
#                 robot_poses = robot_angles[[x+delay for x in pose_bins],:]
#
#                 time_stamp=time_stamp[pose_bins,:]
#
#                 poses[subject_id][section_id] = {
#                     'time': time_stamp,
#                     'skeleton': skeleton_poses,
#                     'robot': robot_poses
#                 }
#
#     pickle.dump(obj=poses, file=open('../physical_curiosity_analysis/data_after_analysis_'+str(delay), 'wb'))
avg_error_per_delays=[]
for delay in range(0,42):


    poses = pickle.load(open('data_after_analysis_'+str(delay), 'rb'))


    # (1) TODO: for each time stamp: skeleton * matrix = robot
    # error = skeleton * matrix - robot

    skeleton_metrix_robot_error={}
    for subject_id, sections in poses.items():
        skeleton_metrix_robot_error[subject_id]={}
        which_matrix = int(subject_id) % 2
        for section_id, section in sections.items():
            section_error=[]
            for i, d in enumerate(section['time']):

                if section_id=='basic':
                    robot_calculation=np.dot(base_matrices['basic'], section['skeleton'][i])
                elif which_matrix == 0:
                    robot_calculation = np.dot(base_matrices['LShoulderPitch-RShoulderRoll'], section['skeleton'][i])
                else:
                    robot_calculation = np.dot(base_matrices['LShoulderRoll-RShoulderPitch'], section['skeleton'][i])

                error=np.linalg.norm(robot_calculation-section['robot'][i])/8

                section_error.append(error)

                skeleton_metrix_robot_error[subject_id][section_id] = {
                'time': section['time'],
                'error': section_error
            }


    #for plot:
    avg_error_per_subject=[]
    subject_id_for_plot=[]
    for subject_id, sections in skeleton_metrix_robot_error.items():
        avg_section=[]
        for section_id, section in sections.items():
            avg_section.append(np.nanmean(section['error']))

        avg_error_per_subject.append(np.rad2deg(np.nanmean(avg_section)))
        subject_id_for_plot.append(int(subject_id))
    avg_error_per_delays.append(avg_error_per_subject)

#Find time interval:
intraval_time=[]
data = pickle.load(open('raw_data', 'rb'))
for subject_id, sections in data.items():
    for section_id, section in sections.items():
        if 'trans' not in section_id:
            now=0
            for step in section['data']:
                intraval= step['time'] -now
                intraval_time.append(intraval)
                now =step['time']
intraval_time = round(np.median(intraval_time),2)

#Plot
data=[]
for i in range(0,42):
    lists=[[x, i*intraval_time] for x in avg_error_per_delays[i]]
    [data.append(x) for x in lists]
error=pd.DataFrame(data,columns=['error','delay'])

for_rank= error.groupby(['delay'],as_index=False).mean()
for_rank= for_rank['error']

sns.set_style("whitegrid")
pal = sns.color_palette("Blues_d", len(for_rank))
rank = for_rank.argsort().argsort()
ax = sns.barplot(x="delay", y="error", data=error, capsize=.2 ,palette=np.array(pal[::-1])[rank])
ax.set(xlabel='Delay(sec)', ylabel='Avg Error (degrees)')
sns.plt.title('Avg Error between robot angles and skeleton angles, in different delays')

# sns.plt.show()

# (2) TODO: create matrix
#  talk to torr
#~~~~~~~~~~~~~this is from torr~~~~~~
# option 1

# res = scipy.optimize.least_squares(predict_row, x0, args=(Xmat, Ymat[row, :]))
#
# def predict_row(row_in_A, a_12x34, row_in_a1234):
#     return row_in_a1234 - np.dot(row_in_A, a_12x34)
#
#  option 2
#  finding the matrix using linear algebra
# Amat = np.linalg.lstsq(Xmat.T, Ymat.T)
# Amat = Amat.T
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# # use delay =15
# poses = pickle.load(open('data_after_analysis_15', 'rb'))
#
# create_matrix_error = {}
#
# skeleton_metrix_robot_error = {}
# for subject_id, sections in poses.items():
#     create_matrix_error[subject_id] = {}
#     which_matrix = int(subject_id) % 2
#     for section_id, section in sections.items():
#         skeleton_vectors=np.empty((0,8))
#         robot_vectors =np.empty((0,8))
#         for i, d in enumerate(section['time']):
#
#             if section_id == 'basic':
#                 robot_calculation = np.dot(base_matrices['basic'], section['skeleton'][i])
#             # elif which_matrix == 0:
#             #     robot_calculation = np.dot(base_matrices['LShoulderPitch-RShoulderRoll'], section['skeleton'][i])
#             # else:
#             #     robot_calculation = np.dot(base_matrices['LShoulderRoll-RShoulderPitch'], section['skeleton'][i])
#
#                 skeleton_vectors=np.vstack((skeleton_vectors, section['skeleton'][i]))
#                 robot_vectors=np.vstack((robot_vectors, section['robot'][i]))
#                 print skeleton_vectors.shape
#
#                 Amat = np.linalg.lstsq(skeleton_vectors, robot_vectors)[0]
#
#                 print robot_vectors - np.dot(skeleton_vectors,Amat)
#                 print robot_vectors - np.dot(skeleton_vectors, np.eye(8))
#
#                 # print Amat
#                 print 'ff'
#                 # Amat = Amat.T
#                 # print Amat
#                 # break
#         break
#     break


# given skeleton and robot --> matrix

# (4) TODO: learning of subject
# assumption: initial matrix is mirror.
# assumption: people are optimal (learner)
# - mathematical model of finding a matrix (from torr)
# --- reward = sum delta_matrix_elements
# - neural networks: skeleton --> robot (no hidden layer, linear output)

# -- on-line learning (play with learning rate)
# --- reward = prediction error
# ==> per subject, per section --> matrix
#     compare real matrix to learned matrix
#     matrix_error


# (3) TODO: task
# error = pose - task_pose
poses = pickle.load(open('data_after_analysis_15', 'rb'))

task_error = {}
for subject_id, sections in poses.items():
    task_error[subject_id] = {}
    which_matrix = int(subject_id) % 2
    if which_matrix == 0:
        subject_matrix = base_matrices['LShoulderPitch-RShoulderRoll']
    else:
        subject_matrix = base_matrices['LShoulderRoll-RShoulderPitch']

    for section_id, section in sections.items():
        task_error[subject_id][section_id] = []
        for i, d in enumerate(section['time']):
            pose=section['skeleton'][i]
            error=0
            task_pose_original=0
            if section_id == 'task1':
                task_pose_original=np.dot(np.array([1, 2, 0, 0, 5, 6, 0, 0]),subject_matrix)
                error=pose-task_pose_original

            elif section_id == 'task2':
                task_pose_original = np.dot(np.array([1, 2, 0, 0, 5, 6, 0, 0]), subject_matrix)
                error = pose - task_pose_original

            elif section_id == 'task3':
                task_pose_original = np.dot(np.array([1, 2, 0, 0, 5, 6, 0, 0]), subject_matrix)
                error = pose - task_pose_original

            else:
                continue

            task_error[subject_id][section_id].append(error)


# find lowest point = score

# TODO: task
# given initial pose (skeleton): pose_initial
# given learned matrix and task : pose_final
# --> plot trajectory (linear extrapolation)
# don't know how (temporal warping) - compare real and optimal trajectory

# TODO: model parts
# 1. learner: skeleton --> robot
# 2. execution: given learner and task --> action
# 3. curiosity: given world --> optimal actor
# 4. comparison of optimal actor and actual action sequence

# (5) TODO: statistical results
# 1. histogram of matrix_error across subjects
# 2. total_score = sum of three scores
# 3. [ alpha-kronbach - internal consistency of scores]
# 4. histogram of total_score across subjects

# (6) TODO: good results
# 1. given learned matrix --> expected control
# 2. for each subject: 3 scores and matrix_error. correlation.

# TODO: deep discussion
# - assumption: learning is important for task execution. checked by result 2.

# k-means on all poses of all people

# sections = ['basic', 'learn', 'task1', 'task2', 'task3']
# for subject_id, sections in data.items():
#     poses[subject_id]= poses[subject_id].items()
#     print poses[subject_id]
#     break
#
# # for section_id in sections:
# #     print poses[][section_id]
# data1 = pd.DataFrame.from_dict(poses, orient='columns', dtype=None)
# print data1
#
