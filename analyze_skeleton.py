import pickle
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import medfilt

median_filter_window = 31
movement_threshold = 0.002

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


# data = pickle.load(open('raw_data', 'rb'))
# # data[id][section] = array(dict{skeleton, robot, time})
# poses = {}
# for subject_id, sections in data.items():           # go over subject
#     poses[subject_id] = {}
#     for section_id, section in sections.items():    # go over sections
#         if 'trans' not in section_id:               # not 'transformation'
#
#             # print(section_id)
#             time_stamp = np.zeros([len(section['data']), 1])
#             skeleton_angles = np.zeros([len(section['data']), 8])
#             robot_angles=np.zeros([len(section['data']), 8])
#             for i, d in enumerate(section['data']): # go over time-steps
#                 time_stamp[i,0] = d['time']
#
#                 skeleton_angles[i, :] = np.array([float(x) for x in d['skeleton'].split(',')])
#
#                 robot_angles[i, :] = np.array([float(x) for x in d['robot'].split(';')[1].split(',')])
#
#             skeleton_poses, pose_bins = get_poses(skeleton_angles)
#             robot_poses = robot_angles[pose_bins,:]
#             time_stamp=time_stamp[pose_bins,:]
#
            # poses[subject_id][section_id] = {
            #     'time': time_stamp,
            #     'skeleton': skeleton_poses,
            #     'robot': robot_poses
            # }


# pickle.dump(obj=poses, file=open('../analysis/data_after_analysis', 'wb'))

poses = pickle.load(open('data_after_analysis', 'rb'))


# (1) TODO: for each time stamp: skeleton * matrix = robot
# error = skeleton * matrix - robot

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

            error=np.linalg.norm(robot_calculation-section['robot'])/8
            # if np.isnan(error):
            #     print section['robot']
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
        avg_section.append(np.mean(section['error']))
    avg_error_per_subject.append(np.mean(avg_section))
    subject_id_for_plot.append(int(subject_id))

y_pos = np.arange(len(subject_id_for_plot))
plt.bar(y_pos, avg_error_per_subject, align='center', alpha=0.5)
plt.xticks(y_pos, subject_id_for_plot)
plt.ylabel('Avg Error (radians)')
plt.xlabel('Subject ID')

plt.show()


# (2) TODO: create matrix
# talk to torr
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
