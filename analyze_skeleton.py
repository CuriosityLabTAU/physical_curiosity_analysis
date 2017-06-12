###Imports:
import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import medfilt
import seaborn as sns
import scipy.optimize
from numpy.linalg import inv
from sklearn.cluster import KMeans
import statsmodels.formula.api as sm
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D

###Parameters:
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

### Get poses from data:
# def get_poses(angles):
#     t = angles[:, 0]
#     # plt.plot(t, angles[:,1:])
#
#     f_angles = medfilt(angles[:, 1:], [median_filter_window, 1])
#     # plt.plot(t, f_angles)
#
#     d_angles = np.gradient(f_angles, axis=0)
#     total_derivative = np.sum(d_angles ** 2, axis=1)
#     # plt.plot(t, total_derivative)
#     # plt.show()
#
#     # binarize to movement/no-movement
#     bin = np.argwhere(total_derivative < movement_threshold)
#     bin = np.array([x[0] for x in bin])
#     # print(bin)
#     d_bin = bin[1:] - bin[:-1]
#     # print(d_bin)
#
#     # get start/stop of no movement
#     start_stop = np.argwhere(d_bin > 1)
#     no_movement_bins_start = np.array([x[0] for x in bin[start_stop[:-1]+1]])
#     no_movement_bins_stop = np.array([x[0] for x in bin[start_stop[1:]]])
#     # print(no_movement_bins_start, no_movement_bins_stop)
#     # get angles of middle of no-movement section
#     middle_bin = [(no_movement_bins_start[i] + no_movement_bins_stop[i])/2.0 for i in range(no_movement_bins_start.shape[0])]
#     middle_bin = [int(x) for x in middle_bin]
#     # print(middle_bin)
#
#     pose = angles[middle_bin,:]
#     return pose ,middle_bin
#
# #Save skeleton robot poses, depending on delay:
# for delay in range(0,50):
#     print delay
#     data = pickle.load(open('raw_data_all', 'rb'))
#     # data[id][section] = array(dict{skeleton, robot, time})
#     poses = {}
#
#     for subject_id, sections in data.items():           # go over subject
#         if subject_id==35.0: ###NO DATA ON TASK 1
#             continue
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
#


### Get error for each time stamp - error = skeleton * matrix - robot:
# avg_error_per_delays=[]
# for delay in range(0,29):
#
#
#     poses = pickle.load(open('data_after_analysis_'+str(delay), 'rb'))
#
#     skeleton_metrix_robot_error={}
#     for subject_id, sections in poses.items():
#         skeleton_metrix_robot_error[subject_id]={}
#         which_matrix = int(subject_id) % 2
#         for section_id, section in sections.items():
#             section_error=[]
#             for i, d in enumerate(section['time']):
#
#                 if section_id=='basic':
#                     robot_calculation=np.dot(base_matrices['basic'], section['skeleton'][i])
#                 elif which_matrix == 0:
#                     robot_calculation = np.dot(base_matrices['LShoulderPitch-RShoulderRoll'], section['skeleton'][i])
#                 else:
#                     robot_calculation = np.dot(base_matrices['LShoulderRoll-RShoulderPitch'], section['skeleton'][i])
#
#                 error=np.linalg.norm((robot_calculation-section['robot'][i])[(0,1,4,5),])/4
#
#                 section_error.append(error)
#
#                 skeleton_metrix_robot_error[subject_id][section_id] = {
#                 'time': section['time'],
#                 'error': section_error
#             }
#
#
#     #Data for plot:
#     avg_error_per_subject=[]
#     subject_id_for_plot=[]
#     for subject_id, sections in skeleton_metrix_robot_error.items():
#         avg_section=[]
#         for section_id, section in sections.items():
#             avg_section.append(np.nanmean(section['error']))
#
#         avg_error_per_subject.append(np.rad2deg(np.nanmean(avg_section)))
#         subject_id_for_plot.append(int(subject_id))
#     avg_error_per_delays.append(avg_error_per_subject)
#
# #Find time interval:
# intraval_time=[]
# data = pickle.load(open('raw_data', 'rb'))
# for subject_id, sections in data.items():
#     for section_id, section in sections.items():
#         if 'trans' not in section_id:
#             now=0
#             for step in section['data']:
#                 intraval= step['time'] -now
#                 intraval_time.append(intraval)
#                 now =step['time']
# intraval_time = round(np.median(intraval_time),2)
#
# #Plot
# data=[]
# for i in range(0,29):
#     lists=[[x, i*intraval_time] for x in avg_error_per_delays[i]]
#     [data.append(x) for x in lists]
# error=pd.DataFrame(data,columns=['error','delay'])
#
# for_rank= error.groupby(['delay'],as_index=False).mean()
# for_rank= for_rank['error']
#
# sns.set_style("whitegrid")
# pal = sns.color_palette("Blues_d", len(for_rank))
# rank = for_rank.argsort().argsort()
# ax = sns.barplot(x="delay", y="error", data=error, capsize=.2 ,palette=np.array(pal[::-1])[rank])
# ax.set(xlabel='Delay(sec)', ylabel='Avg Error (degrees)')
# sns.plt.title('Avg Error between robot angles and skeleton angles, in different delays')
# sns.plt.show()


# #Taking all poses from *all* subjects:
subject_number_of_poses={}
poses = pickle.load(open('data_after_analysis_17', 'rb'))
all_poses=np.empty((0,8))
for subject_id, sections in poses.items():
    subject_number_of_poses[subject_id]=0
    for section_id, section in sections.items():
        for i, d in enumerate(section['time']):
            if section_id == 'learn':
                subject_number_of_poses[subject_id] += 1
            all_poses = np.vstack((all_poses, section['skeleton'][i]))
subject_number_of_poses_df=pd.DataFrame(subject_number_of_poses.items(), columns=['subject_id', 'number_of_poses'])
# print subject_number_of_poses_df


### createing matrix error:
poses = pickle.load(open('data_after_analysis_15', 'rb'))

matrix_error = {}
for subject_id, sections in poses.items():
    if subject_id==35.0: ###NO DATA ON TASK 1
        continue
    if subject_id==14.0: ###NO DATA ON TASK 3
        continue
    if subject_id==20.0: ###NO DATA ON TASK 2
        continue
    if subject_id==37.0: ###NO DATA ON TASK 2
        continue
    matrix_error[subject_id] = {}
    #matrix that was used for subject
    which_matrix = int(subject_id) % 2
    if which_matrix == 0:
        matrix = base_matrices['LShoulderPitch-RShoulderRoll']
    else:
        matrix = base_matrices['LShoulderRoll-RShoulderPitch']

    for section_id, section in sections.items():
        if section_id == 'learn':
            skeleton_vectors=np.empty((0,8))
            robot_vectors =np.empty((0,8))
            for i, d in enumerate(section['time']):
                # print i

                skeleton_vectors=np.vstack((skeleton_vectors, section['skeleton'][i]))
                robot_vectors=np.vstack((robot_vectors, section['robot'][i]))

                if i > 0:
                    pinv_skeleton = np.linalg.pinv(skeleton_vectors)
                    Amat = np.dot(pinv_skeleton, robot_vectors)

                    error= np.linalg.norm((matrix - Amat)[(0,1,4,5),])/4

                    matrix_error[subject_id][i+1] = np.rad2deg(error)

#statistical results - histogram of matrix_error across subjects:
for_correlation=[]
x=[]
for subject_id,i in matrix_error.items():
    x.append(i[min(i, key=i.get)])
    for_correlation.append([subject_id,i[min(i, key=i.get)]])
sns.set(color_codes=True)
hist =sns.distplot(x, bins=9, kde=False, rug=True)
hist.set(xlabel='Min Error (degrees)', ylabel='Number of subjects')
sns.plt.title('Histogram of matrix error across subjects')
sns.plt.show()


# ###tasks:

poses = pickle.load(open('data_after_analysis_15', 'rb'))

##Get task error (error = pose - task_pose):
task_error = {}
for subject_id, sections in poses.items():
    if subject_id==35.0: ###NO DATA ON TASK 1
        continue
    if subject_id==14.0: ###NO DATA ON TASK 3
        continue
    if subject_id==20.0: ###NO DATA ON TASK 2
        continue
    if subject_id==37.0: ###NO DATA ON TASK 2
        continue
    task_error[subject_id] = {}
    which_matrix = int(subject_id) % 2
    if which_matrix == 0:
        subject_matrix = base_matrices['LShoulderPitch-RShoulderRoll']
    else:
        subject_matrix = base_matrices['LShoulderRoll-RShoulderPitch']

    for section_id, section in sections.items():
        if section_id in ['task1','task2','task3']:
            task_error[subject_id][section_id] = []
            for i, d in enumerate(section['time']):
                pose=section['skeleton'][i]
                error=0
                task_pose_original=0
                if section_id == 'task1':
                    task_pose_original=np.dot(np.array([0.00, 0.00, 0.00, -0.034, 0.00, 0.00, 0.00, 0.034]), inv(subject_matrix))
                    error=pose-task_pose_original

                elif section_id == 'task2':
                    task_pose_original = np.dot(np.array([1.25, 0.00, 0.00, -0.034, 1.25, 0.00, 0.00, 0.034]), inv(subject_matrix))
                    error = pose - task_pose_original

                elif section_id == 'task3':
                    task_pose_original = np.dot(np.array([1.45, 1.00, 0.00, -0.034, 1.45, -1.00, 0.00, 0.034]), inv(subject_matrix))
                    error = (pose - task_pose_original)[(0,1,4,5),]

                agg_error=np.rad2deg(np.linalg.norm(error)/8)
                task_error[subject_id][section_id].append(agg_error)

##Get score -  (find lowest point = score):
task_error_score = {}
for subject_id, sections in task_error.items():
    task_error_score[subject_id] = {}
    for section_id, section in sections.items():
        task_error_score[subject_id][section_id] = min(task_error[subject_id][section_id])
task_error_score_df=pd.DataFrame.from_dict(task_error_score,orient='index')
task_error_score_df['subject_id']=task_error_score_df.index
task_error_score_df=pd.melt(task_error_score_df, id_vars=["subject_id"], var_name="Task", value_name="Task score")

##Statistical results on tasks:
#plot all score tasks:
sns.set_style("whitegrid")
ax = sns.barplot(x="subject_id", y="Task score", hue="Task" , data=task_error_score_df)
ax.set(xlabel='Subject ID', ylabel='Task Score(degrees)')
sns.plt.title('Score in tasks across subjects')
sns.plt.show()

# #plot total score for subject id:
total_score = {}
for subject_id, sections in task_error_score.items():
    total_score[subject_id] = 0
    for section_id, section in sections.items():
        total_score[subject_id] += task_error_score[subject_id][section_id]
    for item in for_correlation:
        if item[0]==subject_id:
            item.append(total_score[subject_id])
            break

total_score_df=pd.DataFrame(total_score.items(), columns=['subject_id', 'total_score'])
total_score_df=total_score_df.sort_values('subject_id')

for_rank= total_score_df['total_score']
sns.set_style("whitegrid")
pal = sns.color_palette("Blues", len(for_rank))
rank = for_rank.argsort().argsort()
ax = sns.barplot(x="subject_id", y="total_score", data=total_score_df, capsize=.2 ,palette=np.array(pal[::])[rank])
ax.set(xlabel='Subject ID', ylabel='Total Score(degrees)')
sns.plt.title('Total score across subjects')
sns.plt.show()

# #plot histogram of error over all subjects:
x=total_score_df['total_score'].tolist()
sns.set(color_codes=True)
hist =sns.distplot(x, bins=4, kde=False, rug=True)
hist.set(xlabel='Total Score(degrees)', ylabel='Number of subjects')
sns.plt.title('Histogram Total score across subjects')
sns.plt.show()

# #plot histogram of error over tasks (task 1 << task3)
fig, (ax1, ax2, ax3) = plt.subplots(ncols=3, sharey=True)

x1=task_error_score_df.set_index(['Task']).loc['task1']['Task score'].tolist()
sns.set(color_codes=True)
sns.distplot(x1, bins=5, kde=False, rug=True , ax=ax1)
ax1.set(xlabel='Best Score(degrees)', ylabel='Number of subjects' ,title='Task1')

x2=task_error_score_df.set_index(['Task']).loc['task2']['Task score'].tolist()
sns.set(color_codes=True)
sns.distplot(x2, bins=5, kde=False, rug=True ,ax=ax2)
ax2.set(xlabel='Best Score(degrees)', ylabel='Number of subjects' ,title='Task2')

x3=task_error_score_df.set_index(['Task']).loc['task3']['Task score'].tolist()
sns.set(color_codes=True)
sns.distplot(x3, bins=5, kde=False, rug=True, ax=ax3)
ax3.set(xlabel='Best Score(degrees)', ylabel='Number of subjects' ,title='Task3')
fig.suptitle('Histogram of error over tasks')
sns.plt.show()

# #plot correlation - best_score ~ matrix_error
for_correlation_df = pd.DataFrame(for_correlation, columns=['subject_id','matrix_error','best_score'])
result = sm.ols(formula="best_score ~ matrix_error", data=for_correlation_df).fit()
print result.summary()
sns.lmplot(x='matrix_error',y='best_score',data=for_correlation_df,fit_reg=True)
sns.plt.show()

# #plot correlation - best_score ~ matrix_error
for_correlation_df=pd.merge(for_correlation_df, subject_number_of_poses_df, how='inner', on='subject_id')
print for_correlation_df
result = sm.ols(formula="best_score ~ matrix_error + number_of_poses", data=for_correlation_df).fit()
print result.summary()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_surf = np.arange(0, 30, 4)                # generate a mesh
y_surf = np.arange(0, 90, 4)
x_surf, y_surf = np.meshgrid(x_surf, y_surf)

exog = pd.core.frame.DataFrame({'matrix_error': x_surf.ravel(), 'number_of_poses': y_surf.ravel()})
out = result.predict(exog = exog)
ax.plot_surface(x_surf, y_surf,
                out.reshape(x_surf.shape),
                rstride=1,
                cstride=1,
                color='None',
                alpha = 0.2)

ax.scatter(for_correlation_df['matrix_error'], for_correlation_df['number_of_poses'], for_correlation_df['best_score'],
           c='blue',
           marker='o',
           alpha=1)

ax.view_init(elev=15., azim=180)

ax.set_xlabel('matrix_error')
ax.set_ylabel('number_of_poses')
ax.set_zlabel('best_score')

plt.show()



# ###Crate n poses:

## k-means on all poses. k=16:
kmeans = KMeans(n_clusters=16, n_init=50 ).fit(all_poses)
poses_16= kmeans.cluster_centers_
pickle.dump(obj=poses_16, file=open('../physical_curiosity_analysis/poses_16', 'wb'))

print poses_16

#Todo check the 16 poses.

# TODO new: construct actions space and find optimal sequence


# --- only on section 2 -- weird matrix
# 3. learner - pinv matrix.
# 4. reward = matrix_t - matrix_t-1
# 5. policy - sequence of poses(k=16). p(a_t | a_t-1)
# 6. output ==> optimal sequence of poses


# TODO: model parts
# 1. learner: skeleton --> robot
# 2. execution: given learner and task --> action
# 3. curiosity: given world --> optimal actor
# 4. comparison of optimal actor and actual action sequence

# (6) TODO: good results
# 1. given learned matrix --> expected control
# 2. for each subject: 3 scores and matrix_error. correlation.

# TODO: deep discussion
# - assumption: learning is important for task execution. checked by result 2.
#
# k-means on all poses of all people
#
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

#Todo old:
# [ alpha-kronbach - internal consistency of scores] of tasks.


# given initial pose (skeleton): pose_initial
# given learned matrix and task : pose_final
# --> plot trajectory (linear extrapolation)
# don't know how (temporal warping) - compare real and optimal trajectory


# given skeleton and robot --> matrix
# assumption: initial matrix is mirror.
# assumption: people are optimal (learner)
# - mathematical model of finding a matrix (from torr) [done]
# --- reward = sum delta_matrix_elements
# - neural networks: skeleton --> robot (no hidden layer, linear output)
#
# -- on-line learning (play with learning rate)
# --- reward = prediction error
# ==> per subject, per section --> matrix
#     compare real matrix to learned matrix
#     matrix_error