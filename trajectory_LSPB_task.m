%% Input initial and goal positions
init_pos = [170, 0, 0];
goal_pos = [0, 170, 0];
grasp_h = 70; 
grasp_d = 60; 

%% Set times for long distance movement
t0 = 0;
tf = 2;
timestep = 0.05;

%% Set times for short distance movement
t00 = 0;
timesteps = 0.1;
movestep = 2;
tff = grasp_d * timesteps / movestep;

init_up = init_pos + [0, 0, grasp_h];
init_grasp_pos = init_pos + [0, 0, grasp_h - grasp_d];
goal_up = goal_pos + [0, 0, grasp_h];
goal_grasp_pos = goal_pos + [0, 0, grasp_h - grasp_d];

startup_rvc;

%% Build the robot model
MDH = modified_DH();
d1 = MDH(1, 1);
d2 = MDH(2, 1);
d3 = MDH(3, 1);
d4 = MDH(4, 1);
d5 = MDH(5, 1);
d6 = MDH(6, 1);

a0 = MDH(1, 2);
a1 = MDH(2, 2);
a2 = MDH(3, 2);
a3 = MDH(4, 2);
a4 = MDH(5, 2);
a5 = MDH(6, 2);

alpha0 = MDH(1, 3);
alpha1 = MDH(2, 3);
alpha2 = MDH(3, 3);
alpha3 = MDH(4, 3);
alpha4 = MDH(5, 3);
alpha5 = MDH(6, 3);

L1 = Link([0 d1 a0 alpha0],'modified'); L1.offset = 0;
L2 = Link([0 d2 a1 alpha1],'modified'); L2.offset = -pi/2;
L3 = Link([0 d3 a2 alpha2],'modified'); L3.offset = -pi/2;
L4 = Link([0 d4 a3 alpha3],'modified'); L4.offset = 0;
L5 = Link([0 d5 a4 alpha4],'modified'); L5.offset = pi/2;
L6 = Link([0 d6 a5 alpha5],'modified');
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Robot');
robot.teach;
hold on;

%% Generate the trajectory
p0 = [0 0 0 0 0 0];
v0 = [0 0 0 0 0 0];
Ti = [1, 0, 0, init_up(1);
      0, -1, 0, init_up(2);
      0, 0, -1, init_up(3)];
pf = custom_inverse_kinematics(Ti);
vf = [0 0 0 0 0 0];
[q1] = LSPB_trajectory(p0, v0, pf, vf, t0, tf, timestep);

%% From GRASP_UP position to GRASP position
p_up = init_up;
take_or_place = 0; 
[q2] = Task_Space_Trajectory(p_up, t00, tff, timesteps, movestep, take_or_place);
q = [q1; q2];

%% From GRASP position to GRASP_UP position
p_offset = init_grasp_pos;
take_or_place = 1;
[q3] = Task_Space_Trajectory(p_offset, t00, tff, timesteps, movestep, take_or_place);
q = [q; q3];

%% From GRASP_UP position to GOAL_UP position
L = size(q);
p0 = q(L(1), :);
v0 = [0 0 0 0 0 0];
vf = [0 0 0 0 0 0];
Ti = [1, 0, 0, goal_up(1);
      0, -1, 0, goal_up(2);
      0, 0, -1, goal_up(3)];
pf = custom_inverse_kinematics(Ti);
[q4] = LSPB_trajectory(p0, v0, pf, vf, t0, tf, timestep);
q = [q; q4];

%% From GOAL_UP position to RELEASE position
p_offset = goal_up;
take_or_place = 0;
[q5] = Task_Space_Trajectory(p_offset, t00, tff, timesteps, movestep, take_or_place);
q = [q; q5];

%% From RELEASE position to GOAL_UP position
p_offset = goal_grasp_pos;
take_or_place = 1;
[q6] = Task_Space_Trajectory(p_offset, t00, tff, timesteps, movestep, take_or_place);
q = [q; q6];

%% From GOAL_UP position to HOME position
L = size(q);
p0 = q(L(1), :);
v0 = [0 0 0 0 0 0];
pf = [0, 0, 0, 0, 0, 0];
vf = [0 0 0 0 0 0];
[q7] = LSPB_trajectory(p0, v0, pf, vf, t0, tf, timestep);
q8 = [q; q7];

TT = robot.fkine(q8);
p = transl(TT);

figure('name','Trajectory')
plot3(p(:,1), p(:,2), p(:,3),'LineWidth',3)
robot.plot(q8);

%% Calculate and display forward kinematics for the final pose only
final_pose = robot.fkine(q8(end, :));
fprintf('Forward Kinematics for the Final Pose:\n');
disp(final_pose);

%% Calculate and display inverse kinematics for the final pose
q_inv = robot.ikine(final_pose, 'mask', [1 1 1 1 1 1]);
fprintf('Inverse Kinematics for Final Pose:\n');
disp(q_inv);

%% Verify consistency by computing forward kinematics for the inverse kinematics result
T_check = robot.fkine(q_inv);
fprintf('Check Forward Kinematics from Inverse Kinematics Result:\n');
disp(T_check);

%% Compare T_check with the original final_pose
fprintf('Original Final Pose:\n');
disp(final_pose);

fprintf('Difference between T_check and final_pose:\n');
disp(T_check - final_pose);

%% Check if the results are consistent within numerical precision
if max(max(abs(T_check - final_pose))) < 1e-10
    disp('The kinematics calculations are correct and consistent.');
else
    disp('There is a significant difference, please check the calculations.');
end
