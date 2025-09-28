startup_rvc;

%% Modified D-H Model
d1 = 60;
d2 = 0;
d3 = 0;
d4 = 130;
d5 = 0;
d6 = 90;

a0 = 0;
a1 = 0;
a2 = 150;
a3 = 70;
a4 = 0;
a5 = 0;

alpha0 = 0;
alpha1 = -pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;

% Create links using the Modified D-H parameters
L1 = Link([0 d1 a0 alpha0],'modified'); L1.offset = 0;
L2 = Link([0 d2 a1 alpha1],'modified'); L2.offset = -pi/2;
L3 = Link([0 d3 a2 alpha2],'modified'); L3.offset = -pi/2;
L4 = Link([0 d4 a3 alpha3],'modified'); L4.offset = 0;
L5 = Link([0 d5 a4 alpha4],'modified'); L5.offset = pi/2;
L6 = Link([0 d6 a5 alpha5],'modified');

% Combine the links into a robot model
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Robot');

p0 = [0 0 0 0 0 0];
v0 = [0 0 0 0 0 0];
pf = [0.7551 0.7565 1.7951 0 -0.9808 0.7551];
vf = [0 0 0 0 0 0];
t0 = 0;
tf = 2;
timestep = 0.05;

% Generate synchronized bang-bang trajectory
[q_bangbang, t_max_bangbang] = BangBangTrajectory_Synchronized(p0, v0, pf, vf, t0, tf, timestep);

% Plot Bang-Bang Trajectory
T_bangbang = robot.fkine(q_bangbang); 
p_bangbang = transl(T_bangbang);
figure('name','Bang-Bang Trajectory')
plot3(p_bangbang(:,1), p_bangbang(:,2), p_bangbang(:,3),'LineWidth',3)
robot.plot(q_bangbang);

robot.display(); 
% robot.teach; % Uncomment this line if you need the Teach window for interactive control
hold on;

%% Initial Joint Configuration
q_initial = [0.1, 0.1, 0.0193, 0.1193, 0, 0]; 

%% Compute Forward Kinematics for the Given Joint Configuration
T_robot = robot.fkine(q_initial); 
disp('Forward Kinematics for the Initial Joint Configuration:');
disp(T_robot);

%% Test Inverse Kinematics by Using the Pose from Forward Kinematics
final_pose = T_robot;
q_inv = robot.ikine(final_pose, 'mask', [1 1 1 1 1 1]); 
disp('Inverse Kinematics for the Pose Obtained from Forward Kinematics:');
disp(q_inv);

%% Verify Consistency by Computing Forward Kinematics for the Inverse Kinematics Result
T_check = robot.fkine(q_inv);
disp('Forward Kinematics from Inverse Kinematics Result:');
disp(T_check);

%% Compare T_check with the Original Final Pose
disp('Original Final Pose:');
disp(final_pose);

disp('Difference between T_check and Final Pose:');
disp(T_check - final_pose);

%% Check if the results are consistent within numerical precision
if max(max(abs(T_check - final_pose))) < 1e-10
    disp('The kinematics calculations are correct and consistent.');
else
    disp('There is a significant difference, please check the calculations.');
end

%% Additional Calculation of Forward Kinematics for Different q2 Values
fprintf('Forward Kinematics for Different q2 Values:\n');
q2_values = [1, -0.5, 0.5, 0.66, -0.9, 1.5]; % Example values for q2
for i = 1:length(q2_values)
    q = [0, q2_values(i), 0, 0, 0, 0]; % Set only q2, others are zero
    T = robot.fkine(q); % Calculate forward kinematics
    fprintf('robot.fkine(q) for q2 = %g:\n', q2_values(i));
    disp(T);
end
