clc; clear; close all;

% Load end-effector trajectory from Part A
load('end_effector_trajectory.mat');
dt = t(2) - t(1);
n = length(t);

% Initial joint configuration
q0 = [-1.7752, -1.1823, 0.9674, 0.2149, 1.3664, 1.5708]';
q = q0;
qm = zeros(n,6);
qm(1,:) = q0';
qdot_log = zeros(n,6);

% Load UR10 robot model
ur10 = ur10robot();
robot = ur10;

% Inverse kinematics via Jacobian pseudo-inverse (numerical integration)
for i = 1:n-1
    % Current and next end-effector pose
    T_curr = G0E_traj(:,:,i);
    T_next = G0E_traj(:,:,i+1);

    % Compute end-effector velocity (spatial delta)
    dx = tr2delta(T_curr, T_next) / dt;

    % End-effector Jacobian in world frame {0}
    J = robot.jacobe(q);

    % Joint velocities via pseudo-inverse of Jacobian
    q_dot = pinv(J) * dx;

    % Integrate joint positions
    q = q + q_dot * dt;

    % Log joint positions and velocities
    qm(i+1,:) = q';
    qdot_log(i+1,:) = q_dot';
end

% Animate robot motion
figure;
robot.plot(qm(1,:), 'workspace', [-1 2 -1 3 0 2], 'delay', 0.01);

for i = 2:3:n
    robot.animate(qm(i,:));
    title(sprintf('t = %.2f s', t(i)));
    pause(0.005);
end

% Plot joint positions over time
figure;
plot(t, qm);
xlabel('Time [s]');
ylabel('Joint Positions [rad]');
title('UR10 Joint Positions');
legend('q1','q2','q3','q4','q5','q6');

% Plot joint velocities over time
figure;
plot(t, qdot_log);
xlabel('Time [s]');
ylabel('Joint Velocities [rad/s]');
title('UR10 Joint Velocities');
legend('q̇1','q̇2','q̇3','q̇4','q̇5','q̇6');

% Compute final end-effector position and orientation errors
G0E_final = robot.fkine(q);         % Actual final end-effector pose
G0E_goal  = G0E_traj(:,:,end);      % Desired final end-effector pose

% Position error
pos_error = G0E_goal(1:3,4) - G0E_final.t;

% Orientation error (angle between rotation matrices)
R_err = G0E_goal(1:3,1:3) * G0E_final.R';
angle_error = acos((trace(R_err) - 1)/2);  % Angular difference in rad

fprintf('End-effector position error: [%.4f %.4f %.4f] m\n', pos_error);
fprintf('End-effector orientation error: %.4f rad (%.2f deg)\n', ...
    angle_error, rad2deg(angle_error));


% === UR10 Robot Model Functions ===

% Creates UR10 robot with base offset
function ur10 = ur10robot()
    ur10 = mdl_ur10();
    ur10.base.t = [1;1;0];
end

% Defines UR10 DH parameters and link inertial properties
function r = mdl_ur10()
    deg = pi/180;

    % Denavit-Hartenberg parameters
    a     = [0, -0.612, -0.5723, 0, 0, 0]';
    d     = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]';
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]';
    theta = zeros(6,1);
    DH = [theta d a alpha];

    % Link masses [kg]
    mass = [7.1, 12.7, 4.27, 2.0, 2.0, 0.365];

    % Link centers of mass [m]
    center_of_mass = [
        0.021, 0, 0.027;
        0.38,  0, 0.158;
        0.24,  0, 0.068;
        0.0,   0.007, 0.018;
        0.0,   0.007, 0.018;
        0,     0, -0.026];

    robot = SerialLink(DH, 'name', 'UR10', 'manufacturer', 'Universal Robotics');
    links = robot.links;
    for i = 1:6
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end

    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'ur10', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]);
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg);
    end
    r = robot;
end
