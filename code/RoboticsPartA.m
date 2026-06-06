clc; clear; close all;

% Time parameters
T = 5;
t1 = 1.5;
t2 = 3.5;
dt = 0.01;

% Time vector
t = 0:dt:T;
n = length(t);

idx1 = find(t >= 0 & t <= t1);             % Phase 1: knob rotates
idx2 = find(t > t1 & t < t2);              % Phase 2: door moves
idx3 = find(t >= t2 & t <= T);             % Phase 3: knob returns

% --- Door trajectory
G0Heta_door = zeros(1, n);
for i = idx2
    G0Heta_door(i) = quinticPoly(t1, t2, 0, -deg2rad(30), t(i));
end
G0Heta_door(idx3) = -deg2rad(30);

% --- Knob trajectory
G0Heta_knob = zeros(1, n);
for i = idx1
    G0Heta_knob(i) = quinticPoly(0, t1, 0, -deg2rad(45), t(i));
end
G0Heta_knob(idx2) = -deg2rad(45);
for i = idx3
    G0Heta_knob(i) = quinticPoly(t2, T, -deg2rad(45), 0, t(i));
end

% Initialize end-effector trajectory storage {e}
G0E_traj = zeros(4,4,n);

figure;
trplot(eye(4), 'frame', '0', 'color', 'k');
hold on;

for i = 1:n
    cla(gca);
    trplot(eye(4), 'frame', '0', 'color', 'k');

    % Door frame: rotation about Z-axis
    R_door = trotz(G0Heta_door(i));
    G0D = transl(0, 2, 0) * R_door;
    trplot(G0D, 'frame', 'D', 'color', 'r');

    % Fixed rotation to align knob frame with door surface
    R = [  0   1   0;
           -1   0   0;
            0   0   1 ];

    % Knob frame: rotation about X-axis
    R_knob = trotx(G0Heta_knob(i));
    GDH = transl(0.9, 0, 0.7) * r2t(R) * R_knob;
    G0H = G0D * GDH;
    trplot(G0H, 'frame', 'h', 'color', 'm');

    % End-effector frame relative to knob frame
    RHE = [ 0 0 -1;
            0 1  0;
            1 0  0 ];
    GHE = transl(0.1, 0.1, 0) * r2t(RHE);
    G0E = G0H * GHE;
    G0E_traj(:,:,i) = G0E;
    trplot(G0E, 'frame', 'e', 'color', 'g');

    % Store unit quaternion of knob orientation
    q_h(:,i) = UnitQuaternion(t2r(G0H)).double';

    title(sprintf('t = %.2f sec | θ₁ = %.1f° | θ₂ = %.1f°', ...
        t(i), rad2deg(G0Heta_door(i)), rad2deg(G0Heta_knob(i))));
    axis equal;
    view(3);
    xlim([-1 3]); ylim([-1 3]); zlim([0 2]);
    drawnow;
end

% Plot unit quaternion components of knob orientation over time
figure;
plot(t, q_h);
legend('q_0','q_1','q_2','q_3');
xlabel('Time [s]');
ylabel('Quaternion Component');
title('Unit Quaternion Orientation of Handle');

% Save end-effector trajectory
save('end_effector_trajectory.mat', 'G0E_traj', 't', 'G0Heta_door', 'G0Heta_knob');

% --- 5th-degree polynomial trajectory function
% Computes smooth trajectory between q0 and qf over [t0, tf]
% with zero initial and final velocities and accelerations
function q = quinticPoly(t0, tf, q0, qf, t)
    T = tf - t0;
    tau = (t - t0) / T;
    tau = max(0, min(1, tau));
    q = q0 + (qf - q0)*(10*tau^3 - 15*tau^4 + 6*tau^5);
end
