%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASSIGNMENT: Project 1 
% CLASS: AE544 Analytical Dynamics
% AUTHOR: Giovanna Ucles
% DATE: 12 March 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

% Initalize Parameters
timeSpan = [0 10];
tolerance = 1e-12;
odeOptions = odeset('AbsTol', tolerance, 'RelTol', tolerance);

% Angular Velocity Function
omegaFunc = @(t) [pi/4 + 0.5*sin(5*pi*t);
                   pi/5 + 4*sin(5*pi*t)/3;
                   pi/6 + 0.25*sin(5*pi*t)];

%% Singularity for Asymmetric Euler Angles (3-2-1)
% Initial Euler Angles
initialEuler = [0.1; pi/2 - 0.01; 0.1]; % something close to a singularity

% Integrate Euler Angle Kinematics
[timeEuler45, euler45] = ode45(@(t, euler) eqnsEuler(euler, omegaFunc(t)), timeSpan, initialEuler, odeOptions);
[timeEuler4, euler4] = ode4(@(t, euler) eqnsEuler(euler, omegaFunc(t)), timeSpan, initialEuler);
[timeEuler15s, euler15s] = ode15s(@(t, euler) eqnsEuler(euler, omegaFunc(t)), timeSpan, initialEuler, odeOptions);

% Plot Euler Angles by ODE to see how angles change 
figure;
subplot(3, 1, 1); plot(timeEuler45, euler45); title('Euler Angles (3-2-1) - ODE45'); legend('\psi', '\theta', '\phi');
subplot(3, 1, 2); plot(timeEuler4, euler4); title('Euler Angles (3-2-1) - ODE4'); legend('\psi', '\theta', '\phi');
subplot(3, 1, 3); plot(timeEuler15s, euler15s); title('Euler Angles (3-2-1) - ODE15s'); legend('\psi', '\theta', '\phi');
exportgraphics(gcf, 'euler_angles_by_ode.png', 'Resolution', 300);

% Plot Euler Angles by angle to see how effective each ODE is
figure;
subplot(3, 1, 1);
plot(timeEuler45, euler45(:, 3), 'r', timeEuler4, euler4(:, 3), 'g', timeEuler15s, euler15s(:, 3), 'b');
title('Euler Angle \phi Comparison'); legend('ODE45', 'ODE4', 'ODE15s');

subplot(3, 1, 2);
plot(timeEuler45, euler45(:, 2), 'r', timeEuler4, euler4(:, 2), 'g', timeEuler15s, euler15s(:, 2), 'b');
title('Euler Angle \theta Comparison'); legend('ODE45', 'ODE4', 'ODE15s');

subplot(3, 1, 3);
plot(timeEuler45, euler45(:, 1), 'r', timeEuler4, euler4(:, 1), 'g', timeEuler15s, euler15s(:, 1), 'b');
title('Euler Angle \psi Comparison'); legend('ODE45', 'ODE4', 'ODE15s');
exportgraphics(gcf, 'euler_angles_by_angle.png', 'Resolution', 300);
%% Ambiguity of Euler Parameters (Quaternion)
% Initial Quaternion
cTo = eulerToDcm(initialEuler);
initialQuaternion = dcmToQuaternion(cTo);

% Integrate Quaternion Kinematics
[timeQuat45, quat45] = ode45(@(t, quat) eqnsQuaternion(quat, omegaFunc(t)), timeSpan, initialQuaternion, odeOptions);
[timeQuat4, quat4] = ode4(@(t, quat) eqnsQuaternion(quat, omegaFunc(t)), timeSpan, initialQuaternion);
[timeQuat15s, quat15s] = ode15s(@(t, quat) eqnsQuaternion(quat, omegaFunc(t)), timeSpan, initialQuaternion, odeOptions);

% Plot Quaternion by ODE to see how q1, q2, q3, and q4 change 
figure;
subplot(3, 1, 1); plot(timeQuat45, quat45); title('Quaternion - ODE45'); legend('q1', 'q2', 'q3', 'q4');
subplot(3, 1, 2); plot(timeQuat4, quat4); title('Quaternion - ODE4'); legend('q1', 'q2', 'q3', 'q4');
subplot(3, 1, 3); plot(timeQuat15s, quat15s); title('Quaternion - ODE15s'); legend('q1', 'q2', 'q3', 'q4');
exportgraphics(gcf, 'quaternion_components.png', 'Resolution', 300);

% Plot Quaternion by quaternions to see how effective each ODE is
figure;
for i = 1:4
    subplot(4, 1, i);
    plot(timeQuat45, quat45(:, i), 'r', timeQuat4, quat4(:, i), 'g', timeQuat15s, quat15s(:, i), 'b');
    title(['Quaternion Component q', num2str(i), ' Comparison']);
    legend('ODE45', 'ODE4', 'ODE15s');
end
exportgraphics(gcf, 'quaternion_components_comparison.png', 'Resolution', 300);

% Convert Quaternions to Euler 
eulerFromQuat45 = zeros(length(timeQuat45), 3);
for i = 1:length(timeQuat45)
    eulerFromQuat45(i, :) = dcmToEuler(quaternionToDcm(quat45(i, :)'));
end
figure; plot(timeQuat45, eulerFromQuat45); title('Euler Angles from Quaternion (ODE45)'); legend('\psi', '\theta', '\phi');
exportgraphics(gcf, 'euler_from_quaternion.png', 'Resolution', 300);
%% Classical Rodrigues Parameters (CRP)
% Initial CRP
initialCrp = quaternionToCrp(initialQuaternion);

% Integrate Rodrigues Parameters Kinematics
[timeCrp45, crp45] = ode45(@(t, crp) eqnsCrp(crp, omegaFunc(t)), timeSpan, initialCrp, odeOptions);
[timeCrp4, crp4] = ode4(@(t, crp) eqnsCrp(crp, omegaFunc(t)), timeSpan, initialCrp);
[timeCrp15s, crp15s] = ode15s(@(t, crp) eqnsCrp(crp, omegaFunc(t)), timeSpan, initialCrp, odeOptions);

% Plot Rodrigues Parameters Components
figure;
subplot(3, 1, 1); plot(timeCrp45, crp45); title('CRP - ODE45'); legend('\sigma1', '\sigma2', '\sigma3');
subplot(3, 1, 2); plot(timeCrp4, crp4); title('CRP - ODE4'); legend('\sigma1', '\sigma2', '\sigma3');
subplot(3, 1, 3); plot(timeCrp15s, crp15s); title('CRP - ODE15s'); legend('\sigma1', '\sigma2', '\sigma3');
exportgraphics(gcf, 'crp_components.png', 'Resolution', 300);

% Convert CRP back to Euler
eulerFromCrp45 = zeros(length(timeCrp45), 3);
for i = 1:length(timeCrp45)
    eulerFromCrp45(i, :) = dcmToEuler(crpToDcm(crp45(i, :)'));
end
figure; plot(timeCrp45, eulerFromCrp45); title('Euler Angles from CRP (ODE45)'); legend('\psi', '\theta', '\phi');
exportgraphics(gcf, 'euler_from_crp.png', 'Resolution', 300);
%% 3D Animation (Quaternion)
figure('Name', '3D Attitude Animation');
ax = axes('XLim', [-1 1], 'YLim', [-1 1], 'ZLim', [-1 1]);
view(30, 45); grid on; xlabel('X'); ylabel('Y'); zlabel('Z'); title('Rotating Frame Animation');
[x, y, z] = sphere(20); surf(x, y, z, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
lineX = line([0 1], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
lineY = line([0 0], [0 1], [0 0], 'Color', 'g', 'LineWidth', 2);
lineZ = line([0 0], [0 0], [0 1], 'Color', 'b', 'LineWidth', 2);

for i = 1:length(timeQuat45)
    cBn = quaternionToDcm(quat45(i, :)');
    set(lineX, 'XData', [0 cBn(1, 1)], 'YData', [0 cBn(2, 1)], 'ZData', [0 cBn(3, 1)]);
    set(lineY, 'XData', [0 cBn(1, 2)], 'YData', [0 cBn(2, 2)], 'ZData', [0 cBn(3, 2)]);
    set(lineZ, 'XData', [0 cBn(1, 3)], 'YData', [0 cBn(2, 3)], 'ZData', [0 cBn(3, 3)]);
    drawnow; pause(0.01);
end
%% Functions
function dq = eqnsQuaternion(q, omega)
    skewQ = skewMatrix(q(1:3));
    dq = 0.5*[skewQ + q(4)*eye(3, 3); -q(1:3)']*omega;
end

function dEuler = eqnsEuler(euler, omega)
    M = [0 sin(euler(3)) cos(euler(3)); 0 cos(euler(3))*cos(euler(2)) -sin(euler(3))*cos(euler(2)); cos(euler(2)) sin(euler(3))*sin(euler(2)) cos(euler(3))*sin(euler(2))]/cos(euler(2));
    dEuler = M*omega;
end

function dCrp = eqnsCrp(crp, omega)
    skewC = skewMatrix(crp);
    dCrp = 0.5 * (eye(3) + skewC + crp'*crp) * omega;
end

function skewMatrix = skewMatrix(v)
    skewMatrix = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function cBn = quaternionToDcm(q)
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    cBn = [1 - 2*q2^2 - 2*q3^2, 2*q1*q2 - 2*q3*q4, 2*q1*q3 + 2*q2*q4;
            2*q1*q2 + 2*q3*q4, 1 - 2*q1^2 - 2*q3^2, 2*q2*q3 - 2*q1*q4;
            2*q1*q3 - 2*q2*q4, 2*q2*q3 + 2*q1*q4, 1 - 2*q1^2 - 2*q2^2];
end

function q = dcmToQuaternion(C)
    q4 = 0.5 * sqrt(1 + trace(C));
    if q4 ~= 0
        q1 = 0.25 * (C(2, 3) - C(3, 2)) / q4;
        q2 = 0.25 * (C(3, 1) - C(1, 3)) / q4;
        q3 = 0.25 * (C(1, 2) - C(2, 1)) / q4;
    else
        q1 = 0.5*sqrt(1+C(1,1)-C(2,2)-C(3,3));
        q2 = 0.5*sqrt(1-C(1,1)+C(2,2)-C(3,3));
        q3 = 0.5*sqrt(1-C(1,1)-C(2,2)+C(3,3));
    end
    q = [q1; q2; q3; q4];
end

function crp = quaternionToCrp(q)
    if abs(1 + q(4)) < 1e-6
        crp = [0; 0; 0];
    else
        crp = q(1:3) / (1 + q(4));
    end
end

function cBn = crpToDcm(crp)
    skewC = skewMatrix(crp);
    cBn = eye(3) + (8*skewC^2-4*(1-dot(crp,crp))*skewC)/((1+dot(crp,crp))^2);
end

function cTo = eulerToDcm(euler)
    psi = euler(1); theta = euler(2); phi = euler(3);
    cTo = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
            sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
end

function euler = dcmToEuler(C)
    theta = -asin(C(3, 1));
    if abs(cos(theta)) < 1e-10
        psi = atan2(C(1, 2), C(2, 2));
        phi = 0;
    else
        psi = atan2(C(2, 1) / cos(theta), C(1, 1) / cos(theta));
        phi = atan2(C(3, 2) / cos(theta), C(3, 3) / cos(theta));
    end
    euler = [psi; theta; phi];
end

function [t, Y] = ode4(odeFun, tSpan, y0)
    N = 10;
    t = linspace(tSpan(1), tSpan(2), N);
    dt = t(2) - t(1);
    Y = zeros(length(y0), N);
    Y(:, 1) = y0;
    for i = 1:N - 1
        k1 = dt * odeFun(t(i), Y(:, i));
        k2 = dt * odeFun(t(i) + dt / 2, Y(:, i) + k1 / 2);
        k3 = dt * odeFun(t(i) + dt / 2, Y(:, i) + k2 / 2);
        k4 = dt * odeFun(t(i) + dt, Y(:, i) + k3);
        Y(:, i + 1) = Y(:, i) + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    end
    Y = Y';
end