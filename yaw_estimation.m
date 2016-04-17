%% Author: Akhil Bagaria
%  Input: Gyroscope data from iPhone
%  Output: Attitude estimate roll, pitch, yaw
%  Technique: Linear Kalman Filter (KF)

clear;

%% Load gyro data
load('gyrodata.mat');

%% Describe system dynamics in canonical state space form
%  x[n+1] = Ax[n] + Bu[n]
%  y[n]   = Cx[n]

%  Define the sampling time of the system
dt = 0.01; % corresponding to fs = 100 Hz

%  State transition matrix A
A = [ [1, dt]; [0, 1] ];
B = zeros(2,2);
C = [0, 1];
D = 0;

%% KF Design
Q = 0.01 * eye(2);
R = 0.05;
x0 = [0 0]';
P0 = 0.1 * eye(2);

% Call kalman filter
kf = dsp.KalmanFilter('StateTransitionMatrix', A,...
    'MeasurementMatrix', C,...
    'ProcessNoiseCovariance', Q, ...
    'MeasurementNoiseCovariance', R, ...
    'InitialStateEstimate', x0, ...
    'InitialErrorCovarianceEstimate', P0, ...
    'ControlInputPort', false);

%% Filter the data using the kf object
%  Yaw estimation
yaw_signal = gyroRotationZ(2:end);
yaw_estimate = step(kf, yaw_signal);

%  Roll estimation
roll_signal = gyroRotationX(2:end);
roll_estimate = step(kf, roll_signal);

%  Pitch estimation
pitch_signal = gyroRotationY(2:end);
pitch_estimate = step(kf, pitch_signal);

%% Plotting
figure; subplot(3, 1, 1);
plot(yaw_signal, 'b.'); 
hold on;
plot(yaw_estimate, 'k');
grid; 
legend('Raw', 'KF');
title('Yaw estimation (no bias correction)')
ylabel('Yaw in degrees');
xlabel('Index')

subplot(3, 1, 2); 
plot(roll_signal, 'b.'); 
hold on;
plot(roll_estimate, 'k');
grid; 
legend('Raw', 'KF');
title('Roll estimation (no bias correction)')
ylabel('Roll in degrees');
xlabel('Index')

subplot(3, 1, 3); 
plot(pitch_signal, 'b.'); 
hold on;
plot(pitch_estimate, 'k');
grid; 
legend('Raw', 'KF');
title('Pitch estimation (no bias correction)')
ylabel('Pitch in degrees');
xlabel('Index')