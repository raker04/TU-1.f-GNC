clear; clc; close all;

data = xlsread('gyro_signal_data.xls');

Ts = 0.009994; % sec
f_cutoff = 0.5; % Hz
tau = 1 / 2/pi / f_cutoff; % time constant
alpha = Ts / (tau + Ts)

N = length(data);

t = data(:, 1); % sec
gyro_x = data(:, 2); % rad/s
gyro_y = data(:, 3); % rad/s
gyro_z = data(:, 4); % rad/s

filtered_gyro_x = zeros(1, N);
filtered_gyro_y = zeros(1, N);
filtered_gyro_z = zeros(1, N);
filtered_gyro_x(1) = gyro_x(1);
filtered_gyro_y(1) = gyro_y(1);
filtered_gyro_z(1) = gyro_z(1);

for i = 2:1:N
    filtered_gyro_x(i) = alpha * gyro_x(i) + (1 - alpha) * filtered_gyro_x(i-1);
    filtered_gyro_y(i) = alpha * gyro_y(i) + (1 - alpha) * filtered_gyro_y(i-1);
    filtered_gyro_z(i) = alpha * gyro_z(i) + (1 - alpha) * filtered_gyro_z(i-1);
end

figure;
subplot(3, 1, 1);
hold on
plot(t, gyro_x, 'DisplayName', 'gyro_x')
plot(t, filtered_gyro_x, 'DisplayName', 'filtered_gyro_x')
legend show

subplot(3, 1, 2);
hold on
plot(t, gyro_y, 'DisplayName', 'gyro_y')
plot(t, filtered_gyro_y, 'DisplayName', 'filtered_gyro_y')
legend show

subplot(3, 1, 3);
hold on
plot(t, gyro_z, 'DisplayName', 'gyro_z')
plot(t, filtered_gyro_z, 'DisplayName', 'filtered_gyro_z')
legend show