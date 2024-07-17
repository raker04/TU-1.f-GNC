clear; clc; close all;

data = xlsread('gyro_signal_data.xls');

Ts = 0.009994; % sec
f_cutoff = 0.5; % Hz
tau = 1 / 2/pi / f_cutoff; % time constant
alpha = 0.2;

N = length(data);
N_sample = 50;

t = data(:, 1); % sec
gyro_x = data(:, 2); % rad/s
gyro_y = data(:, 3); % rad/s
gyro_z = data(:, 4); % rad/s

filtered_gyro_x = zeros(1, N);
filtered_gyro_y = zeros(1, N);
filtered_gyro_z = zeros(1, N);
diff_gyro_x = zeros(1, N);
diff_gyro_y = zeros(1, N);
diff_gyro_z = zeros(1, N);
temp_t = zeros(1,N_sample);
temp_t2 = zeros(1,N_sample);
temp_one = ones(1,N_sample);
temp_x = zeros(1,N_sample);
temp_y = zeros(1,N_sample);
temp_z = zeros(1,N_sample);
temp_t(1) = -2*t(1);
temp_t(2) = -1*t(1);
temp_t2(1) = 4*t(1)^2;
temp_t2(2) = 1*t(1)^2;


for i = 1:1:N
    for j = 1:N_sample-1
        temp_t(j) = temp_t(j+1);
        temp_t2(j) = temp_t2(j+1);
        temp_x(j) = temp_x(j+1);
        temp_y(j) = temp_y(j+1);
        temp_z(j) = temp_z(j+1);
    end
    temp_t(N_sample) = t(i);
    temp_t2(N_sample) = t(i)^2;
    temp_x(N_sample) = gyro_x(i);
    temp_y(N_sample) = gyro_y(i);
    temp_z(N_sample) = gyro_z(i);
    A = [temp_t2',temp_t',temp_one'];
    B = [temp_x',temp_y',temp_z'];
    X = inv(A'*A)*A'*B;
    A2 = [temp_t',temp_one'];
    X2 = inv(A2'*A2)*A2'*B;
    diff_gyro_x(i) = X2(1,1);
    diff_gyro_y(i) = X2(1,2);
    diff_gyro_z(i) = X2(1,3);
    if i == 1
        filtered_gyro_x(i) = X(1,1)*t(i)*t(i)+X(2,1)*t(i)+X(3,1);
        filtered_gyro_y(i) = X(1,2)*t(i)*t(i)+X(2,2)*t(i)+X(3,2);
        filtered_gyro_z(i) = X(1,3)*t(i)*t(i)+X(2,3)*t(i)+X(3,3);
    else
        filtered_gyro_x(i) = (X(1,1)*t(i)*t(i)+X(2,1)*t(i)+X(3,1))*(1-alpha)+filtered_gyro_x(i-1)*alpha;
        filtered_gyro_y(i) = (X(1,2)*t(i)*t(i)+X(2,2)*t(i)+X(3,2))*(1-alpha)+filtered_gyro_y(i-1)*alpha;
        filtered_gyro_z(i) = (X(1,3)*t(i)*t(i)+X(2,3)*t(i)+X(3,3))*(1-alpha)+filtered_gyro_z(i-1)*alpha;
    end
end

figure;
subplot(3, 1, 1);
hold on
plot(t, gyro_x, 'DisplayName', 'gyro_x')
plot(t, filtered_gyro_x, 'DisplayName', 'filtered_gyro_x')
plot(t, diff_gyro_x, 'DisplayName', 'diff_gyro_x')
legend show

subplot(3, 1, 2);
hold on
plot(t, gyro_y, 'DisplayName', 'gyro_y')
plot(t, filtered_gyro_y, 'DisplayName', 'filtered_gyro_y')
plot(t, diff_gyro_y, 'DisplayName', 'diff_gyro_y')
legend show

subplot(3, 1, 3);
hold on
plot(t, gyro_z, 'DisplayName', 'gyro_z')
plot(t, filtered_gyro_z, 'DisplayName', 'filtered_gyro_z')
plot(t, diff_gyro_y, 'DisplayName', 'diff_gyro_y')
legend show
