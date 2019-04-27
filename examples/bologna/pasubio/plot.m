clear;
clc;

figure('OuterPosition', [100 80 680 660]);
M = csvread('csvs\deployStatistics_12_10_12.csv');
T = M(:, 1);
R = M(:, 2);
O2 = csvread('csvs\deployStatistics_optimal.csv');
O = O2(:, 2);
VF = csvread('csvs\deployStatistics_vf.csv');
C = VF(:, 2);

S8 = csvread('csvs\deployStatistics_8_10_12.csv');
S9 = csvread('csvs\deployStatistics_9_10_12.csv');
S18 = csvread('csvs\deployStatistics_18_10_12.csv');

h = plot(T, S8(:,2), 'b', T, S9(:,2), 'g', T, R, 'r', T, S18(:,2), 'c', T, C, 'y', T, O, 'm');
legend('sn=8', 'sn=9', 'sn=12', 'sn=18', 'comparison', 'optimal', 'Location', 'NorthWest');

% V5 = csvread('csvs\deployStatistics_12_5_12.csv');
% V15 = csvread('csvs\deployStatistics_12_15_12.csv');
% V20 = csvread('csvs\deployStatistics_12_20_12.csv');
% 
% h = plot(T, V5(:,2), 'b', T, R, 'g', T, V15(:,2), 'r', T, V20(:,2), 'c', T, C, 'y', T, O, 'm');
% legend('v=5m/s', 'v=10m/s', 'v=15m/s', 'v=20m/s', 'comparison', 'optimal', 'Location', 'NorthWest');

% P6 = csvread('csvs\deployStatistics_12_10_6.csv');
% P18 = csvread('csvs\deployStatistics_12_10_18.csv');
% P24 = csvread('csvs\deployStatistics_12_10_24.csv');
% 
% h = plot(T, P6(:,2), 'b', T, R, 'g', T, P18(:,2), 'r', T, P24(:,2), 'c', T, C, 'y', T, O, 'm');
% legend('\itT_3\rm=6s', '\itT_3\rm=12s', '\itT_3\rm=18s', '\itT_3\rm=24s', 'comparison', 'optimal', 'Location', 'NorthWest');

legend('boxoff');
xlim([180 780]);
xlabel('Time (s)', 'FontSize', 13);
ylabel('Covered vehicles', 'FontSize', 13);
title('Impact of sector number on objective function', 'FontSize', 13);
% title('Impact of UAV velocity on objective function', 'FontSize', 13);
% title('Impact of decision period on objective function', 'FontSize', 13);
set(gca, 'FontSize', 13);
