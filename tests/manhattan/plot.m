clear;
clc;

DV = csvread('SimuII\routingStatistics_DV_TD.csv');
DA = csvread('SimuII\routingStatistics_DA_TD.csv');
% DV = csvread('SimuII\routingStatistics_DV_RS.csv');
% DA = csvread('SimuII\routingStatistics_DA_RS.csv');
% DV = csvread('SimuII\routingStatistics_DV_SR.csv');
% DA = csvread('SimuII\routingStatistics_DA_SR.csv');

D = DV(:, 1);
S = DV(:, 2);
R = DV(:, 4);
pktSentDV = DV(:, 5);
pktRecvDV = DV(:, 6);
deliveryDV = pktRecvDV ./ pktSentDV;
delayDV = 1000 * DV(:, 7);
lowerLostDV = DV(:, 8);
overLostDV = DV(:, 9);
overloadDV = DV(:, 10);
completeDV = DV(:, 11) / 100;
pktSentDA = DA(:, 5);
pktRecvDA = DA(:, 6);
deliveryDA = pktRecvDA ./ pktSentDA;
delayDA = 1000 * DA(:, 7);
lowerLostDA = DA(:, 8);
overLostDA = DA(:, 9);
overloadDA = DA(:, 10);
completeDA = DA(:, 11) / 100;

figure('OuterPosition', [100 80 1000 660]);
% --------------------------------------------------
subplot(2, 3, 1);
h1 = plot(D, completeDV, 'b', D, completeDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Completed Ratio', 'FontSize', 12);

subplot(2, 3, 2);
h2 = plot(D, deliveryDV, 'b', D, deliveryDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
ylim([0.7 0.9]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Delivery Ratio', 'FontSize', 12);

subplot(2, 3, 3);
h3 = plot(D, delayDV, 'b', D, delayDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Average Delay (ms)', 'FontSize', 12);

subplot(2, 3, 4);
h4 = plot(D, lowerLostDV, 'b', D, lowerLostDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Lower layer Lost', 'FontSize', 12);

subplot(2, 3, 5);
h5 = plot(D, overLostDV, 'b', D, overLostDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Overflow Lost', 'FontSize', 12);

subplot(2, 3, 6);
h6 = plot(D, overloadDV, 'b', D, overloadDA, 'g--');
legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
legend('boxoff');
xlim([360 960]);
xlabel('Traffic Density (vph)', 'FontSize', 12);
ylabel('Control Overload', 'FontSize', 12);

% --------------------------------------------------
% subplot(2, 3, 1);
% h1 = plot(S, completeDV, 'b', S, completeDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([30 90]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Completed Ratio', 'FontSize', 12);
% 
% subplot(2, 3, 2);
% h2 = plot(S, deliveryDV, 'b', S, deliveryDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([30 90]);
% ylim([0.6 1]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Delivery Ratio', 'FontSize', 12);
% 
% subplot(2, 3, 3);
% h3 = plot(S, delayDV, 'b', S, delayDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([30 90]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Average Delay (ms)', 'FontSize', 12);
% 
% subplot(2, 3, 4);
% h4 = plot(S, lowerLostDV, 'b', S, lowerLostDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([30 90]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Lower layer Lost', 'FontSize', 12);
% 
% subplot(2, 3, 5);
% h5 = plot(S, overLostDV, 'b', S, overLostDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([30 90]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Overflow Lost', 'FontSize', 12);
% 
% subplot(2, 3, 6);
% h6 = plot(S, overloadDV, 'b', S, overloadDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([30 90]);
% xlabel('Road Speed Limit (km/h)', 'FontSize', 12);
% ylabel('Control Overload', 'FontSize', 12);

% --------------------------------------------------
% subplot(2, 3, 1);
% h1 = plot(R, completeDV, 'b', R, completeDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([25 100]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Completed Ratio', 'FontSize', 12);
% 
% subplot(2, 3, 2);
% h2 = plot(R, deliveryDV, 'b', R, deliveryDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthEast');
% legend('boxoff');
% xlim([25 100]);
% ylim([0.6 1]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Delivery Ratio', 'FontSize', 12);
% 
% subplot(2, 3, 3);
% h3 = plot(R, delayDV, 'b', R, delayDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([25 100]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Average Delay (ms)', 'FontSize', 12);
% 
% subplot(2, 3, 4);
% h4 = plot(R, lowerLostDV, 'b', R, lowerLostDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([25 100]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Lower layer Lost', 'FontSize', 12);
% 
% subplot(2, 3, 5);
% h5 = plot(R, overLostDV, 'b', R, overLostDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([25 100]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Overflow Lost', 'FontSize', 12);
% 
% subplot(2, 3, 6);
% h6 = plot(R, overloadDV, 'b', R, overloadDA, 'g--');
% legend('AOMDV', 'PAOMDA', 'Location', 'NorthWest');
% legend('boxoff');
% xlim([25 100]);
% xlabel('Send Rate (pkt/s)', 'FontSize', 12);
% ylabel('Control Overload', 'FontSize', 12);