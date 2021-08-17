
close all;

data = trajread('../../results/results.json');
TimeStep = data(1,:);
TimeStep = TimeStep./4;
inTemp = data(2,:);
outTemp = data(3,:);
rtp = data(4,:);
refTemp = data(5,:);
dollarCost = data(6,:);
power = data(7,:);

figure(1)
hold on
set(gca, 'Fontsize', 22);
grid on
plot(TimeStep, inTemp, 'LineWidth', 5);
plot(TimeStep, outTemp, 'LineWidth', 5);
plot(TimeStep, refTemp, 'LineWidth', 5);
title('Temperature Profiles','interpreter','latex')
xlabel('Hours','interpreter','latex')
ylabel('Temp [$^{\circ}$C]','interpreter','latex')
legend('Internal Temperature', 'External Temperature', 'Reference Temperature')
hold off

figure(2)
hold on
grid on
set(gca, 'Fontsize', 22);
plot(TimeStep, power, 'LineWidth', 5)
% plot(rtp, TimeStep)
title('Power Consumption with RTP Signal','interpreter','latex')
xlabel('Hours','interpreter','latex')
ylabel('Power [kW/h]','interpreter','latex')
hold off

figure(3)
hold on
grid on
set(gca, 'Fontsize', 22);
plot(TimeStep, dollarCost, 'LineWidth', 5)
title('Dollar Cost of Running HVAC','interpreter','latex')
xlabel('Hours','interpreter','latex')
ylabel('USD','interpreter','latex')
hold off

cumsum(dollarCost)
