run ImportData.m

% Data is stored in the following format:
% void printData() {
%   Serial.print(millis()/float(1000));
%   Serial.print(", ");
%   Serial.print(deltaT, 4);
%   Serial.print(", ");
%   Serial.print(theta_0);
%   Serial.print(", ");
%   Serial.print(theta);
%   Serial.print(", ");
%   Serial.print(theta_dot_0);
%   Serial.print(", ");
%   Serial.print(theta_dot);
%   Serial.print(", ");
%   Serial.println(phi);
% }

D_gains = [0 2 4 6 8];

%% analysis
P0_bigarray = MakeBigArray(P0);
P2_bigarray = MakeBigArray(P2);
P4_bigarray = MakeBigArray(P4);
P6_bigarray = MakeBigArray(P6);
P8_bigarray = MakeBigArray(P8);

% mean theta
mean_theta(1) = mean(P0_bigarray(:,4));
mean_theta(2) = mean(P2_bigarray(:,4));
mean_theta(3) = mean(P4_bigarray(:,4));
mean_theta(4) = mean(P6_bigarray(:,4));
mean_theta(5) = mean(P8_bigarray(:,4));

% sd theta
std_theta(1) = std(P0_bigarray(:,4));
std_theta(2) = std(P2_bigarray(:,4));
std_theta(3) = std(P4_bigarray(:,4));
std_theta(4) = std(P6_bigarray(:,4));
std_theta(5) = std(P8_bigarray(:,4));

% mean theta dot
mean_theta_dot(1) = mean(P0_bigarray(:,6));
mean_theta_dot(2) = mean(P2_bigarray(:,6));
mean_theta_dot(3) = mean(P4_bigarray(:,6));
mean_theta_dot(4) = mean(P6_bigarray(:,6));
mean_theta_dot(5) = mean(P8_bigarray(:,6));

% sd theta dot
std_theta_dot(1) = std(P0_bigarray(:,6));
std_theta_dot(2) = std(P2_bigarray(:,6));
std_theta_dot(3) = std(P4_bigarray(:,6));
std_theta_dot(4) = std(P6_bigarray(:,6));
std_theta_dot(5) = std(P8_bigarray(:,6));

%% ttests
[h, p] = ttest2(P0_bigarray(:,4), P2_bigarray(:,4));
ttest_theta(1) = p;
[h, p] = ttest2(P0_bigarray(:,4), P4_bigarray(:,4));
ttest_theta(2) = p;
[h, p] = ttest2(P0_bigarray(:,4), P6_bigarray(:,4));
ttest_theta(3) = p;
[h, p] = ttest2(P0_bigarray(:,4), P8_bigarray(:,4));
ttest_theta(4) = p;

[h, p] = ttest2(P0_bigarray(:,6), P2_bigarray(:,6));
ttest_theta_dot(1) = p;
[h, p] = ttest2(P0_bigarray(:,6), P4_bigarray(:,6));
ttest_theta_dot(2) = p;
[h, p] = ttest2(P0_bigarray(:,6), P6_bigarray(:,6));
ttest_theta_dot(3) = p;
[h, p] = ttest2(P0_bigarray(:,6), P8_bigarray(:,6));
ttest_theta_dot(4) = p;

%% plotting
figure(1)
set(0,'DefaultAxesFontSize',25)
errorbar(D_gains, mean_theta, std_theta, 'LineWidth', 3)
grid on
xlabel('Derivative gain')
ylabel('\theta (degrees)')
title('Mean and standard deviation of \theta for various derivative gains')
subtitle('Two-sample t-tests compared against D=0')
text(D_gains(2), mean_theta(2) + std_theta(2), ['p < ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(3), mean_theta(3) + std_theta(3), ['p < ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(4), mean_theta(4) + std_theta(4), ['p = ', ...
    num2str(ttest_theta(3))], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(5), mean_theta(5) + std_theta(5), ['p < ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')

figure(2)
set(0,'DefaultAxesFontSize',25)
errorbar(D_gains, mean_theta_dot, std_theta_dot, 'LineWidth', 3)
grid on
xlabel('Derivative gain')
ylabel('\theta rate (degrees/second)')
title('Mean and standard deviation of \theta rate for various derivative gains')
subtitle('Two-sample t-tests compared against D=0')
text(D_gains(2), mean_theta_dot(2) + std_theta_dot(2), ['p = ', ...
    num2str( ttest_theta_dot(1) )], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(3), mean_theta_dot(3) + std_theta_dot(3), ['p < ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(4), mean_theta_dot(4) + std_theta_dot(4), ['p = ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')
text(D_gains(5), mean_theta_dot(5) + std_theta_dot(5), ['p < ', ...
    num2str( 1e-9 )], 'FontSize', 25, ...
    'HorizontalAlignment','center')

%% functions
function MakeTestPlot(TestArray)
x = 1:length(TestArray);

figure('DefaultAxesFontSize', 25)
plot(x, TestArray(:,4), ...
    x, TestArray(:,6), ...
    x, TestArray(:,7), ...
    [x(1,1) x(end,1)], [45 45], 'k--', ...
    [x(1,1) x(end,1)], -[45 45], 'k--', ...
    'LineWidth', 2)
legend('$\theta$', '$\dot{\theta}$', '$\phi$', '45', '-45', ...
    'Interpreter', 'latex')
grid on
end

function BigArray = MakeBigArray(CellArray)
BigArray = [];
for i = 1:10
    BigArray = [BigArray; CellArray{i}];
end
end
