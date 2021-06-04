clear, clc, close all

P0 = MakeCell(0);
P2 = MakeCell(2);
P4 = MakeCell(4);
P6 = MakeCell(6);
P8 = MakeCell(8);


TestArray = P4{3};
MakeTestPlot(TestArray)

function MakeTestPlot(TestArray)
x = 1:length(TestArray);

figure('DefaultAxesFontSize', 25)
plot(TestArray(:,1), TestArray(:,4), ...
    TestArray(:,1), TestArray(:,6), ...
    TestArray(:,1), TestArray(:,7), ...
    [TestArray(1,1) TestArray(end,1)], [45 45], 'k--', ...
    [TestArray(1,1) TestArray(end,1)], -[45 45], 'k--', ...
    'LineWidth', 2)
legend('$\theta$', '$\dot{\theta}$', '$\phi$', '45', '-45', ...
    'Interpreter', 'latex')
grid on
xlabel('t (s)')
title('Example of \theta, \theta rate, and servo response \phi')
end

function Array = TrimTime(Array)
Array(:,1) = Array(:,1) - Array(1,1);
end

function OutCell = MakeCell(D)
    for i = 1:10
        name = ['Data/D=', num2str(D), '/D=i_', num2str(i), '.txt'];
        OutCell{i} = table2array(readtable(name));
        OutCell{i} = TrimTime(OutCell{i});
        OutCell{i} = OutCell{i}(100:350,:);
    end
end