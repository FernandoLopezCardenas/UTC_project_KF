% Keep things clean
close; clear; clc;

% load the data previously extracted by the csv file 
% (taken values of x and y from index 100 to 820 and 
%   changed the sign to y)

load("Marconi_computed_path.mat"); 

% plot the circuit (pov from the ground station)
plot(x, y);



%% To extract the data from the .csv
% x -> AF column
% y -> -(AH colum)
load("test1_newDef.mat"); 
hold on;
plot(x_test, y_test);


% For a better plot
grid on; % put the grid
xlabel('X [m]'); ylabel('Y [m]'); % axis labels
legend('Marconi Ref.', 'Test Traj.') % put the legend
set(gcf, 'Position', [100, 100, 789, 958]);% set the dimensio nof the image