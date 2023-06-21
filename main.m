clear;clc;close all
robotSolver = hoverRobot;
robotSolver.tspan = linspace(0,10,101);
robotSolver.x0 = [0.5 0.5];
solution = robotSolver.simulation();

time = solution.x;
state = solution.y;

% figure
% subplot(2,1,1)
% plot(time, state)
% xlabel('Time, t [sec]')
% legend('Depth','Velocity')
% subplot(2,1,2)
% plot(state(1,:),state(2,:))
% xlabel('Depth, d [m]')
% ylabel('Velocity, v [m/s]')
% axis equal
robotSolver.plotResults(time,state)