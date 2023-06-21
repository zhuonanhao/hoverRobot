clear;clc;close all

robotSolver = hoverRobot;
robotSolver.tspan = 0:.1:10;
robotSolver.x0 = [.5 0];
solution = robotSolver.simulation();

time = solution.x;
state = solution.y;

robotSolver.plotResults(time,state)