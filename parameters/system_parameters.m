%% ========= System Parameters =====================

clear all; close all; clc;

%these parameters are used in many textbooks 
M=0.5; %masse of cart (Kg)
m=0.2; %mass of pendulum bob (Kg)
l=0.1; %length of pendulum bob (m)
lc=l/2;%distance from pivot to center of mass = l/2
J = (1/12)*m*l^2;  % moment of inertia of uniform rod about its center
g = 9.81;    % gravitational acceleration (m/s²)
