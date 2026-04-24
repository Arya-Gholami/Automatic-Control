%% Problem Description
% Given the LTI system:
%
%   x_dot(t) = [  0    1 ] x(t) + [ 0 ] u(t)
%              [ -2   -3 ]         [ 1 ]
%
% Design a state feedback control law of the form:
%
%       u(t) = -K * x(t)
%
% for a zero-state regulation problem with:
%
%   - Initial condition: x(0) = [1; 1]
%   - Regulation tolerance: 1e-4
%   - Desired regulation time: t_reg ≈ 0.5 s (±5%)
%--------------------------------------------------------------------------
%%
clear;
close all;
clc;

%% State Matrices and Initial Conditions
A = [0, 1; -2, -3];
B = [0; 1];
C = [1, 1];
D = 0;
x0 = [1; 1];

%% Reachability 
Mr = ctrb(A, B);
rho = rank(Mr);
if rho < size(A,1)
    error('System is NOT reachable.')
end

%% Requirements 
x_tol = 1e-4;
t_reg = 0.5;
t_tol = 0.05; % =5%
x0_norm = sqrt(x0(1,1)^2 + x0(2)^2);

%% Maximum possible eigenvalue 
lambda_max = log(x_tol/x0_norm) / t_reg;
lambda1 = 1.3*lambda_max;
lambda2 = 100*lambda1;
lambda_des = [lambda1, lambda2];

%% State gain
K = place(A, B, lambda_des);
sys_cont = ss(A-B*K, B, C, D);

%% Simulation
t_sim = linspace(0, 0.6, 10000);
[y, t, x] = initial(sys_cont, x0, t_sim);
xnorm = sqrt(x(:, 1).^2 + x(:, 2).^2);

%% Plots
plot(t, xnorm, 'lineWidth', 2);
xlabel('t')
ylabel('||x(t)||_2')
yline(1e-4, 'r')
yline(-1e-4, 'r')
xline(0.95*0.5, 'g')
xline(1.05*0.5, 'g')
grid on
title('State Regulation with Static Feedback')
