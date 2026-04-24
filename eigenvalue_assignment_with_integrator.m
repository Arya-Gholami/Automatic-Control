%% Problem Description
% Given the LTI system
%
%   x_dot(t) = [ -0.1   -1 ] x(t) + [ 1 ] u(t)
%              [  1     0 ]         [ 0 ]
%
%   y(t) = [ 0  1 ] x(t)
%
% Design a state feedback control law with integral action of the form
%
%   u(t) = -K_q * q(t) - K_x * x(t)
%
% for a setpoint output regulation problem, considering r(t) = ε(t),
% and the following transient requirements:
%
%   1) Maximum overshoot:       ŝ ≤ 10%
%   2) Settling time (2% band): t_s,2% ≤ 6.5 s
%--------------------------------------------------------------------------
clear;
close all;
clc;
A = [-0.1, -1; 1, 0];
B = [1; 0];
C =[0, 1];
D = 0;
% reachability
Mr = ctrb(A, B);
rho = rank(Mr);
if rho < size(A,1)
    error('System is NOT reachable. Controller design aborted.')
end
%% Transient requirements
s_hat = 0.1; % max overshoot
ts_2 = 6.5;  % settling time (2%)
% Damping Ratio and Natural Frequency
zeta = abs(log(s_hat)) / (sqrt(pi^2 + log(s_hat)^2));
wn = log(100/2) / (zeta * ts_2);
%% Desired Eigenvalues
lambda1 = -wn*zeta + 1j*wn*sqrt(1-zeta^2);
lambda2 = -wn*zeta - 1j*wn*sqrt(1-zeta^2);
lambda3 = -10* wn * zeta;
lambda_des = [lambda1, lambda2, lambda3];
%% control
% Augmented system for integral action
A_aug = [0, -C; zeros(2,1), A];
B_aug = [0; B];
C_aug = [0, C];
D_aug = D;
K_aug = place(A_aug, B_aug, lambda_des);
Kq = K_aug(1,1);
Kx = K_aug(1,2:end);
%--- control action with integrator system ---
% --> Closed-loop augmented system 
A_aug = [0, -C; -B*Kq, A-B*Kx];
B_aug = [1; zeros(2, 1)];
C_aug = C_aug;
D_aug = D_aug;
sys_cont = ss(A_aug, B_aug, C_aug, D_aug);
%%--- Simulation ---
t_sim = linspace(0, 10, 10000);
u = ones(size(t_sim,2), 1);
[y, t, x] = lsim(sys_cont, u,t_sim);
%%--- Plot Respone ---
figure(1)
plot(t, y, 'linewidth', 2)
yline(s_hat*dcgain(sys_cont) + dcgain(sys_cont), 'g'); % overshoot limit
yline(1.02*dcgain(sys_cont), 'r');                     % +2% band
yline(0.98*dcgain(sys_cont), 'r');                     % -2% band
xline(ts_2, 'g')                                       % settling time limit
xlabel('time')
ylabel('y(t)')
title('Closed-loop Response with Integral Action')
grid on
%%--- END ---

