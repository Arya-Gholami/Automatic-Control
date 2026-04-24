
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
