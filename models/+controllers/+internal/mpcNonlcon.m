function [c, ceq] = mpcNonlcon(u, T_0, Q_h, T_a, C_th, R_th, dt, H_p, T_max)
%MPCNONLCON  MPC hard safety constraint, identical to the one defined at
%   the bottom of scripts/Dynamic_Loads.m (Section 4.9 of the project
%   README): enforces T_pred <= T_max at every step of the horizon.

T_pred = zeros(H_p + 1, 1);
T_pred(1) = T_0;

for k = 1:H_p
    Q_pas = (T_pred(k) - T_a(k)) / R_th;
    dT = (Q_h(k) - Q_pas - u(k)) / C_th;
    T_pred(k+1) = T_pred(k) + dT * dt;
end

c = T_pred(2:end) - T_max;
ceq = [];
end
