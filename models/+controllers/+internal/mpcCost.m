function J = mpcCost(u, T_0, Q_h, T_a, u_prev, C_th, R_th, dt, H_p, T_set, w_T, w_E, w_dU)
%MPCCOST  MPC cost function, identical to the one defined at the bottom of
%   scripts/Dynamic_Loads.m (Section 4.9 of the project README). Shared by
%   both the MATLAB-only MPC benchmark and controllers.MPCCoolingController
%   so the two solve the same optimization problem.

T_pred = zeros(H_p + 1, 1);
T_pred(1) = T_0;
J = 0;

for k = 1:H_p
    Q_pas = (T_pred(k) - T_a(k)) / R_th;
    dT = (Q_h(k) - Q_pas - u(k)) / C_th;
    T_pred(k+1) = T_pred(k) + dT * dt;

    e_T = T_pred(k+1) - T_set;

    if k == 1
        du = u(k) - u_prev;
    else
        du = u(k) - u(k-1);
    end

    J = J + w_T * (e_T^2) + w_E * (u(k)^2) + w_dU * (du^2);
end
end
