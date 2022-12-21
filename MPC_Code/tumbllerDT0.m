function xk1 = tumbllerDT0(xk, uk, Ts)
%% Discrete-time nonlinear dynamic model of the tumbller found after simplifying the inner loop for balancing (Refer ss_mpc.m for reference)
A = [3.9200 -5.7860 3.8100 -0.9451;1 0 0 0;0 1 0 0;0 0 1 0];
B = [1;0;0;0];
xk1 = A*xk+B*uk;
