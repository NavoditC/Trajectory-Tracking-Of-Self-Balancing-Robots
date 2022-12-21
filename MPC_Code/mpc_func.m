function [voltage,x_updated] = mpc_func(nlobj,distance,x)
% Write your mpc code here. Since this loop will be called at
% every time step, try to keep it minimal

EKF = extendedKalmanFilter(@tumbllerStateFcn, @tumbllerMeasurementFcn);
y = distance;
EKF.State = x;
mv=0;
% Set reference for distance and angle
yref = 0.2;
% Correct previous prediction using current measurement.
xk = correct(EKF, y);
% Compute optimal control moves.
[mv,~,~] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
% Predict prediction model states for the next iteration.
predict(EKF, [mv; Ts]);
% Implement first optimal control move and update plant states.
x_updated = tumbllerDT0(x,mv,Ts);

voltage = mv;

end