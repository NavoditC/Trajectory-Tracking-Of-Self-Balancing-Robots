%% Parameter values in SI units
m_cart = 0.493;
m_pend = 0.312;
I_pend = 0.00024;
l = 0.04;
f = 0.01;
kT = 0.11;
R = 10;
r = 0.0335;
g = 9.81;
%% State space system about upright position
v_to_force = (2*kT)/(R*r);
A = [0,                              1,                                                           0,                                                            0;
     0, -(I_pend+m_pend*l*l)*f/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),     (m_pend^2*g*l^2)/(I_pend*(m_cart+m_pend)+m_cart*m_pend*l^2),                   0;
     0,                              0,                                                           0,                                                            1;
     0, -m_pend*l*f/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),             (m_pend*g*l*(m_cart+m_pend))/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),        0];
 
B =  [               0;
     (I_pend+m_pend*l^2)/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)); 
                               0;
     (m_pend*l)/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2))];

C = eye(4);

D = zeros(4,1);
%%
nx=4;
ny=1;
nu=1;
nlobj = nlmpc(nx,ny,nu);
Ts = 0.005;
nlobj.Ts=Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

nlobj.Model.StateFcn = "tumbllerDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = 'tumbllerOutputFcn';
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0];

nlobj.Weights.OutputVariables = 1000;
nlobj.Weights.ManipulatedVariablesRate = 1;
nlobj.OV(1).Min = -Inf;
nlobj.OV(1).Max = Inf;
nlobj.MV.Min = -Inf;
nlobj.MV.Max = Inf;

EKF = extendedKalmanFilter(@tumbllerStateFcn, @tumbllerMeasurementFcn);
x = [0;0;-pi;0];
y = 0.2 + randn(1,1)*0.01;
EKF.State = x;

mv=0;

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

Duration = 40;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
% Set reference
yref = 0.2;
for ct = 1:(Duration/Ts)
    % Correct previous prediction using current measurement.
    xk = correct(EKF, y);
    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
    % Predict prediction model states for the next iteration.
    predict(EKF, [mv; Ts]);
    % Implement first optimal control move and update plant states.
    x = tumbllerDT0(x,mv,Ts);
    % Distance measurement from ultrasonic sensor
    y = 0.2 + randn(1,1)*0.01; 
    % Save plant states for display.
    xHistory = [xHistory x]; 
    waitbar(ct*Ts/Duration,hbar);
end
close(hbar)

figure
subplot(2,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time(s)')
ylabel('z(m)')
title('Tumbller position')

subplot(2,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time(s)')
ylabel('zdot(m/s)')
title('Tumbller velocity')

subplot(2,2,3)
plot(0:Ts:Duration,xHistory(3,:))
xlabel('time(s)')
ylabel('theta(rad)')
title('Inclination from vertical')

subplot(2,2,4)
plot(0:Ts:Duration,xHistory(4,:))
xlabel('time(s)')
ylabel('thetadot(rad/s)')
title('Angular rate of motion from vertical')

% Reference: https://www.mathworks.com/help/mpc/ug/swing-up-control-of-a-pendulum-using-nonlinear-model-predictive-control.html
