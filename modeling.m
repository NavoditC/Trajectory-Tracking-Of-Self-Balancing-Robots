
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

q = (m_cart+m_pend)*(I_pend+(m_pend*l*l))-(m_pend^2*l^2);
% [num,den] = ss2tf(A,B,C,D);

% s = tf('s');
% sys = 28.4085*s^2/(s^4+0.0168*s^3-224.3431*s^2-2.7869*s);



%% LQR Controller
Q_lqr = diag([1000,0,400,0]);
R_lqr = 1;
[K,S,P] = lqr(A,B,Q_lqr,R_lqr);
%K = [K(1) K(3)];
K = K/v_to_force;

%% MPC_Tracking Horizons
