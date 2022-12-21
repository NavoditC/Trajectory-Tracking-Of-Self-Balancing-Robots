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

%% CT State space system about upright position
v_to_force = (2*kT)/(R*r);
A = [0,                              1,                                                           0,                                                            0;
     0, -(I_pend+m_pend*l*l)*f/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),     (m_pend^2*g*l^2)/(I_pend*(m_cart+m_pend)+m_cart*m_pend*l^2),                   0;
     0,                              0,                                                           0,                                                            1;
     0, -m_pend*l*f/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),             (m_pend*g*l*(m_cart+m_pend))/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)),        0];
 
B =  v_to_force*[               0;
     (I_pend+m_pend*l^2)/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2)); 
                               0;
     (m_pend*l)/(I_pend*(m_cart+m_pend)+(m_cart*m_pend*l^2))];
C = eye(4);
D = zeros(4,1);

sysc = ss(A,B,C,D);
%% DT State space system about upright position
Ts = 0.005;
sysd = c2d(sysc,Ts);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% Finding closed loop transfer function of inner loop responsible for balancing the tumbller
[b,a] = ss2tf(Ad,Bd,Cd,Dd);
sys1 = tf(b(1,:),a(1,:),Ts);
sys2 = tf(b(2,:),a(1,:),Ts);
sys3 = tf(b(3,:),a(1,:),Ts);
sys4 = tf(b(4,:),a(1,:),Ts);
G = [sys1;sys2;sys3;sys4];
H = [0 -10 -65 -0.75]; %Gains of the inner loop controller responsbile for balancing the tumbller
tf_closed_loop = feedback(G,H,1);
b = [1.381e-05,-1.387e-05,-1.387e-05,1.381e-05; %Numerator Coefficients
    0.005526,-0.0166,0.0166,-0.005526;
    0.0002333,-0.0002333,-0.0002333,0.0002333;
    0.09336,-0.2801,0.2801,-0.09336];
a = [1,-3.865,5.62,-3.644,0.8898]; %Denominator Coefficients
[A_cl,B_cl,C_cl,D_cl] = tf2ss(b,a); %The found A_cl,B_cl,C_cl,D_cl are used in tumbllerDT0.m