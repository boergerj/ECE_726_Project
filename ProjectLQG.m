clc; clear; close all;

%% Plant Parameters
m0 = 1.5; % Mass of cart
m1 = .75; % Mass of first linkage
m2 = .5; % Mass of second linkage

l1 = .75; % Length of linkage 1
l2 = .5; % Length of linkage 2

%% Linearized Plant

g = 9.8;

M_0 = [...
    m0+m1+m2 (m1/2+m2)*l1 m2*l2/2;
    (m1/2+m2)*l1 (m1/3+m2)*l1^2 m2*l1*l2/2;
    m2*l2/2 m2*l1*l2/2 (m2*l2^2)/3;
    ];

pG_0 = [...
    0 0 0;
    0 -((.5*m1)+m2)*l1*g 0;
    0 0 -.5*m2*l2*g;
    ];

H = [1 0 0]';

A = [...
    zeros(3) eye(3);
    -inv(M_0)*pG_0 zeros(3);
    ];

B = [...
    zeros(3,1);
    inv(M_0)*H;
    ];

C = eye(6);

%% LQG

% Cost matricies for plant response
Q = diag([70;100;100;30;1;1]);
R = 1;

% Cost matricies for noise
W1 = eye(6)*5;
W2 = eye(6)*10;

F = lqr(A, B, Q, R);

S = lqr(A', C', W1, W2);

v1 = awgn(zeros(6,1), 2);
v2 = awgn(zeros(6,1), 2);

A_n = [...
    A    -B*F;
    S*C  A-B*F-S*C;
    ];

B_n = [...
    v1;
    S*v2;
    ];

C_n = eye(12);

ic = [0; deg2rad(20); deg2rad(40); 0; 0; 0; zeros(6,1)];

t = 0:.005:10;
lqgSys = ss(A_n, B_n, C_n, 0);
nRoof = 1;
nFloor = -1;
noise = nFloor+(nRoof-nFloor).*rand(length(t),1);
kalResponse = lsim(lqgSys, noise, t, ic);

figure
plot(t, real(kalResponse(:,1)))
hold on
plot(t, real(kalResponse(:,2)))
plot(t, real(kalResponse(:,3)))
xlabel('Time');
ylabel('Value');
title('LQR + LQG Response');
legend(["Cart Position" "Link1 Angle (rad)" "Link2 Angle (rad)"])

%% Plot
cartPlot(kalResponse, m0, m1, m2, l1, l2, false, false, 10);
