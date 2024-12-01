clc; clear; close all;

%% Plant Parameters
m0 = 2; % Mass of cart
m1 = .75; % Mass of first linkage
m2 = 1; % Mass of second linkage

l1 = .75; % Length of linkage 1
l2 = 1; % Length of linkage 2

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

%% LQR

Q = eye(6);

R = 1;

[K] = lqr(A, B, Q, R);

ic = [0; deg2rad(90); deg2rad(0); 0; 0; 0];

t = 0:.005:10;
lqgSys = ss(A-B*K, B, C, 0);
kalResponse = initial(lqgSys, ic, t);

figure
plot(t, real(kalResponse(:,1)))
hold on
plot(t, real(kalResponse(:,2)))
plot(t, real(kalResponse(:,3)))
xlabel('Time');
ylabel('Position');

%% Plot
cartPlot(kalResponse, m0, m1, m2, l1, l2, false, false, 10);
