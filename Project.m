clc; clear; close all;

%% Plant Parameters
m0 = 1.5; % Mass of cart
m1 = .5; % Mass of first linkage
m2 = .75; % Mass of second linkage

l1 = .5; % Length of linkage 1
l2 = .75; % Length of linkage 2

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

Q = eye(6);

R = 1;

% Weight matricies for noise
W1 = eye(6);
W2 = eye(6)*2;

v1 = awgn(zeros(6), 1);
v2 = awgn(zeros(6), 1);

[~, K] = lqr(A, B, Q, R);
F = inv(R)*B'*K;

S = lqr(A', C', W1, W2);

A_n = [...
    A    -B*F;
    S*C  A-B*F-S*C;
    ];

B_n = [...
    v1;
    S*v2;
    ];

C_n = eye(12);

ic = [0; deg2rad(10); deg2rad(-10); 0; 0; 0; zeros(6,1)];

t = 0:.1:10;
lqgSys = ss(A_n, B_n, C_n, zeros(12,6));
kalResponse = initial(lqgSys, ic, t);

figure
plot(t, real(kalResponse(:,1)))
hold on
plot(t, real(kalResponse(:,2)))
plot(t, real(kalResponse(:,3)))
xlabel('Time');
ylabel('Position');
