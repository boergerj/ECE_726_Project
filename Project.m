clc; clear; close all;

%% Parameters
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
    0 ((.5*m1)+m2)*l1*g 0;
    0 0 .5*m2*l2*g;
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

% Weight matricies for noise
W1 = eye(6);
W2 = eye(6)*2;

% Associated Hamiltonian Matrix
H_kal = [...
    A'   C'*inv(W2)*C;
    W1   -A;
    ];





