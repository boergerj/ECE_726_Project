clc; clear; close all;

%% Parameters
m0 = 2; % Mass of cart
m1 = .75; % Mass of first linkage
m2 = 1; % Mass of second linkage

l1 = .75; % Length of linkage 1
l2 = 1; % Length of linkage 2

%% Linearized Plant

g = 9.8;

M_0 = [...
    m0+m1+m2 .5*m2*l2 .5*m2*l2;
    .5*m2*l2 (1/3)*(m1+m2)*l1^2 .5*m2*l1*l2;
    .5*m2*l2 .5*m2*l1*l2 (1/3)*m2*l2^2;
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
    zeros(3);
    inv(M_0)*H;
    ];

C = eye(6);

%% LQG







