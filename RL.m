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
%% LQR
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

R = 1;

K = lqr(A,B,Q,R);
[Kt,St,Pt] = lqr(A,B,Q,R);
At = A-B*K;

s = .01;
t = [0:s:10];
x0 = [0,deg2rad(20),deg2rad(40),0,0,0] ;

sys_cl = ss(At,B, eye(6),zeros(6,1));

[y_cl, tOut,x_cl] = initial(sys_cl,x0,t);
%% RL
clc
A = [...
    zeros(3) eye(3);
    -inv(M_0)*pG_0 zeros(3);
    ];

B = [...
    zeros(3,1);
    inv(M_0)*H;
    ];

%Q = [1 0 0 0 0 0;
     % 0 1 0 0 0 0;
     % 0 0 1 0 0 0;
     % 0 0 0 1 0 0;
     % 0 0 0 0 1 0;
     % 0 0 0 0 0 1];
Q = diag([70;100;100;30;1;1]);
R = 1;

C = eye(6);
s = .01
t = [0:s:.5]

[Kt,St,Pt] = lqr(A,B,Q,R);   %Testing
[Krl,Srl,Prl] = lqr(A,B,Q,R) %Optimal Case

%Made the guess the same as the optimal case 
  Guess_F = [1.0000 -331.9622  358.9084    3.7416  -17.6071   59.2992]; %Made the guess the same as the optimal case 
Optimal_F = [1.0000 -331.9622  358.9084    3.7416  -17.6071   59.2992];


x0_rl = [0,deg2rad(20),deg2rad(40),0,0,0]

M = 1;       % Number of epochs


for epoch = 1:M
    Arl = A-B*Guess_F
    sys = ss(Arl,zeros(size(Arl)), eye(size(Arl)),0);
    
    xrl = initial(sys,x0_rl,t);

    sim1_epoch_2 = [];
    sim2_epoch_2 = [];
    

    x1rl = xrl(:,1); %Vehicle Position Data 
    x2rl = xrl(:,2); %Vehicle Velocity Data
    x3rl = xrl(:,3); %Pendulum 1 Angle 
    x4rl = xrl(:,4); %Pendulum 1 A. Velocity
    x5rl = xrl(:,5); %Pendulum 2 Angle  
    x6rl = xrl(:,6); %Pendulum 2 A. Velocuty

    %LHS Initial Step
     sim1_epoch_2(1,:) = [...
        x1rl(1)^2 - x1rl(2)^2,  2*x1rl(1)*x2rl(1) - 2*x1rl(2)*x2rl(2), 2*x1rl(1)*x3rl(1) - 2*x1rl(2)*x3rl(2), 2*x1rl(1)*x4rl(1) - 2*x1rl(2)*x4rl(2), 2*x1rl(1)*x5rl(1) - 2*x1rl(2)*x5rl(2), 2*x1rl(1)*x6rl(1) - 2*x1rl(2)*x6rl(2), ...
        x2rl(1)^2 - x2rl(2)^2,  2*x2rl(1)*x3rl(1) - 2*x2rl(2)*x3rl(2), 2*x2rl(1)*x4rl(1) - 2*x2rl(2)*x4rl(2), 2*x2rl(1)*x5rl(1) - 2*x2rl(2)*x5rl(2), 2*x2rl(1)*x6rl(1) - 2*x2rl(2)*x6rl(2), ...
        x3rl(1)^2 - x3rl(2)^2,  2*x3rl(1)*x4rl(1) - 2*x3rl(2)*x4rl(2), 2*x3rl(1)*x5rl(1) - 2*x3rl(2)*x5rl(2), 2*x3rl(1)*x6rl(1) - 2*x3rl(2)*x6rl(2), ...
        x4rl(1)^2 - x4rl(2)^2,  2*x4rl(1)*x5rl(1) - 2*x4rl(2)*x5rl(2), 2*x4rl(1)*x6rl(1) - 2*x4rl(2)*x6rl(2), ...
        x5rl(1)^2 - x5rl(2)^2,  2*x5rl(1)*x6rl(1) - 2*x5rl(2)*x6rl(2), ...
        x6rl(1)^2 - x6rl(2)^2
        ];            



    %RHS Initial Step
    sim2_epoch_2(1) = ([x1rl(1), x2rl(1),x3rl(1), x4rl(1),x5rl(1),x6rl(1)]*(Q + Guess_F'*R*Guess_F)*[x1rl(1), x2rl(1),x3rl(1), x4rl(1),x5rl(1),x6rl(1)]')*s;                          
    
    for i = 2:length(t)-1
        % Left-hand side (Radial Basis)
        lhs_2 = [...
        x1rl(1)^2 - x1rl(i+1)^2,  2*x1rl(1)*x2rl(1) - 2*x1rl(i+1)*x2rl(i+1), 2*x1rl(1)*x3rl(1) - 2*x1rl(i+1)*x3rl(i+1), 2*x1rl(1)*x4rl(1) - 2*x1rl(i+1)*x4rl(i+1), 2*x1rl(1)*x5rl(1) - 2*x1rl(i+1)*x5rl(i+1), 2*x1rl(1)*x6rl(1) - 2*x1rl(i+1)*x6rl(i+1), ...
        x2rl(1)^2 - x2rl(i+1)^2,  2*x2rl(1)*x3rl(1) - 2*x2rl(i+1)*x3rl(i+1), 2*x2rl(1)*x4rl(1) - 2*x2rl(i+1)*x4rl(i+1), 2*x2rl(1)*x5rl(1) - 2*x2rl(i+1)*x5rl(i+1), 2*x2rl(1)*x6rl(1) - 2*x2rl(i+1)*x6rl(i+1), ...
        x3rl(1)^2 - x3rl(i+1)^2,  2*x3rl(1)*x4rl(1) - 2*x3rl(i+1)*x4rl(i+1), 2*x3rl(1)*x5rl(1) - 2*x3rl(i+1)*x5rl(i+1), 2*x3rl(1)*x6rl(1) - 2*x3rl(i+1)*x6rl(i+1), ...
        x4rl(1)^2 - x4rl(i+1)^2,  2*x4rl(1)*x5rl(1) - 2*x4rl(i+1)*x5rl(i+1), 2*x4rl(1)*x6rl(1) - 2*x4rl(i+1)*x6rl(i+1), ...
        x5rl(1)^2 - x5rl(i+1)^2,  2*x5rl(1)*x6rl(1) - 2*x5rl(i+1)*x6rl(i+1), ...
        x6rl(1)^2 - x6rl(i+1)^2
        ]; 
        
        % Right-hand side (cost function approximation using backward Euler)
        rhs_2 = ([x1rl(i), x2rl(i),x3rl(i), x4rl(i),x5rl(i),x6rl(i)]*(Q + Guess_F'*R*Guess_F)*[x1rl(i), x2rl(i),x3rl(i), x4rl(i),x5rl(i),x6rl(i)]')*s + sim2_epoch_2(i-1) ;
        
        % Collect simulation data
        sim1_epoch_2(i,:) = lhs_2;
        sim2_epoch_2(i) = rhs_2;
    end

    % Convert simulation results into column vectors
    sim1_epoch_2 = sim1_epoch_2;
    sim2_epoch_2 = sim2_epoch_2';

    % Calculate the new control policy F based on least-squares estimates
    k_2 = pinv(sim1_epoch_2) * sim2_epoch_2;
    k_2 = [k_2(1,1),k_2(2,1) ,k_2(3,1) ,k_2(4,1) ,k_2(5,1),k_2(6,1);
           k_2(2,1),k_2(7,1) ,k_2(8,1) ,k_2(9,1) ,k_2(10,1),k_2(11,1);
           k_2(3,1),k_2(8,1) ,k_2(12,1),k_2(13,1),k_2(14,1),k_2(15,1);
           k_2(4,1),k_2(9,1) ,k_2(13,1),k_2(16,1),k_2(17,1),k_2(18,1);
           k_2(5,1),k_2(10,1),k_2(14,1),k_2(17,1),k_2(19,1),k_2(20,1);
           k_2(6,1),k_2(11,1),k_2(15,1),k_2(18,1),k_2(20,1),k_2(21,1)
           ];
    
    Fnew_2 = inv(R) * B' * k_2;
    Guess_F = Fnew_2;

    x0_rl = [xrl(end,1),xrl(end,2),xrl(end,3),xrl(end,4),xrl(end,5),xrl(end,6)];

end 

x0_rl = [0,deg2rad(20),deg2rad(40),0,0,0]
Frl = Guess_F

Guess_f_i = [1.0000 -331.9622  358.9084    3.7416  -17.6071   59.2992]
F_optimal = inv(R)*B'*St


%Estimated Case 
A_rl = A-B*Frl
sys_rl = ss(A_rl,zeros(size(A_rl)),eye(size(A_rl)),0)
x_estimated_2 = initial(sys_rl,x0_rl,t)

%Optimal Case 
A_optimal_2 = A-B*F_optimal
sys_optimal_2 = ss(A_optimal_2,zeros(size(A_optimal_2)),eye(size(A_optimal_2)),0)
x_optimal_2 = initial(sys_optimal_2,x0_rl,t)



figure; 
plot(t,x_optimal_2(:,1),'o--',t,x_estimated_2(:,1))%,t,x_optimal_2(:,2),'x--',t,x_estimated_2(:,2),'--',t,x_optimal_2(:,3),'o',t,x_estimated_2(:,3),'b--');
legend('Optimal X' , 'Estimated X','Optimal Link Angle 1','Estimated Link Angle 1','Optimal Link Angle 2','Estimated Link Angle 2')
title('LQR + RL Responses')
xlabel('Time (s)')
ylabel('Position (m)')

%% Plot
cartPlot(x_estimated_2, m0, m1, m2, l1, l2, false, false, 10);