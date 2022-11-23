clear all
clc

%define state matrices 
A = [0 1; 2 -1];
B = [1; 0];
C = [1 0];
D = 0;

% create state space object
sys = ss(A, B, C, D);

%check open loop eigenvalues
E = eig(A);

%desired closed loop eigenvalues
P = [-2 -1];

%solve for K using pole placement
K = place(A, B, P);

%check for closed loop eigenvalues
Acl = A - B*K;
Ecl = eig(Acl);

%create closed loop system
syscl = ss(Acl, B, C, D);

%check step response
step(syscl);
grid on
hold on

%solve for Kr for Normalization
Kdc = dcgain(syscl);
Kr = 1/Kdc;

%create scaled input closed loop system
syscl_scaled = ss(Acl, B*Kr, C, D);
step(syscl_scaled);
grid on