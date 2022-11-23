clear all 
clc

K = 2, T = 10, Tm = 10

G = tf(K, [T 1])

% Reference output controller function
Gm = tf([1], [Tm^2/4 Tm 1])
step(Gm)
grid on 
hold on

% controller designing from the reference function
Ki = 4*T/(Tm^2*K)
Kp = (Tm*K*Ki - 1)/K
Kd = 0

C = tf([Kd Kp Ki], [1 0])

W = feedback(G*C, 1)
step(W)
grid on

