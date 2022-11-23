clear 
close all
clc

K = 5, T = 15, Tm = 23 

% Plant
G = tf(K, [T 1])

% Reference output
Gm = tf([1], [Tm^2/4 Tm 1])
step(Gm)
hold on

% I-PD controller designing
Ki = 4*T/(Tm^2*K)
Kp = (Tm*K*Ki - 1)/K
Kd = 0

Cpd = tf([Kd Kp], 1)
V = feedback(G, Cpd, -1)

Ci = tf([Ki], [1 0])
W = feedback(Ci*V, 1, -1)
step(W)
grid on

