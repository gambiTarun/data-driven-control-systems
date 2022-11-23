% uncontrolled

clear all
clc

num = [1]
denom = [1 3 1]

Gp = tf(num, denom) %plant (actuator->process)
H = [1]

M = feedback(Gp, H)
step(M)
hold on

%%

%controlled

Kp = 20
Ki = 5
Kd = 4

Gc = pid(Kp, Ki, Kd) %controller

Mc = feedback( Gc*Gp , H )
step(Mc)
grid on
