clear 
close all
clc

%% Step 1 : Initial Database

endtime = 200;
starttime = 3;
runtime = 1:endtime;

r = zeros(endtime, 1);
y0 = zeros(endtime, 1);
u0 = zeros(endtime, 1);
x0 = zeros(endtime, 1);

% initial K derived from CHR tuning method 

K_old = [0.8 0.2 0.001];

% reference signal r(t)

for t=1:endtime 
    
    if(t<20)
        r(t) = 0;
    elseif(t<100)
        r(t) = 1.5;
    elseif(t<150)
        r(t) = 0.5;
    else
        r(t) = 1;
    end
    
end

% output using the fixed PID control
for t = starttime:endtime
   
    % system output 
    y0(t) = 0.2*u0(t-1) - 0.1*y0(t-1);
    % plant input
    u0(t) = u0(t-1) + K_old(2)*(r(t)-y0(t)) - K_old(1)*(y0(t) - y0(t-1)) - K_old(3)*(y0(t) - 2*y0(t-1) + y0(t-2));
        
%     % system output signal
%     y0(t) = 0.6*y0(t-1) - 0.1*y0(t-2) + 1.2*x0(t-1) - 0.1*x0(t-2);
%     % error
%     e_t = r(t) - y0(t);
%     % system input signal
%     u0(t) = u0(t-1) + K_old(2)*e_t - K_old(1)*(y0(t) - y0(t-1)) - K_old(3)*(y0(t) - 2*y0(t-1) + y0(t-2));
%     
%     x0(t) = 1.5*u0(t) - 1.5*u0(t)^2 + 0.5*u0(t)^3;
    
end


%% Step 2 : FRIT

sigma = 1;
delta = 0;
Ts = 1;
rho = Ts/sigma;
mu = 0.25*(1-delta) + 0.51*delta;

t1 = -2*exp(-rho/(2*mu))*cos(sqrt(4*mu-1)/(2*mu)*rho);
t2 = exp(-rho/mu);

y = zeros(endtime, 1);
u = zeros(endtime, 1);
x = zeros(endtime, 1);

J_cost = @(K)frit_costfunc(K,y0,u0,starttime,endtime,t1,t2);

K_new = fminsearch(J_cost, K_old);

for t = starttime:endtime
   
    % system output 
    y(t) = 0.2*u(t-1) - 0.1*y(t-1);
    % plant input
    u(t) = u(t-1) + K_new(2)*(r(t)-y(t)) - K_new(1)*(y(t) - y(t-1)) - K_new(3)*(y(t) - 2*y(t-1) + y(t-2));
    
%     y(t) = 0.6*y(t-1) - 0.1*y(t-2) + 1.2*x(t-1) - 0.1*x(t-2);
%     
%     u(t) = u(t-1) + K_new(2)*e_t - K_new(1)*(y(t) - y(t-1)) - K_new(3)*(y(t) - 2*y(t-1) + y(t-2));
%     
%     x(t) = 1.5*u(t) - 1.5*u(t)^2 + 0.5*u(t)^3;

end
    
figure(1);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y0, 'b');
ylabel('y');
xlabel('t[step]');
legend({'r', 'y0'}, 'Location', 'northwest');
grid on;

subplot(2,1,2);
plot(runtime, u0, 'b');
ylabel('u');
xlabel('t[step]');
grid on;

figure(2);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y0, 'k', runtime, y, 'b');
ylabel('y');
xlabel('t[step]');
legend({'r', 'y0', 'y'}, 'Location', 'southeast');
% axis([0 200 0 2.5]);
grid on;

subplot(2,1,2);
plot(runtime, u0, 'k', runtime, u, 'b');
ylabel('u');
xlabel('t[step]');
% axis([0 200 0 2.5]);
grid on;

