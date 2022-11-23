clear 
close all
clc

%% Step 1 : Generating Initial Database

endtime = 200;
starttime = 3;
runtime = 1:endtime;
t_change = endtime;
epochs = 50;

nu = 2;
ny = 3;
N_0 = endtime;

r = zeros(endtime, 1);
y0 = zeros(endtime, 1);
u0 = zeros(endtime, 1);
x0 = zeros(endtime, 1);

%%%%%%%
db = zeros(N_0 , nu+ny+1 + 3 + 1);
%%%%%%%

% initial K derived from CHR tuning method 

K_fix = [0.486 0.227 0.122];  % Kd

% reference signal r(t)

for t=1:endtime 
    
    if(t<50)
        r(t) = 0.5;
    elseif(t<100)
        r(t) = 1.0;
    elseif(t<150)
        r(t) = 2.0;
    elseif(t<=endtime)
        r(t) = 1.5;
    end
    
end

% output using the fixed PID control
for t = starttime:endtime
    
    % system output signal
    y0(t) = 0.6*y0(t-1) - 0.1*y0(t-2) + 1.2*x0(t-1) - 0.1*x0(t-2);
    % error
    e_t = r(t) - y0(t);
    % system input signal
    u0(t) = u0(t-1) + K_fix(2)*e_t - K_fix(1)*(y0(t) - y0(t-1)) - K_fix(3)*(y0(t) - 2*y0(t-1) + y0(t-2));
    
    % intermediate signal
    if(t<t_change)
        x0(t) = 1.5*u0(t) - 1.5*u0(t)^2 + 0.5*u0(t)^3;
    else
        x0(t) = 1.0*u0(t) - 1.0*u0(t)^2 + 1.0*u0(t)^3;
    end
    
end

% the database
db(:, nu+ny+2:nu+ny+4) = repmat(K_fix,endtime,1);

for i = 1:endtime
    t = round(i);

    if(t<=starttime)
        db(t, :) = [r(t+1); r(t); 0; 0; 0; 0; K_fix(1); K_fix(2); K_fix(3); t];
    elseif(t<endtime)
        db(t, :) = [r(t+1); r(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1); K_fix(1); K_fix(2); K_fix(3); t];
    else
        db(t, :) = [0; r(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1); K_fix(1); K_fix(2); K_fix(3); t];
    end
        
end


%% Step 2 : Calculating distance and Select Neighbours

knn = 6;

t1 = -.271;
t2 = .0183;
lr = [0.1 0.001 0.1]*1;    %[ni np nd]

y = zeros(endtime, 1);
x = zeros(endtime, 1);
u = zeros(endtime, 1);
r1 = zeros(endtime, 1);
K_nn = ones(endtime, knn);
N = zeros(endtime, 1);
wi = zeros(endtime, knn);
d = zeros(endtime, N_0);

K_old = zeros(endtime, 3);
K_new = zeros(endtime, 3);
K = zeros(1, 3);
grad = zeros(endtime, 3);
yr = zeros(endtime, 1);
J_ep = zeros(epochs, 1);
e = zeros(endtime, 1);

% initializing K_old and K_new
K_old(1:starttime, :) = repmat(K_fix,starttime,1);
K_new(1:starttime, :) = repmat(K_fix,starttime,1);

% initializing N
N(starttime) = N_0;

for ep = 1:epochs
    
    for t = starttime:endtime-1
        
        % information vector / Query
        query_t = [r(t+1); r(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1)].';

        % k nearest neighbours
        for j = 1:N_0

            d(t,j) = sum(abs((query_t - db(j,1:nu+ny+1))./(max(db(1:N_0,1:nu+ny+1)) - min(db(1:N_0,1:nu+ny+1)))));

        end

        [~, sorted_index] = sort(d(t,1:N_0));

        K_nn(t, :) = sorted_index(1:knn);

        %% Step 3 : Computing PID Parameters

        wi(t, :) = exp(-d(t,K_nn(t, :)))/sum(exp(-d(t,K_nn(t, :))));

        K_old(t, :) = wi(t, :)*db(K_nn(t,:), nu+ny+2:nu+ny+4);
               
        %% Step 4 : FRIT PID Parameters Adjustment
        
        r1(t) = y0(t) + 1/K_fix(2)*(u0(t) - u0(t-1) + K_fix(1)*(y0(t) - y0(t-1)) + K_fix(3)*(y0(t) - 2*y0(t-1) + y0(t-2)));
        
        yr(t+1) = -t1*yr(t) - t2*yr(t-1) + (1+t1+t2)*r1(t); 
        
        e(t+1) = y0(t+1) - yr(t+1);

        x0_t = u0(t) - u0(t-1) + K_old(t,1)*(y0(t) - y0(t-1)) + K_old(t,3)*(y0(t) - 2*y0(t-1) + y0(t-2));

        grad(t, :) = e(t+1)*(1+t1+t2)*[-(y0(t)-y0(t-1))/K_old(t,2); x0_t/K_old(t,2)^2; -(y0(t)-2*y0(t-1)+y0(t-2))/K_old(t,2)];

        K_new(t, :) = K_old(t, :) - lr.*grad(t, :);
        
        % Updating the PID parameters
        db(t, nu+ny+2:nu+ny+4) = [K_new(t, 1); K_new(t,2); K_new(t,3)].';

    end
    
    J_ep(ep) = 1/endtime*sum(e.^2);
    
end

for t = starttime: endtime-1
    
    y(t) = 0.6*y(t-1) - 0.1*y(t-2) + 1.2*x(t-1) - 0.1*x(t-2);
    
    e_t = r(t) - y(t);
    
    d_t = zeros(1, N_0);
    
    % information vector / Query
    query_t = [r(t+1); r(t); y(t:-1:t-ny+1); u(t-1:-1:t-nu+1)].';

    % k nearest neighbours
    for j = 1:N_0

        d_t(j) = sum(abs((query_t - db(j,1:nu+ny+1))./(max(db(1:N_0,1:nu+ny+1)) - min(db(1:N_0,1:nu+ny+1)))));

    end

    [~, sorted_index] = sort(d_t);

    K_nn(t, :) = sorted_index(1:knn);

    %% Step 3 : Computing PID Parameters

    wi(t, :) = exp(-d(t,K_nn(t, :)))/sum(exp(-d(t,K_nn(t, :))));

    K = wi(t, :)*db(K_nn(t,:), nu+ny+2:nu+ny+4);
    
    u(t) = u(t-1) + K(2)*e_t - K(1)*(y(t) - y(t-1)) - K(3)*(y(t) - 2*y(t-1) + y(t-2));
    
    if(t<t_change)
        x(t) = 1.5*u(t) - 1.5*u(t)^2 + 0.5*u(t)^3;
    else
        x(t) = 1.0*u(t) - 1.0*u(t)^2 + 1.0*u(t)^3;
    end
        
end
    
figure(1);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y0, 'b');
ylabel('y');
xlabel('t[step]');
legend({'r', 'y0'}, 'Location', 'northwest');
axis([0 200 0 2.5]);
grid on;

subplot(2,1,2);
plot(runtime, u0, 'b');
ylabel('u');
xlabel('t[step]');
axis([0 200 0 2.5]);
grid on;

figure(2);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y, 'b');
ylabel('y');
xlabel('t[step]');
legend({'r', 'y'}, 'Location', 'northwest');
axis([0 200 0 2.5]);
grid on;

subplot(2,1,2);
plot(runtime, u, 'b');
ylabel('u');
xlabel('t[step]');
axis([0 200 0 2.5]);
grid on;

figure(3);
subplot(2,1,1);
plot(runtime, r, '--c', runtime, r1, '--r', runtime, y0, 'k', runtime, y, 'b', runtime, yr, 'm');
ylabel('y');
xlabel('t[step]');
legend({'r', 'r1', 'fixed PID', 'DD FRIT', 'yr'}, 'Location', 'northwest');
axis([0 200 0 2.5]);
grid on;

subplot(2,1,2);
plot(runtime, u0, 'k', runtime, u, 'b');
ylabel('u');
xlabel('t[step]');
axis([0 200 0 2.5]);
grid on;

figure(4);
subplot(3,1,1);
plot(runtime, K_old(:, 1), 'b');
ylabel('Kp');
xlabel('t[step]');
title('K old')
% axis([0 200 0.4 0.5]);
grid on;

subplot(3,1,2);
plot(runtime, K_old(:, 2), 'b');
ylabel('Ki');
xlabel('t[step]');
% axis([0 200 0.22 0.24]);
grid on;

subplot(3,1,3);
plot(runtime, K_old(:, 3), 'b');
ylabel('Kd');
xlabel('t[step]');
% axis([0 200 0.115 0.125]);
grid on;

figure(6)
plot(1:epochs, J_ep);
title('Cost function');
xlabel('epochs');
grid on;
