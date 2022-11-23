clear 
close all
clc

%% Step 1 : Generating Initial Database

endtime = 200;
starttime = 3;
runtime = 1:endtime;
t_change = endtime;
epochs = 1;

beta_1 = 0.9;
beta_2 =  0.999;
m = 0;
v = 0;
epsilon = 1e-8;

knn = 20;

t1 = -.271;
t2 = .0183;
a1 = 0.5;
a2 = 0.01;
lr = [0.8 0.2 0.9].*1;    %[ni np nd]
grad_yt1_ut = 1;

nu = 2;
ny = 3;
N_0 = knn + 0;

r = zeros(endtime, 1);
y0 = zeros(endtime, 1);
u0 = zeros(endtime, 1);
x0 = zeros(endtime, 1);
noise = wgn(endtime, 1, -40);
K = zeros(endtime, 3);  % [Kp Ki Kd]

%%%%%%%
db = zeros((endtime-starttime)*epochs + N_0 , nu+ny+1 + 3 + 1);
del_db = zeros((endtime-starttime)*epochs, nu+ny+1 + 3 + 1);
del_c = 0;
%%%%%%%

% initial K derived from CHR tuning method 

K(:,1) = 0.486;  % Kp 
K(:,2) = 0.227;  % Ki
K(:,3) = 0.122;  % Kd

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
    u0(t) = u0(t-1) + K(t,2)*e_t - K(t,1)*(y0(t) - y0(t-1)) - K(t,3)*(y0(t) - 2*y0(t-1) + y0(t-2));
    
    % intermediate signal
    if(t<t_change)
        x0(t) = 1.5*u0(t) - 1.5*u0(t)^2 + 0.5*u0(t)^3;
    else
        x0(t) = 1.0*u0(t) - 1.0*u0(t)^2 + 1.0*u0(t)^3;
    end
    
end

% the initial database
Ns = N_0/endtime;   % sampling frequency 
c = 1;

for i = starttime:1/Ns:endtime-1
    
    t = round(i);
    
    if(t<=starttime)
        db(c, :) = [r(t+1); r(t); 0; 0; 0; 0; K(t,1); K(t,2); K(t,3); t];
    elseif(t<endtime)
        db(c, :) = [r(t+1); r(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1); K(t,1); K(t,2); K(t,3); t];
    else
        db(c, :) = [0; r(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1); K(t,1); K(t,2); K(t,3); t];
    end
    
    c = c+1;
end

%% Step 2 : Calculating distance and Select Neighbours

y = zeros(endtime, 1);
x = zeros(endtime, 1);
u = zeros(endtime, 1);
K_nn = ones(endtime, knn);
N = zeros(endtime, 1);
wi = zeros(endtime, knn);
d = zeros(endtime, endtime+N_0);

K_new = zeros(endtime, 3);
yr = zeros(endtime, 1);
J = zeros(endtime, 1);
J_ep = zeros(epochs, 1);
e = zeros(endtime, 1);

K_old = zeros(endtime, 3);

% initializing K_old and K_new
K_old(1:starttime, :) = K(1:starttime, :);
K_new(1:starttime, :) = K(1:starttime, :);

% initializing N
N(starttime) = N_0;

for ep = 1:epochs
    
    m = 0;
    v = 0;
    
    for t = starttime:endtime-1

        % information vector / Query
        query_t = [r(t+1); r(t); y(t:-1:t-ny+1); u(t-1:-1:t-nu+1)].';

%         % k nearest neighbours
%         for j = 1:N(t)
% 
%             d(t,j) = sum(abs((query_t - db(j,1:nu+ny+1))./(max(db(1:N(t),1:nu+ny+1)) - min(db(1:N(t),1:nu+ny+1)))));
% 
%         end
% 
%         [~, sorted_index] = sort(d(t,1:N(t)));
% 
%         K_nn(t, :) = sorted_index(1:knn);
% 
        N(t+1) = N(t) + 1;
% 
% 
        %% Step 3 : Computing PID Parameters
% 
%         wi(t, :) = exp(-d(t,K_nn(t, :)))/sum(exp(-d(t,K_nn(t, :))));
% 
%         K_old(t, :) = wi(t, :)*db(K_nn(t,:), nu+ny+2:nu+ny+4);

        modelp = fitrensemble(db(1:N(t),1:nu+ny+1), db(1:N(t),nu+ny+2));
        modeli = fitrensemble(db(1:N(t),1:nu+ny+1), db(1:N(t),nu+ny+3));
        modeld = fitrensemble(db(1:N(t),1:nu+ny+1), db(1:N(t),nu+ny+4));
        
        K_old(t, :) = [predict(modelp,query_t) predict(modeli,query_t) predict(modeld,query_t)];
            
        e_t = r(t) - y(t);

        u(t) = u(t-1) + K_old(t,2)*e_t - K_old(t,1)*(y(t) - y(t-1)) - K_old(t,3)*(y(t) - 2*y(t-1) + y(t-2));

        if(t<t_change)
            x(t) = 1.5*u(t) - 1.5*u(t)^2 + 0.5*u(t)^3;
        else
            x(t) = 1.0*u(t) - 1.0*u(t)^2 + 1.0*u(t)^3;
        end

        y(t+1) = 0.6*y(t) - 0.1*y(t-1) + 1.2*x(t) - 0.1*x(t-1);


        %% Step 4 : PID Parameters Adjustment

        yr(t+1) = -t1*yr(t) - t2*yr(t-1) + r(t)*(1 + t1 + t2);

        e(t+1) = yr(t+1) - y(t+1);
        J(t+1) = 1/2*e(t+1)^2;

        g_t = [e(t+1)*(y(t)-y(t-1))*grad_yt1_ut; -e(t+1)*e_t*grad_yt1_ut; e(t+1)*(y(t)-2*y(t-1)+y(t-2))*grad_yt1_ut];
        
        m = beta_1 * m + (1 - beta_1) * g_t;
        v = beta_2 * v + (1 - beta_2) * power(g_t, 2);
        m_hat = m / (1 - power(beta_1, t));
        v_hat = v / (1 - power(beta_2, t));
       
        K_new(t, :) = K_old(t, :) - lr .* (m_hat ./ (sqrt(v_hat) + epsilon)).';


        %% Step 5 : Removal of Redundant Data

        extract1 = d(t,1:N(t)) < a1;
        extract2 = sum(((db(1:N(t), nu+ny+2:nu+ny+4) .* repmat(extract1.',1,3) - K_new(t, :))./K_new(t, :)).^2,2) < a2;

        for index = (1:N(t)).*extract2.'
            if(index~=0 && N(t+1)>knn)
                del_c = del_c + 1;
                del_db(del_c, :) = db(index, :);
                db(index, :) = [];
                N(t+1) = N(t+1) - 1;
            end
        end

        % storing query to database
        t_info = N(t+1) ;
        db(t_info, :) = [r(t+1); r(t); y(t:-1:t-ny+1); u(t-1:-1:t-nu+1); K_new(t,1); K_new(t,2); K_new(t,3); t];
        
    end
    
    N(starttime) = N(t+1);
    J_ep(ep) = 1/endtime*sum(e.^2);
    
end
    
figure(1);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y0, 'k');
ylabel('y');
xlabel('t[step]');
legend({'Reference', 'fixed pid output'}, 'Location', 'northwest');
axis([0 200 0 2.5]);
grid on;

subplot(2,1,2);
plot(runtime, u0, 'k');
ylabel('u');
xlabel('t[step]');
axis([0 200 0 2.5]);
grid on;

figure(2);
subplot(2,1,1);
plot(runtime, r, '--r', runtime, y, 'b');
ylabel('y');
xlabel('t[step]');
legend({'Reference', 'dd pid output'}, 'Location', 'northwest');
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
plot(runtime, r(1:endtime), '--r', runtime, y0, 'k', runtime, y, 'b', runtime, yr, 'm');
ylabel('y');
xlabel('t[step]');
legend({'Reference', 'y0', 'y', 'yr'}, 'Location', 'northwest');
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

figure(5)
plot(db(:, 10), db(:, 5), '*', del_db(:, 10), del_db(:, 5), '.');
legend('Database point', 'Redundant point');
grid on;

figure(6)
plot(1:epochs, J_ep);
title('Cost function');
xlabel('epochs');
grid on;
