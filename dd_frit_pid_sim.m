clear 
close all
clc

starttime = 3;
endtime = 200;
nu = 2;
ny = 3;

sigma = 1;
delta = 0;
Ts = 1;
rho = Ts/sigma;
mu = 0.25*(1-delta) + 0.51*delta;

t1 = -2*exp(-rho/(2*mu))*cos(sqrt(4*mu-1)/(2*mu)*rho);
t2 = exp(-rho/mu);

lr = [0.1 0.001 0.1]*1;    %[ni np nd]
knn = 6;
epochs = 100;
N_0 = endtime+1;

K = [0.4860 0.2270 0.1220];
             
filename = 'init_sim';      
open(filename);              
sim(filename);

u0 = logsout.getElement('u0').Values.Data;
y0 = logsout.getElement('y0').Values.Data;
r0 = logsout.getElement('r0').Values.Data;

db = [[r0(2:endtime+1); zeros(1,1)] r0 y0 [zeros(1,1); y0(1:endtime)] [zeros(1,2).'; y0(1:endtime-1)] [zeros(1,1); u0(1:endtime)] repmat(K,endtime+1,1)];

x = zeros(endtime, 1);
r1 = zeros(endtime, 1);
K_nn = ones(endtime, knn);
wi = zeros(endtime, knn);
d = zeros(endtime, N_0);

K_old = zeros(endtime, 3);
K_new = zeros(endtime, 3);
grad = zeros(endtime, 3);
yr = zeros(endtime, 1);
J_ep = zeros(epochs, 1);
e = zeros(endtime, 1);

% initializing K_old and K_new
K_old(1:starttime, :) = repmat(K,starttime,1);
K_new(1:starttime, :) = repmat(K,starttime,1);

% initializing N
N(starttime) = N_0;

for ep = 1:epochs
    
    for t = starttime:endtime
        
        % information vector / Query
        query_t = [r0(t+1); r0(t); y0(t:-1:t-ny+1); u0(t-1:-1:t-nu+1)].';

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
        
%         r1(t) = y0(t) + 1/K_old(t,2)*(u0(t) - u0(t-1) + K_old(t,1)*(y0(t) - y0(t-1)) + K_old(t,3)*(y0(t) - 2*y0(t-1) + y0(t-2)));
        r1(t) = y0(t) + 1/K(2)*(u0(t) - u0(t-1) + K(1)*(y0(t) - y0(t-1)) + K(3)*(y0(t) - 2*y0(t-1) + y0(t-2)));
        
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

filename = 'dd_frit_sim';
open(filename);
sim(filename);

u = logsout.getElement('u').Values.Data;
y = logsout.getElement('y').Values.Data;
r = logsout.getElement('r').Values.Data;

figure(1)
plot(1:epochs, J_ep);
title('Cost function');
xlabel('epochs');
grid on;
