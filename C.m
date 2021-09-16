%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Flow set
% Return 0 if outside of C, and 1 if inside C
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [value] = C(state_vec) 

global n_states n_agents;
global x1_d_vec x1_dd_vec x3_d_vec fc_d_vec;
global kp_vec kd_vec kp_y_vec;
global gamma1_vec gamma2_vec;
global kc bc kf;
global epsilon1 epsilon2 threshold;
m = n_states;
n = n_agents;
global nFlag;

% states %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
states = zeros(m,n);
if(length(state_vec) == m*n)
    for i = 1:n
        for jj = 1:m
            states(jj,i) = state_vec(jj + m*(i-1));
        end
    end
else 
end % if(length(state_vec) == m*n)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% states;
count = 0;

global rho;
global bSetTimer;
global bTurning;

for k = 1 :n
x1 = states(1,k);
x2 = states(2,k);
x3 = states(3,k);
qf = states(5,k);
qh = states(6,k);
tau = states(7,k);
time = states(8,k);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omega = states(9,k);
v = states(10,k);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_d(1) = x1_d_vec(k);
x_dd(1) = x1_dd_vec(k);
x_d(2) = x3_d_vec(k);

gamma1 = gamma1_vec(k);
gamma2 = gamma2_vec(k);

kp = kp_vec(k);
kd = kd_vec(k);
kp_y = kp_y_vec(k);

x3_error = x3-x_d(2);

if(x1 >= 0)
    fc = kc*x1 + bc*x2;
else
    fc = 0;
end

if ((qf == 0) && (qh == 1) && (fc <= gamma2))
    count = count+1;
elseif ((qf == 1) && (qh == 1) && (fc >= gamma1))
    count = count+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif((qf == 0) && (qh == 4) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1)) && (bTurning(k) == 1))
    count = count +1;
elseif((qh == 0) && (qf == 0) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1)))
    count = count+1;
elseif((qf == 0) && (qh == 2) && (abs(x3_error) <= epsilon1) && (nFlag <= n_agents))
   count = count+1;
elseif((qf == 0) && (qh == 3) && (tau >= 0) && (bSetTimer == 1))
    count = count+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
%     value = 0;    
end
end % for k = 1:n

if count > 0
    value = 1;
else
    value = 0;
end


end
