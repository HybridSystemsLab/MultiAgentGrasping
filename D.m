%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Jump set
% Return 0 if outside of D, and 1 if inside D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function inside = D(state_vec) 

global n_states n_agents;
global x1_d_vec x1_dd_vec x3_d_vec fc_d_vec;
global kp_vec kd_vec kp_y_vec;
global gamma1_vec gamma2_vec;
global kc bc kf;
global epsilon1 epsilon2 threshold;
m = n_states;
n = n_agents;
global nFlag;
global initT maxT eta bSetTimer bTurning;

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
count = 0;

% initT(1) = 1;
% initT(2) = 2;
% initT(3) = 0;

if (bSetTimer == 0)
    nFlag = 0;
    for k = 1:n
        x3 = states(3,k);
        x3_d = x3_d_vec(k);
        x3_error = abs(x3-x3_d);
        if(x3_error <= epsilon1)
            nFlag = nFlag +1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global rho phi_d epsilon_ang;

for k = 1:n
x1 = states(1,k);
x2 = states(2,k);
x3 = states(3,k);
qf = states(5,k);
qh = states(6,k);
tau = states(7,k);
time = states(8,k);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omega = states(9,k);
v = states(10,k);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_d(1) = x1_d_vec(k);
x_dd(1) = x1_dd_vec(k);
x_d(2) = x3_d_vec(k);

gamma1 = gamma1_vec(k);
gamma2 = gamma2_vec(k);

kp = kp_vec(k);
kd = kd_vec(k);
kp_y = kp_y_vec(k);

fc = 0;
if(x1 >= 0)
    fc = kc*x1 + bc*x2;
else
    fc = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x3_error = x3-x_d(2);

if(x1 >= 0)
    fc = kc*x1 + bc*v;
else
    fc = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ((qf == 0) && (qh == 1) && (abs(x3_error) <= epsilon1) && (fc >= gamma2))
% jumps to qf=1
    count = count+1;
elseif ((qf == 1) && (qh == 1) && (abs(x3_error) <= epsilon1) && (fc <= gamma1))
% jumps to qf=0
    count = count+1;
elseif((qh ~= 0) && (qh ~= 4) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1)) && (bTurning(k) == 0))
% jumps to qh=0
    count = count+1;
elseif((qf == 0) && (qh == 4) && (abs(x3_error) <= epsilon1) && (nFlag <= n_agents))
% jumps to qh=2
    count = count+1;    
elseif((qf == 0) && (qh == 2) && (nFlag == n_agents))
% jumps to qh=3
    count = count+1;
elseif((qf == 0) && (qh == 3) && (tau <= 0) && (nFlag == n_agents) && (bSetTimer == 1))
% jumps to qh=1
    count = count+1;
elseif ((qh == 0) && (qf == 0) && (abs(x3_error) >= epsilon1) && (abs(x3_error) <= rho) && (x1 <= x_dd(1)))
    count = count+1;
else
%     inside = 0;    
end
end % for k = 1:n

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if count > 0
    inside = 1;
else
    inside = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end






