%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: Simulation of a hybrid system
% Description: Jump map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xplus_vec = g(state_vec)

global n_states n_agents;
global x1_d_vec x1_dd_vec x3_d_vec fc_d_vec;
global kp_vec kd_vec kp_y_vec;
global gamma1_vec gamma2_vec;
global kc bc kf;
global epsilon1 epsilon2 threshold;
m = n_states;
n = n_agents;
global nFlag;
global initT maxT eta etaCheck bSetTimer;

global rho phi_d phi;
global alpha;

global bTurning;
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

xplus_vec = [];

global v0;

for k = 1:n
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
% fc_d = fc_d_vec(k);

gamma1 = gamma1_vec(k);
gamma2 = gamma2_vec(k);

kp = kp_vec(k);
kd = kd_vec(k);
kp_y = kp_y_vec(k);

fc = 0;
x3_error = x3-x_d(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xplus = states(:,k);
global x0_vec;
y0 = x0_vec(3,k);   % initial y-position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(x1 >= 0)
    fc = kc*x1 + bc*v;
else
    fc = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global noise1 noise2 noise3 noise4 noise5 n1 n2 n3 n4 n5 maxCnt cnt_n time_old;
% 
% n1 = noise1(cnt_n,1);
% n2 = noise2(cnt_n,1);
% n3 = noise3(cnt_n,1);
% n4 = noise3(cnt_n,1);
% n5 = noise3(cnt_n,1);


% noise
% x1 = x1 + n3;
% x3 = x3 + n4;
% omega = omega + n1;
% v = v + n2;
% fc = fc + n5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Force control
if ((qf == 0) && (qh == 1) && (abs(x3_error) <= epsilon1) && (fc >= gamma2))
    xplus(5) = 1;
    xplus(6) = 1;
%     v;
    
    xplus(2) = v;
    xplus(10) = v;
    
% Position contol (x-direction)
elseif ((qf == 1) && (qh == 1) && (abs(x3_error) <= epsilon1) && (fc <= gamma1))
    xplus(5) = 0;
    xplus(6) = 1;
    xplus(4) = 0;
    xplus(2) = v0*alpha;
    xplus(10) = v0*alpha;
%     if v ~= v0
% %         qf, qh, v
%     end
    
% Position control (y-direction)
elseif((qh ~= 0) && (qh ~= 4) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1)) && (bTurning(k) == 0))
    xplus(5) = 0;
    xplus(6) = 0;
    xplus(2) = 0.0;
    xplus(4) = v0;
    xplus(9) = -sign(y0) * pi/2;
    xplus(10) = v0;
    
%     if v ~= v0
%         qf, qh, v
%     end
    xplus(4) = v;
    xplus(10) = v;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wait
elseif((qf == 0) && (qh == 4) && (abs(x3_error) <= epsilon1))
    xplus(5) = 0;
    xplus(6) = 2;
    
    xplus(2) = 0.0;
    xplus(4) = 0.0;
    xplus(9) = phi_d;
    xplus(10) = 0.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set timer
elseif((qf == 0) && (qh == 2) && (nFlag == n_agents))
    xplus(5) = 0;
    xplus(6) = 3;
 
    xplus(2) = 0.0;
    xplus(4) = 0.0;
    xplus(9) = phi_d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set timer
    initT(k) = maxT - eta(k);
    xplus(7) = initT(k);
    bSetTimer = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif((qf == 0) && (qh == 3) && (tau <= 0) && (nFlag == n_agents) && (bSetTimer == 1))
    xplus(5) = 0;
    xplus(6) = 1;
    xplus(7) = 0;
    % velocity
    xplus(2) = v0*alpha;
    xplus(4) = 0.0;
    xplus(9) = phi_d;
    xplus(10) = v0*alpha;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Turn 
elseif((qh == 0) && (qf == 0) && (abs(x3_error) >= epsilon1) && (abs(x3_error) <= rho) && (x1 <= x_dd(1)) && (bTurning(k) == 0))
        xplus(5) = 0;
        xplus(6) = 4;
        xplus(10) = v0*alpha;
        % 08/03/17
        xplus(10) = v;
        bTurning(k) = 1;
else
%     xplus;
%     time, v;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xplus_vec = [xplus_vec; xplus];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end % for k = 1:n

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global x0_vec epsilon_ang rho;

if(bSetTimer == 0)
    etaCheck = zeros(1,n_agents);
    if (nFlag <= n_states)
        maxT = 0;
        for k = 1:n
            x1_d = x1_d_vec(k);
            z1 = states(1,k);
    %     
            eta(k) = -z1 / (v0*alpha);
            etaCheck(k) = eta_check2(z1,x1_d,v0*alpha);
           
            eta = etaCheck;
            if(eta(k)>=maxT)
                maxT = eta(k);
%                 maxT;
            end
            
            initT(k) = maxT - eta(k);
        end
    end
%     if (nFlag == n_agents)
%     end
%     bSetTimer = 1;
end

%%%%%%%%%%%%
end


