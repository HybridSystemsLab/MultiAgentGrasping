%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: Simulation of a hybrid system
% Description: Flow map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xdot_vec = f(state_vec)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%11/03/16
global n_states n_agents;
global x1_d_vec x1_dd_vec x3_d_vec fc_d_vec;
global kp_vec kd_vec kp_y_vec;
global gamma1_vec gamma2_vec;
global kc bc kf;
global epsilon1 epsilon2 threshold;
m = n_states;
n = n_agents;
global nFlag;
global initT maxT eta etaCheck bSetTimer alpha v0;

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

% 07/31/17 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global x0_vec;
y0 = 0.0; % initial y-position

global rho phi_d epsilon_ang phi v0;
% rho = 0.3;
% phi_d = 0.0;
% epsilon_ang = 0.001;

global bImpact tImpact bForce tForce;
global bTurning;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


xdot_vec = [];
for k = 1:n
% x1 = x(1);      % x1, x position
% x2 = x(2);      % x2, x velocity
% x3 = x(3);      % x3, y position
% x4 = x(4);     % x4, y velocity
% qf = x(5);       
% qh = x(6);
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

y0 = x0_vec(3,k);   % initial y-position
% phi_d = phi(k);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_d(1) = x1_d_vec(k);
x_dd(1) = x1_dd_vec(k);
x_d(2) = x3_d_vec(k);
fc_d = fc_d_vec(k);

gamma1 = gamma1_vec(k);
gamma2 = gamma2_vec(k);

kp = kp_vec(k);
kd = kd_vec(k);
kp_y = kp_y_vec(k);

fc = 0;
u = 0;

if(x1 >= 0)
    fc = kc*x1 + bc*x2;
else
    fc = 0;
end


%%%%%%%%%%
if(bSetTimer == 0)
    etaCheck = zeros(1,n_agents);
    if (nFlag <= n_states)
        if nFlag < n_states
            maxT = 0;
        end
        initT = zeros(1,k);
        for k = 1:n
            kp = kp_vec(k);
            kd = kd_vec(k);
            x1_0 = states(1,k);
            x2_0 = states(2,k);
            x1_d = x1_d_vec(k);
            z1 = states(1,k);
            etaCheck(k) = eta_check(kp,kd,x1_0,v0*alpha,x1_d);
           
            eta = etaCheck;
            if(eta(k)>=maxT)
                maxT = eta(k);
                maxT;
            end
            
            initT(k) = maxT - eta(k);
          
        end
    end
%     bSetTimer = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add noise
global noise1 noise2 noise3 noise4 noise5 n1 n2 n3 n4 n5 maxCnt cnt_n time_old;
% 
% n1 = noise1(cnt_n,1);
% n2 = noise2(cnt_n,1);
% n3 = noise3(cnt_n,1);
% n4 = noise3(cnt_n,1);
% n5 = noise3(cnt_n,1);

if (time - time_old) > 0.01
    cnt_n = cnt_n + 1;
    time_old = time;
end

n1 = 0;
n2 = 0;
n3 = 0;
n4 = 0;
n5 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1_error = x1-x_d(1);
x3_error = x3-x_d(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xdot = zeros(m,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% noise
% x1 = x1 + n3;
% x3 = x3 + n4;
% omega = omega + n1;
% v = v + n2;
% fc = fc + n5;

if omega <= -pi/2
    omega = -pi/2;
elseif omega >= pi/2
    omega = pi/2;
end



% horizontal - position controller  
if ((qf == 0) && (qh == 1) && (fc <= gamma2))
    xdot(1) = v;
    xdot(3) = 0.0;
   
    xdot(1) = v * cos(omega);
    xdot(1) = xdot(1) + n1;
    xdot(10) = -kp * x1_error -kd * x2 - fc;
    %
    xdot(10) = xdot(10) + n4;       % v
    
    xdot(2) = xdot(10);
    xdot(1) = x2;

% horizontal - force controller
elseif ((qf == 1) && (qh == 1) && (fc >= gamma1))
    xdot(1) = v;
    v;
    xdot(10) = kf * (fc_d - fc);

    xdot(1) = v * cos(omega);
    xdot(1) = xdot(1) + n1;
    xdot(10) = xdot(10) + n4;
    
    xdot(2) = xdot(10);
    xdot(1) = x2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Turning - position controller
elseif((qf == 0) && (qh == 4) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1) && (bTurning(k) == 1)))
    if abs(x3_error) <= rho
        if sign(omega) == sign(y0)
            xdot(9) = 0.0;
        else
            xdot(9) = sign(y0);
        end
    end
    xdot(1) = v * cos(omega);
    xdot(3) = v * sin(omega);
    
    xdot(1) = xdot(1) + n1;         % position x
    xdot(3) = xdot(3) + n2;         % position y
%     xdot(9) = xdot(9) + n3;         % omega
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif((qf == 0) && (qh == 2) && (abs(x3_error) <= epsilon1) && (nFlag <= n_agents))
%     xdot(1) = 0.0;
%     xdot(3) = 0.0;
%     xdot(9) = 0.0;
%     xdot(10) = 0.0;
   
%
% vertical - position controller
elseif((qh == 0) && (qf == 0) && (abs(x3_error) >= epsilon1) && (x1 <= x_dd(1)))
    xdot(1) = 0.0;
    xdot(3) = -v * sign(y0);
    
    xdot(1) = xdot(1) + n1;         % position x
    xdot(3) = xdot(3) + n2;         % position y
%
elseif((qf == 0) && (qh == 3) && (tau >= 0) && (bSetTimer == 1))
    xdot(7) = -1;
else
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xdot(9) = xdot(9) + n3;         % omega
xdot(10) = xdot(10) + n4;       % v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% time
xdot(8) = 1;
xdot_vec = [xdot_vec; xdot];

if (bImpact(k) == 0) && (x1 >= 0.0)
    bImpact(k) = 1;
    tImpact(k) = time;
%     k, time, x1, qf
end

if (bForce(k) == 0) && (qf == 1)
    bForce(k) = 1;
    tForce(k) = time;
%     k
%     time
%     x1
%     
end
end % for k = 1:n

end
