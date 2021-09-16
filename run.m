clc
clear all;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Grasping Algorithm
% output - G
% i : number of contact points
% G(1,i), G(2,i) : direction of contact force
% G(3,i), G(4,i) : contact point
% G(5,i) : force intensity

% load object data;
load wall_G.mat;
load wall_V.mat;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[m n] = size(G);

% initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x1, horizontal position x
% x2, horizontal velocity \dot{x}
% x3, vertical position y
% x4, vertical velocity \dot{y}
% qf, logic variable qf
% qh, logic variable qh
% tau, timer
% t, time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% w, angular velocity (turn rate)
% v, linear velocity (on x-direction)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots
bObject = 0;        % "bObject = 1", show contact location
bLocal = 0;         % "bLocal = 1", show agents' xy-trajectories in local frame
bTimePlot = 1;      % "bTimePlot = 1", show all agents' state
bTimePlot2 = 0;     % "bTimePlot2 = 1", [car-type model] show all agents' state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tmpG = [];
vec_G = [];
for i = 1:n
    if G(5,i) > 0.001
        for j = 1:m
            vec_G(j,1) = G(j,i);
        end
        tmpG = [tmpG, vec_G];
    end
end
G = tmpG;

global n_states n_agents;
[m n] = size(G);
n_agents = n;
n_states = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global nFlag;
global initT maxT eta bSetTimer;
global etaCheck;
initT = zeros(1, n_agents);
maxT = 0.0;
eta = zeros(1, n_agents);
etaCheck = zeros(1, n_agents);
nFlag = 0;
bSetTimer = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global x0_vec;
x0_vec = zeros(n_states,n_agents);
x0_vec(1,1) = -2.5;
x0_vec(3,1) = -3;

x0_vec(1,2) = -2;
x0_vec(3,2) = 3;

x0_vec(1,3) = -2.5;
x0_vec(3,3) = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial heading angle (along the y-direction)
for i = 1:n_agents
    if x0_vec(3,i) >= 0
        x0_vec(9,i) = -pi/2.0;
    else
        x0_vec(9,i) = pi/2.0;
    end
end

% linear velocity, v
global v0;
global alpha;
alpha = 1.0;
v0 = 1.0;
x0_vec(10,1) = v0;
x0_vec(10,2) = v0;
x0_vec(10,3) = v0;

x0_vec(4,1) = v0;
x0_vec(4,2) = v0;
x0_vec(4,3) = v0;

global rho phi_d epsilon_ang phi;
rho = v0;
phi_d = 0.0;
epsilon_ang = 0.001;
phi = zeros(1,n_agents);

global bImpact tImpact bForce tForce;
bImpact = zeros(1, n_agents);
tImpact = zeros(1, n_agents);
bForce = zeros(1, n_agents);
tForce = zeros(1, n_agents);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global bTurning;
bTurning = zeros(n_agents,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reset the desired contact force %%%%%%%%%%%%%%
global fc_d_vec;
fc_d_vec = zeros(1,n_agents);
for i = 1:n_agents
    magnitude = G(5,i);
    fc_d_vec = round(magnitude, 4) * 10;
%     fc_d_vec(i) = 5;
%     fc_d_vec(i) = 10;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% controller design %%%%%%%%%%%%%%%%%%%%%%%%%%%%
global x1_d_vec x1_dd_vec x3_d_vec;
global kp_vec kd_vec kp_y_vec;
global gamma1_vec gamma2_vec;
x1_d_vec = zeros(1,n);
x1_dd_vec = zeros(1,n);
x3_d_vec = zeros(1,n);
% threshold, kc, bc, kf, epsilon1, epsilon2;
kp_vec = zeros(1,n);
kd_vec = zeros(1,n);
kp_y_vec = zeros(1,n);
gamma1_vec = zeros(1,n);
gamma2_vec = zeros(1,n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global kc bc kf;
global epsilon1 epsilon2 threshold;

kc = 10;                    % elastic parameter of contact
bc = 0.3;                   % viscous parameter of contact

kp_vec = [2, 2, 2];
kd_vec = [0.7, 0.5, 0.5];
kp_y_vec = [4, 4, 4];
gamma1_vec = [0.76; 0.76; 0.76];
gamma2_vec = [1.33; 1.33; 1.33];
%
kp_vec = [1, 1, 1];
kd_vec = [2, 2, 2];
kf = 4.0;

epsilon1 = 0.01;
epsilon2 = 0.05;
threshold = 0.05;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frame variables
global frame_x frame_y;
scale = 2;
frame_x = zeros(2,2,n);
frame_y = zeros(2,2,n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pos = zeros(2,1);
dir_x = zeros(2,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%08/04/17
x1_d_vec(1) = 1.29;
x1_d_vec(2) = 1.26;
x1_d_vec(3) = 1.04;

fc_d_vec(1) = 4.39;
fc_d_vec(2) = 4.2;
fc_d_vec(3) = 2.97;


% global state_vec;
state_vec = [];
for k = 1:n
    % Set the parameters
    gamma1 = gamma1_vec(k);
    gamma2 = gamma2_vec(k);
    % x0
    pos(1) = G(1,k);            % contact point
    pos(2) = G(2,k);
    dir_x(1) = G(3,k);          % direction of contact force
    dir_x(2) = G(4,k);
    magnitude = G(5,k);         % force intensity (fc_d)
    
    dir_x = dir_x/norm(dir_x);

    % Drawing axes %%%%%%%%t%%%%%%%%%%%%%%%%%%%%%
    frame_x(1,1,k) = pos(1);
    frame_x(1,2,k) = pos(2);
    frame_x(2,1,k) = pos(1)+dir_x(1)*scale;
    frame_x(2,2,k) = pos(2)+dir_x(2)*scale;
    frame_y(1,1,k) = pos(1);
    frame_y(1,2,k) = pos(2);
    frame_y(2,1,k) = pos(1)-dir_x(2)*scale;
    frame_y(2,2,k) = pos(2)+dir_x(1)*scale;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1_dd_vec(k) = x1_d_vec(k) - threshold;

% Global Frame
x_d = zeros(2,1);
x_dd = zeros(2,1);
tmp = zeros(2,1);

x_d(1) = x1_d_vec(k);
x_d(2) = x3_d_vec(k);
x_dd(1) = x1_dd_vec(k);
x_dd(2) = 0;

x_d_ = global_vec2(pos, dir_x, x_d);
x_dd_ = global_vec2(pos, dir_x, x_dd);

% equilibrium point - force controller
fc_d = fc_d_vec(k);
x_f_x1 = fc_d/kc;
tmp(1) = x_f_x1;
tmp(2) = 0;
x_f_x1_ = global_vec2(pos, dir_x, tmp);

% get initial state (wrt global frame)
x0 = x0_vec(:,k);
x0_ = x0;
%
tmp(1) = x0(1);
tmp(2) = x0(3);
tmp = global_vec2(pos, dir_x, tmp);
x0_(1) = tmp(1);
x0_(3) = tmp(2);
%
tmp(1) = x0(2);
tmp(2) = 0;
tmp = global_vec2(pos, dir_x, tmp);
x0_(2) = tmp(1);
x0_(4) = tmp(2);

% SET state variables %%%%%%%%%%%%%%%%%%%%%%%%%%
state_vec = [state_vec; x0];
end % k
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add noise
global noise1 noise2 noise3 noise4 noise5 n1 n2 n3 n4 n5 maxCnt cnt_n time_old;
% var = 1;
maxCnt = 1000;
% epsilon1 = var;
var = epsilon1;
var = 0.3;
%
noise1 = var .* randn(maxCnt,1);
noise2 = var .* randn(maxCnt,1);
noise3 = var .* randn(maxCnt,1);
noise4 = var .* randn(maxCnt,1);
noise5 = 0.001 .* randn(maxCnt,1);


n1 = 0.0;
n2 = 0.0;
n3 = 0.0;
n4 = 0.0;
n5 = 0.0;
cnt_n = 1;
time_old = 0.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulation horizon
TSPAN=[0 15];
JSPAN = [0 20];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-6,'MaxStep',.1);

% local
[t,j,x_vec] = HyEQsolver( @f,@g,@C,@D,...
    state_vec,TSPAN,JSPAN,rule,options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local Frame
nSol = length(x_vec);
state = zeros(nSol, n_states, n_agents);
for i = 1:nSol
    for k = 1:n_agents
        for jj = 1:n_states
            state(i,jj,k) = x_vec(i, jj+n_states * (k-1));
        end % for k = 1:n_agents
    end % for j = 1:n_states
end % for i = 1:length(x_vec)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Debug
xx1 = zeros(nSol, n_states);
xx2 = zeros(nSol, n_states);
xx3 = zeros(nSol, n_states);
xx1 = state(:,:,1);
xx2 = state(:,:,2);
xx3 = state(:,:,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Frame
x = zeros(nSol, n_states);
x_ = zeros(nSol, n_states);
state_ = zeros(nSol, n_states, n_agents);
for k = 1:n_agents
    pos(1) = G(1,k);            % contact point
    pos(2) = G(2,k);
    dir_x(1) = G(3,k);          % direction of contact force
    dir_x(2) = G(4,k);
    dir_x = dir_x/norm(dir_x);
	x = state(:,:,k);
    for i = 1:nSol
    	tmp(1) = x(i,1);
        tmp(2) = x(i,3);
        tmp = global_vec2(pos, dir_x, tmp);
        x_(i,1) = tmp(1);
        x_(i,3) = tmp(2);
        %
        tmp(1) = x(i,2);
        tmp(2) = 0;
        tmp = global_vec2(pos, dir_x, tmp);
        x_(i,2) = tmp(1);
        x_(i,4) = tmp(2);
        %
        x_(i,5) = x(i,5);
        x_(i,6) = x(i,6);
    end % for i = 1:nSol
    state_(:,:,k) = x_(:,:);
end % for k = 1:n_agents

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Debug
xx1_ = zeros(nSol, n_states);
xx2_ = zeros(nSol, n_states);
xx3_ = zeros(nSol, n_states);
xx1_ = state_(:,:,1);
xx2_ = state_(:,:,2);
xx3_ = state_(:,:,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:n_agents
    save('time', 't');
    save('jump', 'j');
    result = state(:,:,k);
    save(['local_', num2str(k)], 'result');
    result = state_(:,:,k);
    save(['global_', num2str(k)], 'result');  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots

% draw object
figure(1);
VV=[V;V(1,:)];
plot(VV(:,1),VV(:,2),'b-','LineWidth',1.5);
hold on;
%
for i = 1:n_agents
plot(state_(1,1,i), state_(1,3,i), 'rx');
hold on;
plot(state_(:,1,i), state_(:,3,i),  'k-');
hold on;
plot(state_(end,1,i), state_(end,3,i),  'ks', 'MarkerFaceColor', 'k');
hold on;
%
text(frame_x(1,1,i)+0.3, frame_x(1,2,i)+0.5,num2str(i));
hold on;
end
axis equal;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% bObject = 0;        % "bObject = 1", show contact location
% bLocal = 0;         % "bLocal = 1", show agents' xy-trajectories in local frame
% bTimePlot = 1;      % "bTimePlot = 1", show all agents' state
% bTimePlot2 = 0;     % "bTimePlot2 = 1", [car-type model] show all agents' state
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fn = round(n_agents/2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draw axes
% draw object
if(bObject == 1)
figure(2);
clf
VV=[V;V(1,:)];
plot(VV(:,1),VV(:,2),'b:','LineWidth', 0.5);
hold on;
for i = 1:n_agents
    plot(frame_x(1,1,i), frame_x(1,2,i),  'bs', 'MarkerFaceColor', 'b');
    hold on;
    plot(frame_x(:,1,i), frame_x(:,2,i), 'r-','LineWidth', 1.5);
    hold on;
    plot(frame_y(:,1,i), frame_y(:,2,i), 'g-','LineWidth', 1.5);
    hold on;
%
    plot(state_(1,1,i), state_(1,3,i), 'rx');
    hold on;
end
xlabel('x_1');
ylabel('x_3');
title('Object');
axis equal;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%local
if(bLocal==1)
figure(3);
clf;
for k = 1:n_agents
    fx = state(:,:,k);
    
subplot(fn,2,k)
plot(fx(:,1), fx(:,3), 'k-');
axis equal;
grid on;
%
hold on;
plot(fx(1,1),fx(1,3), 'rx');
hold on;
plot(fx(nSol,1),fx(nSol,3), 'ro');
hold on;
plot(0, 0, 'rs', 'MarkerFaceColor', 'r');
hold on;
xlabel('x');      % x_1 state
ylabel('y');      % x_3 state
title(['Local ', num2str(k)]);
end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot all state variables
if(bTimePlot == 1)

% Agent 1
k = 1;
fx = state(:,:,k);
fx_ = state_(:,:,k);
ft = state(:,8,k);
fj = j;

% 
hFig = figure(15);
set(hFig, 'Position', [50 50 350 650])

figure(15)
clf
subplot(7,1,1), plot(ft,fx(:,1),'k-');
title(['local', num2str(k)]);
ylabel('x_1')
%%%%%%%%%%%%%%%%
ttt = 0;
for i = 1:nSol
    if(ttt == 0 && fx(i,1) >= 0)
       hold on;
        plot(ft(i), fx(i,1), 'kx');
       ttt = ft(i);
%        i
    end
end
ttt1 = ttt;
%%%%%%%%%%%%%%%%
subplot(7,1,2), plot(ft,fx(:,2),'k-');
ylabel('x_2')
subplot(7,1,3), plot(ft,fx(:,3),'k-');
ylabel('x_3')
subplot(7,1,4), plot(ft,fx(:,5),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,5) ~= fx(i-1,5))
        plot(ft(i), fx(i,5), '*');
    end
end
%%%%%%%%%
ylabel('q_f')
subplot(7,1,5), plot(ft,fx(:,6),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,6) ~= fx(i-1,6))
        plot(ft(i), fx(i,6), '*');
    end
end
%%%%%%%%%
ylabel('q_h')
subplot(7,1,6), plot(ft,fx(:,7),'k-');
ylabel('tau')
xlabel('t [sec]');
subplot(7,1,7), plot(ft,fx(:,4),'k-');
ylabel('x_4')
xlabel('t [sec]');

% Agent 2
k = 2;
fx = state(:,:,k);
fx_ = state_(:,:,k);
ft = state(:,8,k);
fj = j;

% 
hFig = figure(16);
set(hFig, 'Position', [50 50 350 650])

figure(16)
clf
subplot(7,1,1), plot(ft,fx(:,1),'k-');
% grid on;
title(['local', num2str(k)]);
ylabel('x_1')
%%%%%%%%%%%%%%%%
ttt = 0;
for i = 1:nSol
    if(ttt == 0 && fx(i,1) >= 0)
       hold on;
       plot(ft(i), fx(i,1), 'kx');
       ttt = ft(i);
%        i
    end
end
ttt1 = ttt;
%%%%%%%%%%%%%%%%
subplot(7,1,2), plot(ft,fx(:,2),'k-');
ylabel('x_2')
subplot(7,1,3), plot(ft,fx(:,3),'k-');
ylabel('x_3')
subplot(7,1,4), plot(ft,fx(:,5),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,5) ~= fx(i-1,5))
        plot(ft(i), fx(i,5), '*');
    end
end
%%%%%%%%%
ylabel('q_f')
subplot(7,1,5), plot(ft,fx(:,6),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,6) ~= fx(i-1,6))
        plot(ft(i), fx(i,6), '*');
    end
end
%%%%%%%%%
ylabel('q_h')
subplot(7,1,6), plot(ft,fx(:,7),'k-');
ylabel('tau')
xlabel('t [sec]');
subplot(7,1,7), plot(ft,fx(:,4),'k-');
ylabel('x_4')
xlabel('t [sec]');


% Agent 3
k = 3;
fx = state(:,:,k);
fx_ = state_(:,:,k);
ft = state(:,8,k);
fj = j;

hFig = figure(17);
set(hFig, 'Position', [50 50 350 650])

figure(17)
clf
subplot(7,1,1), plot(ft,fx(:,1),'k-');
title(['local', num2str(k)]);
ylabel('x_1')
%%%%%%%%%%%%%%%%
ttt = 0;
for i = 1:nSol
    if(ttt == 0 && fx(i,1) >= 0)
       hold on;
       plot(ft(i), fx(i,1), 'kx');
       ttt = ft(i);
%        i
    end
end
ttt1 = ttt;
%%%%%%%%%%%%%%%%
subplot(7,1,2), plot(ft,fx(:,2),'k-');
ylabel('x_2')
subplot(7,1,3), plot(ft,fx(:,3),'k-');
ylabel('x_3')
subplot(7,1,4), plot(ft,fx(:,5),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,5) ~= fx(i-1,5))
        plot(ft(i), fx(i,5), '*');
    end
end
%%%%%%%%%
ylabel('q_f')
subplot(7,1,5), plot(ft,fx(:,6),'k-');
%%%%%%%%%
hold on;
for i = 2:nSol
    if (fx(i,6) ~= fx(i-1,6))
        plot(ft(i), fx(i,6), '*');
    end
end
%%%%%%%%%
ylabel('q_h')
subplot(7,1,6), plot(ft,fx(:,7),'k-');
ylabel('tau')
xlabel('t [sec]');
subplot(7,1,7), plot(ft,fx(:,4),'k-');
ylabel('x_4')
xlabel('t [sec]');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Plot agents' x1 state variables
% hFig = figure(18);
% set(hFig, 'Position', [50 100 600 300])
% figure(18)
% clf
% line([155,0],[0,0], 'Color', 'k', 'LineStyle', ':')
% hold on;
% h1 = plot(xx1(:,8), xx1(:,1))
% hold on;
% h2 = plot(xx2(:,8), xx2(:,1))
% hold on;
% h3 = plot(xx3(:,8), xx3(:,1))
% ylabel('x_1')
% xlabel('t [sec]');
% title('local');
% legend([h1 h2 h3],{'1', '2', '3'});
% box on;
% axis([0 15 -2.5 1])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (bTimePlot2 == 1)
for k = 1:n_agents
figure(19+k);
hFig = figure(19+k);
set(hFig, 'Position', [50 50 350 650])

for i = 1:nSol
    sState(i,1) = state(i,6,k);
    if state(i,5,k) == 1
        sState(i,1) = -1;
    end
    if sState(i,1) == 4;
        sState(i,1) = 0;
    end
end

subplot(6,1,1)
plot(t,state(:,1,k), 'k-', 'LineWidth', 1);
hold on;
line([TSPAN(2),0],[0,0], 'Color', 'k', 'LineStyle', ':')
ylabel('x');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), state(i,1,k), '*');
    end
end
xlim([0 13]);

subplot(6,1,2)
plot(t,state(:,3,k), 'k-', 'LineWidth', 1);
hold on;
ylabel('y');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), state(i,3,k), '*');
    end
end
xlim([0 13]);

subplot(6,1,3)
plot(t,state(:,9,k), 'k-', 'LineWidth', 1);
hold on;
ylabel('\theta');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), state(i,9,k), '*');
    end
end
xlim([0 13]);

subplot(6,1,4)
plot(t,state(:,10,k), 'k-', 'LineWidth', 1);
hold on;
ylabel('v');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), state(i,10,k), '*');
    end
end
xlim([0 13]);

subplot(6,1,5)
plot(t,sState(:,1), 'k:', 'LineWidth', 1);
hold on;
ylabel('q');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), sState(i,1), '*');
    end
end
xlim([0 13]);

subplot(6,1,6)
plot(t,state(:,7,k), 'k-', 'LineWidth', 1);
hold on;
ylabel('\tau');
for i = 2:nSol
    if (sState(i,1) ~= sState(i-1,1))
        plot(t(i), state(i,7,k), '*');
    end
end
xlim([0 13]);
end

end
