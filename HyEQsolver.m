function [t j x] = HyEQsolver(f,g,C,D,x0,TSPAN,JSPAN,rule,options)
%HYEQSOLVER solves hybrid equations.
%   Syntax: [t j x] = HyEQsolver(f,g,C,D,x0,TSPAN,JSPAN,rule,options) 
%   computes solutions to the hybrid equations
%
%   \dot{x} = f(x)  x \in C x^+ = g(x)  x \in D
%
%   where x is the state, f is the flow map, g is the jump map, C is the
%   flow set, and D is the jump set. It outputs the state trajectory (t,j)
%   -> x(t,j), where t is the flow time parameter and j is the jump
%   parameter.
%
%   x0 defines the initial condition for the state.
%
%   TSPAN = [TSTART TFINAL] is the time interval. JSPAN = [JSTART JSTOP] is
%       the interval for discrete jumps. The algorithm stop when the first
%       stop condition is reached.
%
%   rule for jumps
%       rule = 1 (default) -> priority for jumps rule = 2 -> priority for
%       flows
%
%   options - options for the solver see odeset f.ex.
%       options = odeset('RelTol',1e-6);
%
%         Example: Bouncing ball with Lite HyEQ Solver
% 
%         % Consider the hybrid system model for the bouncing ball with data given in
%         % Example 1.2. For this example, we consider the ball to be bouncing on a
%         % floor at zero height. The constants for the bouncing ball system are
%         % $\gamma=9.81$ and $\lambda=0.8$. The following procedure is used to
%         % simulate this example in the Lite HyEQ Solver:
% 
%         % * Inside the MATLAB script run_ex1_2.m, initial conditions, simulation
%         % horizons, a rule for jumps, ode solver options, and a step size
%         % coefficient are defined. The function HyEQsolver.m is called in order to
%         % run the simulation, and a script for plotting solutions is included.
%         % * Then the MATLAB functions f_ex1_2.m, C_ex1_2.m, g_ex1_2.m, D_ex1_2.m
%         % are edited according to the data given below.
%         % * Finally, the simulation is run by clicking the run button in
%         % run_ex1_2.m or by calling run_ex1_2.m in the MATLAB command window.
% 
%         % For further information, type in the command window:
%         helpview(['Example_1_2.html']);
% 
%         % Define initial conditions
%         x1_0 = 1;
%         x2_0 = 0;
%         x0   = [x1_0; x2_0];
% 
%         % Set simulation horizon
%         TSPAN = [0 10];
%         JSPAN = [0 20];
% 
%         % Set rule for jumps and ODE solver options
%         %
%         % rule = 1 -> priority for jumps
%         %
%         % rule = 2 -> priority for flows
%         %
%         % set the maximum step length. At each run of the
%         % integrator the option 'MaxStep' is set to
%         % (time length of last integration)*maxStepCoefficient.
%         %  Default value = 0.1
% 
%         rule               = 1;
% 
%         options            = odeset('RelTol',1e-6,'MaxStep',.1);
% 
%         % Simulate using the HyEQSolver script
%         % Given the matlab functions that models the flow map, jump map,
%         % flow set and jump set (f_ex1_2, g_ex1_2, C_ex1_2, and D_ex1_2
%         % respectively)
% 
%         [t j x] = HyEQsolver( @f_ex1_2,@g_ex1_2,@C_ex1_2,@D_ex1_2,...
%             x0,TSPAN,JSPAN,rule,options);
% 
%         % plot solution
% 
%         figure(1) % position
%         clf
%         subplot(2,1,1),plotflows(t,j,x(:,1))
%         grid on
%         ylabel('x1')
% 
%         subplot(2,1,2),plotjumps(t,j,x(:,1))
%         grid on
%         ylabel('x1')
% 
%         figure(2) % velocity
%         clf
%         subplot(2,1,1),plotflows(t,j,x(:,2))
%         grid on
%         ylabel('x2')
% 
%         subplot(2,1,2),plotjumps(t,j,x(:,2))
%         grid on
%         ylabel('x2')
% 
%         % plot hybrid arc
% 
%         plotHybridArc(t,j,x)
%         xlabel('j')
%         ylabel('t')
%         zlabel('x1')
% 
%         % plot solution using plotHarc and plotHarcColor
% 
%         figure(4) % position
%         clf
%         subplot(2,1,1), plotHarc(t,j,x(:,1));
%         grid on
%         ylabel('x_1 position')
%         subplot(2,1,2), plotHarc(t,j,x(:,2));
%         grid on
%         ylabel('x_2 velocity')
% 
% 
%         % plot a phase plane
%         figure(5) % position
%         clf
%         plotHarcColor(x(:,1),j,x(:,2),t);
%         xlabel('x_1')
%         ylabel('x_2')
%         grid on
%
%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @ Hybrid Dynamics and Control Lab, 
% http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
% http://hybridsimulator.wordpress.com/
% Filename: HyEQsolver.m
%--------------------------------------------------------------------------
%   See also plotflows, plotHarc, plotHarcColor, plotHarcColor3D,
%   plotHybridArc, plotjumps.
%   Copyright @ Hybrid Dynamics and Control Lab,
%   Revision: 0.0.0.1 Date: 04/23/2014 10:48:24


if ~exist('rule','var')
    rule = 1;
end

if ~exist('options','var')
    options = odeset();
end

% simulation horizon
tstart = TSPAN(1);
tfinal = TSPAN(end);

% simulate
options = odeset(options,'Events',@(t,x) zeroevents(x,C,D,rule));
tout = tstart;
xout = x0.';
jout = JSPAN(1);
j = jout(end);

% Jump if jump is prioritized:
if rule == 1
    while (j<JSPAN(end))
        % Check if value it is possible to jump current position
        insideD = D(xout(end,:).');
        if insideD == 1
            [j tout jout xout] = jump(g,j,tout,jout,xout);
        else
            break;
        end
    end
end
fprintf('Completed: %3.0f%%',0);
while (j < JSPAN(end) && tout(end) < TSPAN(end))
    % Check if it is possible to flow from current position
    insideC = C(xout(end,:).');
    if insideC == 1
        [t,x] = ode45(@(t,x) f(x),[tout(end) tfinal],xout(end,:).', options);
        nt = length(t);
        tout = [tout; t];
        xout = [xout; x];
        jout = [jout; j*ones(1,nt)'];
    end
    
    %Check if it is possible to jump
    insideD = D(xout(end,:).');
    if insideD == 0
        break;
    else
        if rule == 1
            while (j<JSPAN(end))
                % Check if it is possible to jump from current position
                insideD = D(xout(end,:).');
                if insideD == 1
                    [j tout jout xout] = jump(g,j,tout,jout,xout);
                else
                    break;
                end
            end
        else
            [j tout jout xout] = jump(g,j,tout,jout,xout);
        end
    end
    fprintf('\b\b\b\b%3.0f%%',max(100*j/JSPAN(end),100*tout(end)/TSPAN(end)));
end
t = tout;
x = xout;
j = jout;
fprintf('\nDone\n');
end

function [value,isterminal,direction] = zeroevents(x,C,D,rule )
isterminal = 1;
direction = -1;
insideC = C(x);
if insideC == 0
    % Outside of C
    value = 0;
elseif (rule == 1)
    % If priority for jump, stop if inside D
    insideD = D(x);
    if insideD == 1
        % Inside D, inside C
        value = 0;
    else
        % outside D, inside C
        value = 1;
    end
else
    % If inside C and not priority for jump or priority of jump and outside 
    % of D
    value = 1;
end
end

function [j tout jout xout] = jump(g,j,tout,jout,xout)
% Jump
j = j+1;
y = g(xout(end,:).');
% Save results
tout = [tout; tout(end)];
xout = [xout; y.'];
jout = [jout; j];
end
