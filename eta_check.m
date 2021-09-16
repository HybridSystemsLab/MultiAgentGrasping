% function time = eta_check(z1_0, z2_0, a, b, maxT, z1_d)
function time = eta_check(kp, kd, x1_0, x2_0, x1_d)
time = 0;

z = [];
% z1 = [];
% z2 = [];
epsilon = 0.01;

z1_0 = x1_0 - x1_d;
z2_0 = x2_0;

z0 = [z1_0; z2_0];

time = 0;

tspan = [0 5];
odeset('RelTol',1e-6,'MaxStep',.1);
[t,z] = ode45(@(t,z) odefcn(t,z,-kp,-kd), tspan, z0);

for i = 1:length(z)
    if((time == 0) && (z(i) >= -x1_d ))
        time = t(i);
%         time
        break;
    end
end


end

