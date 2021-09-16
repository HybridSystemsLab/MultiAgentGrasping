% function time = eta_check(z1_0, z2_0, a, b, maxT, z1_d)
function time = eta_check2(z1_0,z1_d,v)

z_0 = z1_0 - z1_d;

time = 0;

tspan = [0 5];
odeset('RelTol',1e-6,'MaxStep',.1);
[t,z] = ode45(@(t,z) odefcn2(t,z,v), tspan, z_0);

for i = 1:length(z)
    if((time == 0) && (z(i) >= -z1_d))
        time = t(i);
%         time
        break;
    end
end


end

