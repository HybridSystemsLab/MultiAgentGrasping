function dzdt = odefcn(t,z,a,b)
    dzdt = zeros(2,1);
    dzdt(1) = z(2);
    dzdt(2) = a*z(1) + b*z(2);
end