K1 = 3.780014812946319580e-03;
K2 = -1.993117621168494225e-03;
K3 = 5.228068857832113281e-07;
P1 = -1.876385213108733296e-04;
P2 = -5.847600405104458332e-04;

pu = [1.0 2 3 4 5 5 4 3 2 1; 1 2 3 4 5 5 4 3 2 1] ./ 10;

function fx(xd, yd)
    rd = xd^2 + yd^2;
    return xd * (1 + K1 * rd + K2 * rd^2 + K3 * rd^3) + P1 * (rd + 2 * xd^2) + 2 * P2 * xd * yd;
end

function fy(xd, yd)
    rd = xd^2 + yd^2;
    return yd * (1 + K1 * rd + K2 * rd^2 + K3 * rd^3) + P2 * (rd + 2 * yd^2) + 2 * P1 * xd * yd;
end

function f(u,p)
    y = [0.0,0.0];
    rd = u[1]^2 + u[2]^2;
    y[1] = fx(u[1], u[2]) - p[1];
    y[2] = fy(u[1], u[2]) - p[2];

    return y;
end

for i in 1:10
    local prob = NonlinearProblem(f, u0, pu[:,i]);
    local solver = solve(prob);
    println(solver);
    #print(fx(solver[1], solver[2]));
    #print(" ");
    #println(fy(solver[1], solver[2]));
end

