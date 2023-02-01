function xdot = pendulum(t, x, K, parameters)
    % extracting parameters
    M = parameters.M;
    m = parameters.m;
    g = parameters.g;
    l = parameters.l;
    
    % extracting x elements
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);
    u = -K*x;
    
    % computations
    num1 = -m*l*sin(x3)*(x4^2) + m*g*sin(x3)*cos(x3) + u;
    num2 = -m*l*sin(x3)*cos(x3)*(x4^2) + (M+m)*g*sin(x3) + u*cos(x3);
    denom = M + m*(sin(x3)^2);
    
    xdot(1) = x2;
    xdot(2) = num1/denom;
    xdot(3) = x4;
    xdot(4) = num2/(l*denom);
    
    xdot = xdot';