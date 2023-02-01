function xdot = pendulum(t, x, parameters)
    % extracting parameters
    M = parameters.M;
    g = parameters.g;
    l = parameters.l;
    
    % extracting x elements
    x1 = x(1);
    x2 = x(2);
    u = 0;
    
    % computations
    xdot_int = parameters.F*x;
    
    xdot(1) = xdot_int(1)+ x2;
    xdot(2) = xdot_int(2)-(g/l)*sin(x1)-(1/(M*l))*cos(x1)*u;
    
    xdot = transpose(xdot);