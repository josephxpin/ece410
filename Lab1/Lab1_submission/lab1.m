% clear variables and closes figures
clear
close all force

% establishing parameters in struct
parameters = struct('M',0.2,'l',0.15,'g',9.81);

% initial conditions
x0_1 = [0;(sqrt((parameters.g/parameters.l)))];
x0_2 = [0;(1.99*sqrt((parameters.g/parameters.l)))];

% setting tolerances
options = odeset('RelTol',1e-7,'AbsTol',1e-7);

% setting Tspan of times
Tspan = linspace(0,10,1e3);

% numerical integration of ODE - using init condition 1
[t,x] = ode45(@pendulum,Tspan,x0_1,options,parameters);

% extract x elements
x1 = x(:,1);
x2 = x(:,2);


% plotting x elements
f1 = figure;
subplot(211),plot(t,x1),xlabel("time [s]"),ylabel("x1")
title("State variable x over time")
hold on
subplot(212),plot(t,x2),xlabel("time [s]"),ylabel("x2")
hold on

% plotting orbit
f2 = figure;
title("Orbit of state variable x")
hold on
plot(x1,x2),xlabel("x1"),ylabel("x2")
hold on

% numerical integration of ODE - using init condition 2
[t,x] = ode45(@pendulum,Tspan,x0_2,options,parameters);

% extract x elements
x3 = x(:,1);
x4 = x(:,2);

% plotting time functions
f3 = figure;
subplot(211),plot(t,x3),xlabel("time (t)"),ylabel("x1")
title("State variable x over time")
hold on
subplot(212),plot(t,x4),xlabel("time (t)"),ylabel("x2")
hold on

% plotting orbit
f4 = figure;
title("Orbit of state variable x")
hold on
plot(x3,x4),xlabel("x1"),ylabel("x2")
hold on

% symbolic linearlization
syms x1 x2 t u m l g real;
syms xdot [2 1];

x = [ x1 ; x2 ];

% Jacobian computation to get matrices A, B, C, D
A = jacobian( [ (x2) ; -(g/l)*sin(x1)-((cos(x1)*u)/(m*l)) ],x);
B = jacobian( [ (x2) ; -(g/l)*sin(x1)-((cos(x1)*u)/(m*l)) ],u);
C = jacobian (x1,x);
D = jacobian (x1,u);

% initial conditions
A = subs(A,x,[0;0]);
A = subs(A,u,0)

B = subs(B,x,[0;0]);
B = subs(B,u,0)

C = subs(C,x,[0;0]);
C = subs(C,u,0)

D = subs(D,x,[0;0]);
D = subs(D,u,0)

A_sym = A;
B_sym = B;
C_sym = C;
D_sym = D;

% %linearize about generic equilibrium
clear x1 x2 t u m l g xdot A B C D;

syms x1 x2 t u m l g theta real;
syms xdot [2 1];

x = [ x1 ; x2 ];

% Jacobian computation to get matrices A, B, C, D
A = jacobian( [ (x2) ; -(g/l)*sin(x1)-((cos(x1)*u)/(m*l)) ],x);
B = jacobian( [ (x2) ; -(g/l)*sin(x1)-((cos(x1)*u)/(m*l)) ],u);
C = jacobian (x1,x);
D = jacobian (x1,u);


% initial conditions
A = subs(A,x,[theta;0]);
A = subs(A,u,-m*g*tan(theta));
A = simplify(A)

B = subs(B,x,[theta;0]);
B = subs(B,u,-m*g*tan(theta));
B = simplify(B)

% Creating column vector z
syms z1 z2;
xdot = [x2; -(g/l)*sin(x1)-((cos(x1)*u)/(m*l))];
z = [z1; z2];

% Setting up linearization
zdot = (A_sym*(z - [0;0])) + (B_sym*(u - 0));
Xdot = [xdot; zdot];
Xdot = subs(Xdot, m, parameters.M);
Xdot = subs(Xdot, l, parameters.l);
Xdot = subs(Xdot, g, parameters.g);
Xdot = subs(Xdot, theta, x1);
Xdot = subs(Xdot, u, 0);

% Setting up pendulum output function
augmented_pend = matlabFunction(Xdot, 'Vars', {t, [x;z]});

% Numerically integrating the ODE
[tret, xret] = ode45(augmented_pend,Tspan,[x0_1;x0_1],options);

% Plots of x compared to its linearized representation z for x0_1
f5 = figure;
subplot(211);
plot(tret,xret(:,1))
hold on
plot(tret,xret(:,3))
xlabel("time [s]")
ylabel("x1/z1")
legend("X","Z")
title("State variable x1 and z1 over time")
hold on

subplot(212)
plot(tret,xret(:,2))
hold on
plot(tret,xret(:,4))
xlabel("time [s]")
ylabel("x2/z2")
legend("X","Z")
title("State variable x2 and z2 over time")

f6 = figure;
plot(xret(:,1), xret(:,2))
hold on
plot(xret(:,3), xret(:,4))
xlabel("x1/z1")
ylabel("x2/z2")
legend("X","Z")
title("Overlayed orbitals of x and z")

% Plots of x compared to its linearized representation z for x0_2

[tret, xret] = ode45(augmented_pend,Tspan,[x0_2;x0_2],options);
f7 = figure;
subplot(211);
plot(tret,xret(:,1))
hold on
plot(tret,xret(:,3))
xlabel("time [s]")
ylabel("x1/z1")
legend("X","Z")
title("State variable x1 and z1 over time (init conditions 2)")
hold on

subplot(212)
plot(tret,xret(:,2))
hold on
plot(tret,xret(:,4))
xlabel("time [s]")
ylabel("x2/z2")
legend("X","Z")
title("State variable x2 and z2 over time")
%hold on

f8 = figure;
plot(xret(:,1), xret(:,2))
hold on
plot(xret(:,3), xret(:,4))
xlabel("x1/z1")
ylabel("x2/z2")
legend("X","Z")
title("Overlayed orbitals of x and z")

% PART 6: LTI Representations

A_d = subs(A_sym, m, parameters.M);
A_d = subs(A_d, g, parameters.g);
A_d = subs(A_d, l, parameters.l);
A_d = subs(A_d, theta, x1);
A_d = double(A_d);

B_d = subs(B_sym, m, parameters.M);
B_d = subs(B_d, g, parameters.g);
B_d = subs(B_d, l, parameters.l);
B_d = subs(B_d, theta, x1);
B_d = double(B_d);

C_d = subs(C_sym, m, parameters.M);
C_d = subs(C_d, g, parameters.g);
C_d = subs(C_d, l, parameters.l);
C_d = subs(C_d, theta, x1);
C_d = double(C_d);

D_d = subs(D_sym, m, parameters.M);
D_d = subs(D_d, g, parameters.g);
D_d = subs(D_d, l, parameters.l);
D_d = subs(D_d, theta, x1);
D_d = double(D_d);

LTI = ss(A_d, B_d, C_d, D_d);
transfer_func = zpk(LTI);
P = pole(LTI);
e = eig(A_d);

% Output information on LTI system 
disp("Transfer Function of System: ")
transfer_func
disp("Poles of System: ")
P
disp("Eigen Values of System: ")
e

% Output 5 - Controller design
s = 2

C = tf([-30 -300],[1 1000])
tf_Zeros = zero(zpk(1+C*LTI))

[F,G,H,L]=ssdata(C);

x0_fin = [pi/4 0];

parameters = struct('M',0.2,'l',0.15,'g',9.81,'F',F);

[tfin,xfin]=ode45(@controlled_pendulum,Tspan,x0_fin,options,parameters);

% extract x elements
x1_fin = xfin(:,1);
x2_fin = xfin(:,2);


% plotting x elements
f9 = figure;
subplot(211),plot(t,x1_fin),xlabel("time [s]"),ylabel("x1")
title("State variable x over time")
hold on
subplot(212),plot(t,x2_fin),xlabel("time [s]"),ylabel("x2")
hold on
