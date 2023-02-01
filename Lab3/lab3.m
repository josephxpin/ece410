%% Output 1:
% print the symbolic forms of the A and B matrices

% declare symbolic variables
syms x1 x2 x3 x4 u t M m l g real;
syms xdot [4 1];

% declare parameters of the system
parameters = struct('M',1.0731,'m',0.2300,'l',0.3302,'g',9.8);

% establish x variable
x = [x1;x2;x3;x4];

% intermediate numerator and denominator variables
num1 = -m*l*sin(x3)*(x4^2) + m*g*sin(x3)*cos(x3) + u;
num2 = -m*l*sin(x3)*cos(x3)*(x4^2) + (M+m)*g*sin(x3) + u*cos(x3);
denom = M + m*(sin(x3)^2);

% construct the functions
f2 = num1 / denom;
f4 = num2 / (l*denom);

% generate A and B
A = jacobian([ x2 ; f2 ; x4 ; f4 ],x);
B = jacobian([ x2 ; f2 ; x4 ; f4 ],u);

% substitute x to get the symbolic matrix
% linearize about x = 0
A = subs(A,[x1 x2 x3 x4],[0 0 0 0])
B = subs(B,[x1 x2 x3 x4],[0 0 0 0])

% substitute in our parameters
A = subs(A,[M m l g],[parameters.M parameters.m parameters.l parameters.g]);
B = subs(B,[M m l g],[parameters.M parameters.m parameters.l parameters.g]);

%% Output 2:
% Print the gain vectors K1 and K2
% Produce the given figure with five subplots

% Construct Kalman's Controllability Matrix
% rank of Qc must be 4 to prove (A,B) is controllable
Qc = [B A*B (A^2)*B (A^3)*B];
rank_of_Qc = rank(Qc)

% set Tspan of times for plotting
Tspan = linspace(0,25,1e3);

% set tolerances for plotting - to be used for the entire lab
options = odeset('RelTol',1e-7,'AbsTol',1e-7);

% define the desired eigenvalues
p1 = [-1 -2 -3 -4];

% define initial condition
x0 = [-0.5; 0; -pi/4; 0];

% generate K1
% A and B must be converted to doubles for this to work
K1 = place(double(A),double(B),p1)

%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K1,parameters);

% compute output u with the gain matrix K1
% since x was transposed to get it as ode45 output, we must retranspose it
% we then transpose again because to plot we must have a column vector
u = transpose(-K1*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig1 = figure;
sgtitle("Output 2 - Different controller responses using Pole Placement")
subplot(511),plot(t,x1,'color', 'g'),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2,'color', 'g'),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3,'color', 'g'),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4,'color', 'g'),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,u,'color', 'g'),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% define the desired eigenvalues
p2 = [-1 -2 -3 -20];

% generate K2
K2 = place(double(A),double(B),p2)

%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K2,parameters);

u = transpose(-K2*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
subplot(511),plot(t,x1,'color', 'r')
legend("Response using controller K1","Response using controller K2")
subplot(512),plot(t,x2,'color', 'r')
legend("Response using controller K1","Response using controller K2")
subplot(513),plot(t,x3,'color', 'r')
legend("Response using controller K1","Response using controller K2")
subplot(514),plot(t,x4,'color', 'r')
legend("Response using controller K1","Response using controller K2")

% plot u
subplot(515),plot(t,u,'color', 'r')
legend("Response using controller K1","Response using controller K2")

sgtitle("Output 2 - Control responses using Pole Placement")

%% Output 3.1:
% Produce the figure with five subplots given in the handout

% define Q and R as specified in the lab handout
q2 = 5;
R = 0.5;

% establish q1's as specified in the lab handout
q1_1 = 0.1;
q1_2 = 0.005;

% construct Q's 1 and 2
Q1 = [ q1_1 0 0 0 ; 0 0 0 0 ; 0 0 q2 0 ; 0 0 0 0 ];
Q2 = [ q1_2 0 0 0 ; 0 0 0 0 ; 0 0 q2 0 ; 0 0 0 0 ];

% compute via lqr command the K's
K3 = lqr(double(A),double(B),Q1,R);
K4 = lqr(double(A),double(B),Q2,R);

% for K3:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K3,parameters);

% compute output u with the gain matrix K3
u = transpose(-K3*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig2 = figure;
subplot(511),plot(t,x1,'color', 'g'),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2,'color', 'g'),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3,'color', 'g'),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4,'color', 'g'),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,u,'color', 'g'),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% for K4:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K4,parameters);

% compute output u with the gain matrix K4
u = transpose(-K4*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
subplot(511),plot(t,x1,'color', 'r')
legend("q1 = 0.1","q1 = 0.005")
subplot(512),plot(t,x2,'color', 'r')
legend("q1 = 0.1","q1 = 0.005")
subplot(513),plot(t,x3,'color', 'r')
legend("q1 = 0.1","q1 = 0.005")
subplot(514),plot(t,x4,'color', 'r')
legend("q1 = 0.1","q1 = 0.005")

% plot control signal u
subplot(515),plot(t,u,'color', 'r')
legend("q1 = 0.1","q1 = 0.005")

sgtitle("Output 3.1 - Control responses when modulating q1")

%% Output 3.2:
% Produce the figure with five subplots given in the handout

% define Q and R as specified in the lab handout
q1 = 0.05;
R = 0.5;

% establish q2's as specified in the lab handout
q2_1 = 1;
q2_2 = 2000;

% construct Q's 1 and 2
Q1 = [ q1 0 0 0 ; 0 0 0 0 ; 0 0 q2_1 0 ; 0 0 0 0 ];
Q2 = [ q1 0 0 0 ; 0 0 0 0 ; 0 0 q2_2 0 ; 0 0 0 0 ];

% compute via lqr command the K's
K3 = lqr(double(A),double(B),Q1,R);
K4 = lqr(double(A),double(B),Q2,R);

% for K3:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K3,parameters);

% compute output u with the gain matrix K3
u = transpose(-K3*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig3 = figure;
subplot(511),plot(t,x1,'color', 'g'),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2,'color', 'g'),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3,'color', 'g'),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4,'color', 'g'),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,u,'color', 'g'),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% for K4:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K4,parameters);

% compute output u with the gain matrix K4
u = transpose(-K4*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
subplot(511),plot(t,x1,'color', 'r')
legend("q2 = 1","q2 = 2000")
subplot(512),plot(t,x2,'color', 'r')
legend("q2 = 1","q2 = 2000")
subplot(513),plot(t,x3,'color', 'r')
legend("q2 = 1","q2 = 2000")
subplot(514),plot(t,x4,'color', 'r')
legend("q2 = 1","q2 = 2000")

% plot control signal u
subplot(515),plot(t,u,'color', 'r')
legend("q2 = 1","q2 = 2000")

sgtitle("Output 3.2 - Control responses when modulating q2")

%% Output 3.3:
% Produce the figure with five subplots given in the handout

% define q1/q2 as specified in the lab handout
q1 = 0.05;
q2 = 5;

% establish R's as specified in the lab handout
R1 = 0.005;
R2 = 10;

% construct Q
Q = [ q1 0 0 0 ; 0 0 0 0 ; 0 0 q2 0 ; 0 0 0 0 ];

% compute via lqr command the K's
K3 = lqr(double(A),double(B),Q,R1);
K4 = lqr(double(A),double(B),Q,R2);

% for K3:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K3,parameters);

% compute output u with the gain matrix K3
u = transpose(-K3*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig4 = figure;
subplot(511),plot(t,x1,'color', 'g'),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2,'color', 'g'),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3,'color', 'g'),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4,'color', 'g'),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,u,'color', 'g'),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% for K4:
%Simulate the linearized system
clear x;
[t,x] = ode45(@pendulum,Tspan,x0,options,K4,parameters);

% compute output u with the gain matrix K4
u = transpose(-K4*x');

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
subplot(511),plot(t,x1,'color', 'r')
legend("R = 0.005","R = 10")
subplot(512),plot(t,x2,'color', 'r')
legend("R = 0.005","R = 10")
subplot(513),plot(t,x3,'color', 'r')
legend("R = 0.005","R = 10")
subplot(514),plot(t,x4,'color', 'r')
legend("R = 0.005","R = 10")

% plot control signal u
subplot(515),plot(t,u,'color', 'r')
legend("R = 0.005","R = 10")

sgtitle("Output 3.3 - Control responses when modulating R")

%% Output 4.1:
% Produce the comparison plots from the lab handout
% Produce the figure requested in the lab handout

% setting Q and R
Q = [0.05 0 0 0 ; 0 0 0 0 ; 0 0 5 0 ; 0 0 0 0 ];
R = 0.005;

% re-establish x
clear x x1 x2 x3 x4 u t;
syms x1 x2 x3 x4 u t;
x = [x1;x2;x3;x4];

% compute K
K = lqr(double(A),double(B),Q,R);

% getting symbolic function f put together
f = [x2;f2;x4;f4];

% substitute for u as prescribed
f = subs(f,u,K*x);
f = subs(f,M,parameters.M);
f = subs(f,m,parameters.m);
f = subs(f,l,parameters.l);
f = subs(f,g,parameters.g);

% using matlabFunction to obtain the function
fun = matlabFunction(f,'Vars',{t,x});

% set x0
x0 = [-1;0;pi/4;0];

% solve using ode45
[t,x] = ode45(fun,Tspan,x0);

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig5 = figure;
subplot(511),plot(t,x1,'color', 'g'),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2,'color', 'g'),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3,'color', 'g'),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4,'color', 'g'),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,transpose(K*x'),'color', 'g'),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% now use the linear system
[t,x] = ode45(@pendulum,Tspan,x0,options,K,parameters);

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
subplot(511),plot(t,x1,'color', 'r')
legend("Non-linear system","Linear system")
subplot(512),plot(t,x2,'color', 'r')
legend("Non-linear system","Linear system")
subplot(513),plot(t,x3,'color', 'r')
legend("Non-linear system","Linear system")
subplot(514),plot(t,x4,'color', 'r')
legend("Non-linear system","Linear system")

% plot control signal u
subplot(515),plot(t,transpose(K*x'),'color', 'r')
legend("Non-linear system","Linear system")

sgtitle("Output 4.1 - Non-Linear System vs. Linear System")

%% Output 4.2:
% decrease y continuously until the system fails to converge to equilibrium

% re-establish x
clear x x1 x2 x3 x4 u t;
syms x1 x2 x3 x4 u t;
x = [x1;x2;x3;x4];

% set x0
x0 = [-2;0;pi/4;0];

% compute K
K = lqr(double(A),double(B),Q,R);

% getting symbolic function f put together
f = [x2;f2;x4;f4];

% substitute for u as prescribed
f = subs(f,u,K*x);
f = subs(f,M,parameters.M);
f = subs(f,m,parameters.m);
f = subs(f,l,parameters.l);
f = subs(f,g,parameters.g);

% using matlabFunction to obtain the function
fun = matlabFunction(f,'Vars',{t,x});

% solve using ode45
[t,x] = ode45(fun,Tspan,x0);

% extract x elements
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);

% plot x elements
fig6 = figure;
subplot(511),plot(t,x1),xlabel("time [s]"),ylabel("x1")
title("State variable x1 over time")
hold on
subplot(512),plot(t,x2),xlabel("time [s]"),ylabel("x2")
title("State variable x2 over time")
hold on
subplot(513),plot(t,x3),xlabel("time [s]"),ylabel("x3")
title("State variable x3 over time")
hold on
subplot(514),plot(t,x4),xlabel("time [s]"),ylabel("x4")
title("State variable x4 over time")
hold on

% plot control signal u
subplot(515),plot(t,transpose(K*x')),xlabel("time [s]"),ylabel("u")
title("Control Signal")
hold on

% for loop will continue to decrease y until we hit 50
for c = 5:5:50
    % set x0 using for loop index
    x0 = [-c;0;pi/4;0];

    % solve using ode45
    [t,x] = ode45(fun,Tspan,x0);

    % extract x elements
    x1 = x(:,1);
    x2 = x(:,2);
    x3 = x(:,3);
    x4 = x(:,4);

    % plot x elements
    subplot(511),plot(t,x1)
    hold on
    subplot(512),plot(t,x2)
    hold on
    subplot(513),plot(t,x3)
    hold on
    subplot(514),plot(t,x4)
    hold on

    % plot control signal u
    subplot(515),plot(t,transpose(K*x'))
    hold on
end

sgtitle("Output 4.2 - Non-Linear System with decreasing x1 initial condition")
