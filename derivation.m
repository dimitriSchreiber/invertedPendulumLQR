clear; close all;
syms theta_dd theta_d theta x_dd x_d x f m g l M
theta_dd = (f*cos(theta) + (M+m)*g*sin(theta) + l*m*theta_d^2*sin(theta)*cos(theta)) / (l*(M+m*sin(theta)^2))
x_dd = (f + m*sin(theta)*g*cos(theta) + m*sin(theta)*l*theta^2) / (M + m*sin(theta)^2)
X_bar_d = [x_d; x_dd; theta_d; theta_dd]

%this linearizing X_bar_d around X_bar = [0, 0, 0, 0]'
A_linear = jacobian(X_bar_d, [x, x_d, theta, theta_d])
A = subs(A_linear, [x, x_d, theta, theta_d], [0, 0, 0, 0])
B_linear = jacobian(X_bar_d, [f])
B = subs(B_linear, [x, x_d, theta, theta_d], [0, 0, 0, 0])

%my jacobians do not match https://www.youtube.com/watch?v=qjhAAQexzLg&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=12
%they do match: https://danielpiedrahita.wordpress.com/portfolio/cart-pole-control/

%check eigenvalues of A, set cartpole parameters
Ad = subs(A, [m, M, l, g], [1, 5, 2, -10])
Bd = subs(B, [m, M, l, g], [1, 5, 2, -10])
Ad = double(Ad)
Bd = double(Bd)
eig(Ad)

%check if this system is controllable
controlabillity = ctrb(Ad, Bd)
rankctrb = rank(controlabillity)

%place eigenvalues, from lecture videos
p = [-.3; -.4; -.5; -.6];  % just barely
p = [-1; -1.1; -1.2; -1.3]; % good
p = [-2; -2.1; -2.2; -2.3]; % aggressive
p = [-3; -3.1; -3.2; -3.3]; % aggressive
K = place(Ad,Bd,p);

%confirm all stable eigenvalues
eig(Ad - Bd*K)

%grab from lecture code:
% K = lqr(A,B,Q,R);

%[m, M, l, g] = [1, 5, 2, -10]
m = 1
M = 5
L = 2
d = 0
g = -10
s = 1
A = Ad
B = Bd

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
%     [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,0,-K*(y-[1; 0; pi; 0])),tspan,y0);
else 
end

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end
