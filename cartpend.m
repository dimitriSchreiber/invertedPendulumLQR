function dy = cartpend(y,m,M,L,g,d,u)

Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

dy(1,1) = y(2);
dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
dy(3,1) = y(4);
dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;

%theta_dd = (f*cos(theta) + (M+m)*g*sin(theta) + l*m*theta_d^2*sin(theta)*cos(theta)) / (l*(M+m*sin(theta)^2))
%x_dd = (f + m*sin(theta)*g*cos(theta) + m*sin(theta)*l*theta^2) / (M + m*sin(theta)^2)
%X_bar_d = [x_d; x_dd; theta_d; theta_dd]

%this linearizing X_bar_d around X_bar = [0, 0, 0, 0]'
%A = subs(A_linear, [x, x_d, theta, theta_d], [0, 0, 0, 0])
%B = subs(B_linear, [x, x_d, theta, theta_d], [0, 0, 0, 0])
