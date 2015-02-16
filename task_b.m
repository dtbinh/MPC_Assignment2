clc, clear variables

h = 1; % sampling time in minutes

A = diag([0.5, 0.6, 0.5, 0.6]);
% B = [diag([0.5, 0.4]); diag([0.25, 0.6])]
B = [0.5, 0, 2.5, 0].';


C = [1, 1, 0, 0;
     0, 0, 1, 1];


I = eye(4);
H = [1, 0; 0, 1]; 
Cz = H*C;

zsp = [1, -1]';

Ho = C.'*C
f = 2*zsp.'*C


% Aeq = [I-A, -B; H*C, zeros(2,1)]
% beq = [zeros(4,1); zsp];
Aeq = H*C;
beq = zsp;

options = optimset('Algorithm','active-set','Display','off');

x_opt = quadprog(Ho, f, [], [], Aeq, beq,[],[],[],options);
