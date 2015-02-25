%==========================================================================
% SSY280 Model Predictive Control 2012
%
% Homework Assignment 2:
% MPC Control of a Linearized MIMO Well Stirred Chemical Reactor 
% Revised 2013-02-10
%==========================================================================

%******************* Initialization block *********************************

clear;
% close all
clc
tf=50;                  % number of simulation steps

%==========================================================================
% Process model
%==========================================================================

h = 1; % sampling time in minutes

A = [ 0.2681   -0.00338   -0.00728;
      9.7032    0.3279   -25.44;
         0         0       1   ];
B = [ -0.00537  0.1655;
       1.297   97.91;
       0       -6.637];
C = [ 1 0 0;
      0 1 0;
      0 0 1];
Bp = [-0.1175;
      69.74;
       6.637];
   
n = size(A,1); % n is the dimension of the state
m = size(B,2); % m is the dimension of the control signal
p = size(C,1); % p is the dimension of the measured output

d = 0.01*[zeros(1*tf/5,1); ones(4*tf/5,1)]'; %unmeasured disturbance trajectory

x0 = [0.01;1;0.1]; % initial condition of system's state

%==========================================================================
% Set up observer model
%==========================================================================

% Three cases to be investigated

example = 'b';
switch example
    case 'a'
        nd = 2;
        Bd = zeros(n,nd);
        Cd = [1 0;0 0; 0 1]; 
    case 'b'
        nd = 3;
        Bd = zeros(n,nd); 
        Cd = [1 0 0;0 0 1;0 1 0];
    case 'c'
        nd = 3; 
        Bd = [zeros(3,2) Bp];
        Cd = [1 0 0;0 0 0;0 1 0];
end

% Augment the model with constant disturbances

Ae = [A, Bd; zeros(nd,n), eye(nd)];
Be = [B;zeros(nd + n - size(B, 1), m)];
Ce = [C Cd];
O = obsv(Ae, Ce);

Or = rank(O);

if example == 'c'
    Bdist = [zeros(n,nd); eye(nd)];
    SYS = ss(Ae,[Be Bdist],Ce,[],60);
else
    SYS = ss(Ae, Be, Ce, zeros(size(Ce, 1), size(Be, 2)), 60);
end
% Calculate observer gain 

Qn = eye(2)*10000;
Rn = eye(3);
[KEST, Le, P] = kalman(SYS, Qn, Rn, 'delayed');


% eig(Ae - Le*Ce);

%==========================================================================
% Prepare for computing steady state targets
%==========================================================================

% Select 1st and 3rd outputs as controlled outputs   
H = [1, 0, 0; 0, 0, 1];
Cz = H*C;
% Matrices for steady state target calculation to be used later

As = [eye(3)-A, -B;
      Cz, zeros(2,2)];
Adet = [eye(3)-A, -Bd;
      C, Cd];
    
n+nd
rank(Adet)

zsp = [0; 0];



%==========================================================================
% Set up MPC controller
%==========================================================================

N = 10;                   % prediction horizon
M = 3;                    % control horizon

Q = diag([1 0.001 1]);  % state penalty
Pf = Q;                 % terminal state penalty
R = 0.01*eye(m);        % control penalty

%=================================
% Build Hessian Matrix
%=================================

Hm = [[kron(eye(N-1), Q), zeros((N-1)*n, n); zeros(n, (N-1)*n), Pf], zeros(N*n, M*m); zeros(M*m, N*n), kron(eye(M), R)];

%==========================================
% Equality Constraints
%==========================================

Aeq = [[kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)], [kron(eye(M), -B); [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]]];
AA = [A; zeros((N-1)*n,n)];

%==========================================
% Inequality Constraints
%==========================================

Ain = [];
Bin = [];

%==============================================
% Choose QP solver 
%==============================================

solver = 'interior-point-convex'; % 'active-set'
options = optimset('Algorithm', solver, 'Display', 'off');


%******************* End of initialization ********************************    

%==========================================================================
% Simulation
%==========================================================================
    
% Initialization

xhat = [x0; zeros(nd,1)];
dhat = zeros(nd,1);
xk = x0;
yk = C*xk;
uk = zeros(m,1);


xvec = x0;
xhatvec = xhat;
uvec = uk;
dhatvec = dhat;
yvec = C*xhat(1:n);
realy = [];
% Simulate closed-loop system 


for k = 1:tf

    %======================================
    % Update the observer state xhat(k|k-1)
    %======================================

    xhat = Ae*xhat + Be*uk + Le*(yk - Ce*xhat);
    dhat = xhat(n+1:end);

    %==============================================
    % Update the process state x(k) and output y(k)
    %==============================================

    xk = A*xk + B*uk + Bp*d(k);
    yk = C*xk;

    %=========================================
    % Calculate steady state targets xs and us
    %=========================================

    bs = [Bd*dhat; -H*Cd*dhat];
    xus = As\bs;
    xs = xus(1:n);
    us = xus((n+1):end);

    %============================================
    % Solve the QP (for the deviation variables!)
    %============================================

    dx = xhat(1:n) - xs;
    beq = AA*dx;

    z = quadprog(Hm,[],Ain,Bin,Aeq,beq,[],[],[],options);
    
    du = z(N*n+1:N*n+m);
    uk = du + us;
    

    %===============================        
    % Store current variables in log 
    %===============================
    xvec = [xvec, xk];
    xhatvec = [xhatvec, xhat];
    uvec = [uvec, uk];
    dhatvec = [dhatvec, dhat];
    yvec = [yvec, C*xhat(1:n)];
    realy = [realy, C*xk];
%     YOUR CODE GOES HERE

end % simulation loop


%==========================================================================
% Plot results
%==========================================================================
figure(1)
clf
subplot(3, 1, 1)
hold on
plot(yvec(1,:), 'b')
plot(realy(1,:), 'b--')
xlabel('Minutes')

legend('Concentration observed', 'Concentration real')
subplot(3, 1, 2)
hold on
plot(yvec(2,:), 'r')
plot(realy(2,:), 'r--')
xlabel('Minutes')

legend('Temperature observed', 'Temperature real')
subplot(3, 1, 3)
hold on
plot(yvec(3,:), 'k')
plot(realy(3,:), 'k--')
xlabel('Minutes')
legend('Tank level observed', 'Tank level real')


figure(2)
clf
subplot(2,1,1)
plot(uvec(1, :), 'g')
xlabel('Minutes')
legend('Control signal 1')
subplot(2,1,2)
plot(uvec(2, :), 'r')
xlabel('Minutes')
legend('Control signal 2')
