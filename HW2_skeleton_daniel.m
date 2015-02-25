%==========================================================================
% SSY280 Model Predictive Control 2012
%
% Homework Assignment 2:
% MPC Control of a Linearized MIMO Well Stirred Chemical Reactor 
% Revised 2013-02-10
%==========================================================================

%******************* Initialization block *********************************

clear;
close all
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

d = 0.01*[zeros(1*tf/5,1); ones(4*tf/5,1)]; %unmeasured disturbance trajectory

x0 = [0.01;1;0.1]; % initial condition of system's state

%==========================================================================
% Set up observer model
%==========================================================================

% Three cases to be investigated

example = 'a';
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

% Calculate observer gain 
SYS = ss(Ae, Be, Ce, zeros(size(Ce, 1), size(Be, 2)), 60);
[KEST, Le, P] = kalman(SYS, eye(2)*0.01, eye(3));



%==========================================================================
% Prepare for computing steady state targets
%==========================================================================

% Select 1st and 3rd outputs as controlled outputs   
H = [1, 0, 0; 0, 0, 1];
Cz = H*C;
% Matrices for steady state target calculation to be used later

As = [eye(3)-A, -B;
      Cz, zeros(2,2)];

dhat = 0.2*ones(nd,1);
zsp = [0; 0];
 
bs = [Bd*dhat; -H*Cd*dhat];
xs = As\bs;


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

%     YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE

Hm = [[kron(eye(N-1), Q), zeros((N-1)*n, n); zeros(n, (N-1)*n), Pf], zeros(N*n, M*m); zeros(M*m, N*n), kron(eye(M), R)];

Aeq = [[kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)], [kron(eye(M), B); [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]]];
AA = [A; zeros((N-1)*n,n)];

%==========================================
% Equality Constraints
%==========================================

%     YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE

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
    
% YOUR CODE GOES HERE - INITIALIZE ALL VARIABLES NEEDED 

xhat = [x0; zeros(nd,1)];
xk = x0;
yk = C*xk;
uk = zeros(m,1);

xvec = [];
uvec = [];
xhatvec = [];
dhatvec = [];
yvec = [];

% Simulate closed-loop system 
    


for k = 1:tf

    %======================================
    % Update the observer state xhat(k|k-1)
    %======================================

    %         YOUR CODE GOES HERE
    xhat = Ae*xhat + Be*uk + Le*(yk - Ce*xhat);
    dhat = xhat((end-nd+1):end);

    %==============================================
    % Update the process state x(k) and output y(k)
    %==============================================

    xk = A*xk + B*uk + Bd*ones(nd,1)*d(k);
    yk = C*xk;

    %=========================================
    % Calculate steady state targets xs and us
    %=========================================


    bs = [Bd*dhat; -H*Cd*dhat];
    xus = As\bs;
    xs = xus(1:n);
    us = xus((n+1):end);

%         YOUR CODE GOES HERE

    %============================================
    % Solve the QP (for the deviation variables!)
    %============================================

%     UPDATE RHS OF EQUALITY CONSTRAINT HERE
    dx = xhat(1:n) - xs;
    beq = AA*dx;
%     beq = AA*xhat(1:n);
        
    z = quadprog(Hm,[],Ain,Bin,Aeq,beq,[],[],[],options);
    
    du = z(N*n+1:N*n+m);
    
    uk = du + us;
        
%     CALCULATE THE NEW CONTROL SIGNAL HERE
    % NOTE THAT YOU NEED TO GO FROM DEVIATION VARIABLES TO 'REAL' ONES!

    %===============================        
    % Store current variables in log 
    %===============================
    xvec = [xvec, xk];
    xhatvec = [xhatvec, xhat];
    uvec = [uvec, uk];
    dhatvec = [dhatvec, dhat];
    yvec = [yvec, yk];
%     YOUR CODE GOES HERE

end % simulation loop

figure(1)
hold on
plot(xvec(1,:))
plot(xhatvec(1,:), 'r--');

% figure(2)

%==========================================================================
% Plot results
%==========================================================================
        
% YOUR CODE GOES HERE