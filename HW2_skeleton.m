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
tf=50;                  % number of simulation steps

%==========================================================================
% Process model
%==========================================================================

h = 1; % sampling time in minutes

A = [ 0.2681   -0.00338   -0.00728;
      9.7032    0.3279   -25.44;
         0         0       1   ];
B = [ -0.00537  0.1655;
       1.297   97.91 ;
       0       -6.637];
C = [ 1 0 0;
      0 1 0;
      0 0 1];
Bp = [-0.1175;
      69.74;
       6.637 ];
   
n = size(A,1); % n is the dimension of the state
m = size(B,2); % m is the dimension of the control signal
p = size(C,1); % p is the dimension of the measured output

d=0.01*[zeros(1*tf/5,1);ones(4*tf/5,1)]; %unmeasured disturbance trajectory

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
        nd=3;
        Bd = zeros(n,nd); 
        Cd = [1 0 0;0 0 1;0 1 0];
    case 'c'
        nd=3; 
        Bd = [zeros(3,2) Bp];
        Cd = [1 0 0;0 0 0;0 1 0];
end

% Augment the model with constant disturbances

Ae = ...
Be = ...  YOUR CODE GOES HERE
Ce = ...

% Calculate observer gain 

Le = ...  YOUR CODE GOES HERE

%==========================================================================
% Prepare for computing steady state targets
%==========================================================================

% Select 1st and 3rd outputs as controlled outputs   
H = [1 0 0;0 0 1]; 

% Matrices for steady state target calculation to be used later

YOUR CODE GOES HERE

%==========================================================================
% Set up MPC controller
%==========================================================================

N=10;                   % prediction horizon
M=3;                    % control horizon

Q = diag([1 0.001 1]);  % state penalty
Pf = Q;                 % terminal state penalty
R = 0.01*eye(m);        % control penalty
    
    %=================================
    % Build Hessian Matrix
    %=================================

    YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE
    
    %==========================================
    % Equality Constraints
    %==========================================
    
    YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE
    
    %==========================================
    % Inequality Constraints
    %==========================================
    
    Ain = [];
    Bin = [];
    
    %==============================================
    % Choose QP solver 
    %==============================================
    
    solver = 'int';
    switch solver
        case 'int'
            options = optimset('Algorithm','interior-point-convex','Display','off');
        case 'set'
            options = optimset('Algorithm','active-set','Display','off');
    end

%******************* End of initialization ********************************    

%==========================================================================
% Simulation
%==========================================================================
    
% Initialization
    
YOUR CODE GOES HERE - INITIALIZE ALL VARIABLES NEEDED 
    
% Simulate closed-loop system 
    
    for k = 1:tf

        %======================================
        % Update the observer state xhat(k|k-1)
        %======================================
        
        YOUR CODE GOES HERE
        
        %==============================================
        % Update the process state x(k) and output y(k)
        %==============================================
        
        xk = A*xk + B*uk + Bp*d(k);
        yk = C*xk;        
        
        %=========================================
        % Calculate steady state targets xs and us
        %=========================================
        
        YOUR CODE GOES HERE
        
        %============================================
        % Solve the QP (for the deviation variables!)
        %============================================
        
        UPDATE RHS OF EQUALITY CONSTRAINT HERE
        
        % NOTE THAT HM IS USED FOR THE HESSIAN, NOT TO BE CONFUSED 
        %   WITH H THAT IS USED FOR SELECTING CONTROLLED VARIABLES
        z = quadprog(HM,[],Ain,Bin,Aeq,beq,[],[],[],options);
        
        CALCULATE THE NEW CONTROL SIGNAL HERE
        % NOTE THAT YOU NEED TO GO FROM DEVIATION VARIABLES TO 'REAL' ONES!
        
        %===============================        
        % Store current variables in log 
        %===============================
        
        YOUR CODE GOES HERE
        
   end % simulation loop
 
%==========================================================================
% Plot results
%==========================================================================
        
YOUR CODE GOES HERE


