clc, clear

syms q r p x1 x2 a1 a12 a2 b1 b12 b2
A = [a1, a12; a12, a2];
B = [b1, b12; b12, b2];
xk = [x1; x2];
Q = [q, 0; 0, q];
R = [r, 0; 0, r];
Pf = [p, 0; 0, p];

n = 2;
m = 2;

N = 10;
M = 3;

% AA = [A; zeros((N-1)*n,n)]
% beq = AA*xk
H = [[kron(eye(N-1), Q), zeros((N-1)*n, n); zeros(n, (N-1)*n), Pf], zeros(N*n, M*m); zeros(M*m, N*n), kron(eye(M), R)];

% Aeq = [[kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)], [kron(eye(M), B); [zeros((M)*n, m*(M-1)), kron(ones(M, 1), -B)]]];

a = [kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)];
b = [kron(eye(M), B); [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]];

Aeq = [[kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)], [kron(eye(M), B); [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]]];

[a,b]
% size(a)
% size(b)
% kron(eye(M), B)
% [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]





