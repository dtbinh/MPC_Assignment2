clc, clear

syms q r p x1 x2 a11 a12 a13 a21 a22 a23 a31 a32 a33 b11 b12 b21 b22 b31 b32
A = [a11, a12, a13; a21, a22, a23; a31, a32, a33];
B = [b11, b12; b21, b22; b31, b32];
xk = [x1; x2];
Q = [q, 0; 0, q];
R = [r, 0; 0, r];
Pf = [p, 0; 0, p];

n = 3;
m = 2;

N = 10;
M = 3;

Aeq = [[kron(eye(N), eye(n)) + kron([zeros(1,N); [eye(N-1), zeros(N-1, 1)]], -A)], [kron(eye(M), -B); [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]]];

Aeq
% size(a)
% size(b)
% kron(eye(M), B)
% [zeros((N-M)*n, m*(M-1)), kron(ones(N-M, 1), -B)]





