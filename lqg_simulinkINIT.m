clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; 

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C=[1 0 0 0;
   0 0 1 0]; 

D = zeros(size(C,1),size(B,2));

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = .0001;
K = lqr(A,B,Q,R);  

