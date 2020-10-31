clc
clear all
close all

%system variables
m = 1;
M = 5;
L = 2;
b = 0.1;
I = 0.006;
g = -9.8;
l=1;
d = 0;
s=tf('s');
q = (M+m)*(I+m*l^2)-(m*l)^2;
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

c=menu('Inverted pendulum','Free oscillation','Controlled oscillation');
if(c==1)
    c1=menu('Type of oscillation','Frictionless oscillation','Damped oscillation');
    if(c1==1)
        d=0;
    else
        d=1;
    end
    tspan = 0:.001:50;
    y0 = [0; 0; pi+0.3; 0];
    [t,y] = ode45(@(t,y)pendcart(y,m,M,L,g,d,0),tspan,y0);
     %Animating pendcart
     for k=1:100:length(t)
         drawcartpend(y(k,:),m,M,L);
     end
figure(2);
subplot(2,2,1);
plot(t,y(:,1))
ylabel('X posn');
xlabel('time');
title('Stabilaisation of x wrt time');

subplot(2,2,2);
plot(t,y(:,2))
ylabel('Velocity');
xlabel('time');
title('Stabilaisation of v wrt time');

subplot(2,2,3);
plot(t,pi-y(:,3))
ylabel('Theta error');
xlabel('time');
title('Stabilaisation of theta wrt time');

subplot(2,2,4);
plot(t,y(:,4))
ylabel('Angular velocity');
xlabel('time');
title('Stabilaisation of w wrt time');
else
    ch=menu('Control System: ','P','PI','PD','PID','LQR');
    
if(ch==1)
    kp=45;
    K=pid(kp);
    %display(P_pend);
    %display(K);
    TP=feedback(K*P_pend,1)
    %display(TP);
    t=0:0.01:10;
    figure(1);
    impulse(TP,t)
    title({'Response of Pendulum Position to an Impulse Disturbance';'under P Control'});
    
elseif(ch==2)
    kp=100;
    ki=5;
    K=pid(kp,ki);
    %display(P_pend)
    %display(K)
    TPI=feedback(K*P_pend,1)
    t=0:0.01:10;
    figure(1);
    impulse(TPI,t)
    title({'Response of Pendulum Position to an Impulse Disturbance';'under PI Control'});
        
elseif(ch==3)
    kp=100;
    kd=2;
    K=pid(kp,0,kd);
    TPD=feedback(K*P_pend,1)
    figure(1);
    impulse(TPD)
    title({'Response of Pendulum Position to an Impulse Disturbance';'under PD Control'});
    
elseif(ch==4)
    Kp = 350;
    Ki = 300;
    Kd = 50;
    C = pid(Kp,Ki,Kd);
    T = feedback(C*P_pend,1);
    figure(1);
    subplot(211);
    impulse(T)
    title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control'});
    P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
    T2 = feedback(1,P_pend*C)*P_cart;
    t = 0:0.01:6;
    subplot(212)
    impulse(T2);
    title('Impulse Disturbance Response of Cart Position under PID Control');
    
elseif(ch==5)
A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -d/(M*L) -(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; 1/(M*L)];
% eig(A)

%cost matrix
Q = [1 0 0 0;
    0 1 0 0;
    0 0 500 0;
    0 0 0 250];
R = 0.001;

%checking for stability
rank(ctrb(A,B))

%negative feedback
K = lqr(A,B,Q,R);

% t_f=K*inv((s*eye(4)-A))*B;

%predicting the path of pendcart
tspan = 0:.001:10;
y0 = [2; 0; pi-0.8; 0];
[t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);

% u=-K*y0;
% [T,J]=ode45(@(T,J)(J*y0.'*Q*y0+u.'*R*u),tspan,0);

%Animating pendcart
for k=1:100:length(t)
    drawcartpend(y(k,:),m,M,L);
end
%Analysing output stability for given disturbance
figure(2);
subplot(2,2,1);
plot(t,y(:,1))
ylabel('X posn');
xlabel('time');
title('Stabilaisation of x wrt time');

subplot(2,2,2);
plot(t,y(:,2))
ylabel('Velocity');
xlabel('time');
title('Stabilaisation of v wrt time');

subplot(2,2,3);
plot(t,pi-y(:,3))
ylabel('Theta error');
xlabel('time');
title('Stabilaisation of theta wrt time');

subplot(2,2,4);
plot(t,y(:,4))
ylabel('Angular velocity');
xlabel('time');
title('Stabilaisation of w wrt time');

% figure(3);
% step(t_f,t)
% 
% figure(4);
% plot(T,J,'g-')
% 
% figure(5);
% margin(t_f)

%Analysing output stability for impulse disturbance
figure(3);
C=[1 0 0 0;
   0 0 1 0];
D=0;
sys=ss(A,B,C,D);
subplot(311);
impulse(sys)
n=length(K);
A1=A - B * K;
for i=1:n
    B1(:,i)=B * K(i);
end
C1=C;
D1=D;
sys=ss(A1,B1,C1,D1);
subplot(312);
impulse(sys(:,1))
subplot(313);
impulse(sys(:,3))
end
end