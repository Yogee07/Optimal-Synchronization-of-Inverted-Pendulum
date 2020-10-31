function drawcartpend(y,m,M,L)
x = y(1);
th = y(3);
% dimensions
W = 1*sqrt(M/5);  
H = .5*sqrt(M/5); 
wr = .2;          
mr = .3*sqrt(m);  

% positions
y = wr/2+H/2.2; 
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px = x + L*sin(th);
py = y - L*cos(th);

plot([-10 10],[0 0],'k','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.2,'FaceColor',[0.4 0.5 0.9])
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])

plot([x px],[y py],'k','LineWidth',1.5)

rectangle('Position',[(px-mr/2),(py-mr/2),mr,mr],'Curvature',1,'FaceColor',[.1 1 0.4])

xlim([-5 5]);
ylim([-2 2.5]);
drawnow
hold off