function animate_swingup(t,xa,par)

%% Augment state vector and hip position
x0=[-0.3215,    0.3215,    0,   0, ...
          0,         0,    0,   0];

xhip0 = [-par.a1*sin(x0(1)), par.a1*cos(x0(1))];

x = repmat(x0, numel(t), 1);
% substitute state for 4th and 8th 
x(:, 4) = xa(:, 1);
x(:, 8) = xa(:, 2);

xhip = repmat(xhip0, numel(t), 1);

%% Settings
colors = [1 0 0
          0 0 1
          0 0 0
          0 1 0];

%% Create link shapes
shape3 = linkshape( par.a3);
shape4 = linkshape(-par.a4);

%% Objects
animationfig = figure;
AxesHandle = axes('Parent',animationfig);
link3 = patch('Parent',AxesHandle, 'FaceColor',colors(3,:));
link4 = patch('Parent',AxesHandle, 'FaceColor',colors(4,:));

%% Animation
for n=1:length(t) 
    
    % state vector
    p3 = x(n,3);
    p4 = x(n,4);
    xh = xhip(n,1);
    yh = xhip(n,2);
    % leg positions
    pos3 = move(R(p3)*shape3,[xh;yh]);
    pos4 = move(R(p4)*shape4,[xh;yh]+R(p3)*[0;par.a3]);
    % potentially add rotation with slope gamma   
    set(link3,'Xdata',pos3(1,:),'Ydata',pos3(2,:));
    set(link4,'Xdata',pos4(1,:),'Ydata',pos4(2,:));
    
    axis equal
    axis([-0.7 1.3 1 3])
    axis off
    pause(0.05)
    drawnow
end

pause(1)
close(animationfig)


function shape = linkshape(l)
link_width = 0.1;
n   = linspace(pi/2,-pi/2,20);
top_arc    = (link_width/2)*[sin(n);cos(n)];
bottom_arc = (link_width/2)*[-sin(n);-cos(n)];
if l<0
    bottom_arc(2,:) = bottom_arc(2,:)+l;
else
    top_arc(2,:) = top_arc(2,:)+l;
end
shape = [top_arc, bottom_arc];


function rot = R(phi)
rot = [cos(phi)  -sin(phi);
       sin(phi)   cos(phi)];
   
function c = move(a, b)
c(1,:) = a(1,:) + b(1);
c(2,:) = a(2,:) + b(2);

        
