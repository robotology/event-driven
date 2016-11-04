% Displays a rotating point on the screen
% Author: Valentina Vasco

% center coordinates
x0 = 0;
y0 = 0;

% radius
r = 0.5;

% number of rotations
n_rot = 3;
% final angle
end_theta = 2*n_rot*pi;

% velocity of rotation
v = 0.06; 

rectangle('position', [-1 -1 2 2], 'facecolor', 'k', 'edgecolor', 'k');
hold on;
for theta = 0:v:end_theta
   
    x = x0 + r*cos(theta);
    y = y0 + r*sin(theta);
    
    plot(x, y, 'w.', 'MarkerSize', 30);
    drawnow;
    plot(x, y, 'k.', 'MarkerSize', 30);
    
end