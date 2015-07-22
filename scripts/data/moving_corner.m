function [ ] = moving_corner( f )
% Displays a moving corner on your screen. 
% Author: Valentina Vasco

figure(1); clf; %hold on;
axes('position', [0.0 0.0 1.0 1.0]);

% lines' length
l = 0.2;

% velocity
spp = 1 / f;

% starting points
p1= [1 0];
p2_1 = [1 l];
p2_2 = [(1-l) 0];

while(p1(2) < 1 & p1(1) > 0)
    rectangle('position', [0 0 1 1], 'facecolor', 'k', 'edgecolor', 'k');
    plot([p1(1), p2_1(1)], [p1(2), p2_1(2)], 'Color', 'w');
    hold on;
    plot([p1(1), p2_2(1)], [p1(2), p2_2(2)], 'Color', 'w');
    set(gca, 'xlim', [0 1]);
    set(gca, 'ylim', [0 1]);
    drawnow;

    % update points' positions
    p1(1) = p1(1) - 1/f;
    p1(2) = p1(2) + 1/f;
    
    p2_1(1) = p2_1(1) - 1/f;
    p2_1(2) = p2_1(2) + 1/f;
    
    p2_2(1) = p2_2(1) - 1/f;
    p2_2(2) = p2_2(2) + 1/f;
    
end
