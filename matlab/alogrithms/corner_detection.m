% Detects corners using Harris' corner detection approach. 
% For each incoming event, it approximates the derivative, as the difference 
% between the number of events in the actual pixel (x0, y0) and in the 
% neighboring pixels (x0+1, y0) (x0, y0+1), in a temporal window. 
% The event is labeled as corner event if the minimum eigenvalue is larger 
% than a threshold.
% Author: Valentina Vasco

clear;
filename = 'corner3DVS.txt';

% amplitude of temporal window
deltat =  0.1; % seconds

% threshold for the minimum eigenvalue
thresh0 = 1.5; 
px_thr = 20; 

sel_channel = 0;
sel_pol = 1;

% import events
events = importdata(filename);

% select events with selected channel and polarity
events(events(:, 1) ~= sel_channel, :) = [];
events(events(:, 3) ~= sel_pol, :) = [];

% convert to seconds
events(:, 2) = events(:, 2)./1000000;

q = 1;
i_start = 30000;
ts0 = events(i_start, 2);

while(ts0 < events(end, 2))
    
    % events falling in the temporal window defined by deltat
    data = events(events(:, 2) >= ts0 & (events(:, 2) <= ts0 + deltat), 2:end);
    data(:, 2) = [];
    
    for i = 1:size(data, 1)
        x0 = data(i, 2);
        y0 = data(i, 3);
        
        % (x0, y0 - px_thr <= y <= y0 + px_thr)
        data_xy = data((x0 == data(:, 2)), :);
        data_xy = data_xy(abs(data_xy(:, 3) - y0) <= px_thr, :);
        n_xy = size(data_xy, 1);
        
        % (x0 + 1, y0 - px_thr <= y <= y0 + px_thr)
        data_dx = data((x0 - data(:, 2)) == -1, :);
        data_dx = data_dx(abs(data_dx(:, 3) - y0) <= px_thr, :);
        n_dx = size(data_dx, 1);
        
        dx(i) = n_dx - n_xy;
        
        % (x0 - px_thr <= x <= x0 + px_thr, y0 + 1)
        data_up = data((y0 - data(:, 3)) == -1, :);
        data_up = data_up(abs(data_up(:, 2) - x0) <= px_thr, :);
        n_up = size(data_up, 1);
        
        % (x0 - px_thr <= x <= x0 + px_thr, y0)
        data_yx = data((y0 - data(:, 3)) == 0, :);
        data_yx = data_yx(abs(data_yx(:, 2) - x0) <= px_thr, :);
        n_yx = size(data_yx, 1);
        
        dy(i) = n_up - n_yx;
        
        de_dxdy(i) = dx(i)*dy(i);
        D = [dx(i) de_dxdy(i); de_dxdy(i) dy(i)];
        
        % compute the matrix singular values and take the minimum
        S = svd(D);
        l(i) = min(S);
        
        % reset
        data_xy = [];
        data_yx = [];
        data_dx = [];
        data_up = [];
        
    end
    
    % normalize the minimum eigenvalue by the number of events in the
    % temporal window
    l = l./size(data, 1);
    
    % label as corner events the events where the minimum eigenvalue aboves
    % the threshold
    corn_x = data(l >= thresh0, 2);
    corn_y = data(l >= thresh0, 3);
    corn_ts = data(l >= thresh0, 1);
    
    if(length(corn_x) ~= 0)
        % if some corners were detected, assign them to the corner
        % variables
        corner_x(q:(q + length(corn_x) - 1)) = corn_x;
        corner_y(q:(q + length(corn_x) - 1)) = corn_y;
        corner_ts(q:(q + length(corn_x) - 1)) = corn_ts;
    else
        % if no corner was detected, assign -1 to the corner variables
        corner_x = -1;
        corner_y = -1;
        corner_ts = -1;
    end
    
    q = q + length(corn_x);
    
    drawnow;
    pause(0.8);
    subplot(2, 1, 1);
    plot(data(:, 2), data(:, 3), 'b.');
    hold on;
    plot(corner_x, corner_y, 'ko');
    grid on;
    legend('events in the window', 'corners');
    xlabel('x');
    ylabel('y');
    set(gca, 'xlim', [0 128]);
    set(gca, 'ylim', [0 128]);
    title('Detected corners');
    hold off;
    subplot(2, 1, 2);
    plot(l);
    hold on;
    plot(thresh0*ones(length(l), 1));
    legend('min eigenvalue', 'threshold');
    title('Eigenvalue distribution');
    xlabel('# events');
    ylabel('minimum eigenvalue');
    hold off;
    
    % take the last timestamp of the temporal window
    ts0 = data(end, 1);
    
    % reset
    data = [];
    
end