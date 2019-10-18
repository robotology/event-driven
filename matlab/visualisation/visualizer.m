% Visualizes a batch of events, defined by deltat
% Author: Valentina Vasco



filename = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison3\20_256\ATIS\data.log.txt';

sel_channel = 0;
sel_pol = 1;

events = importdata(filename); % import data
%events(events(:, 1) ~= sel_channel, :) = []; % remove all events with channel different from the one selected
%events(events(:, 3) ~= sel_pol, :) = []; % remove all events with polarity different from the one selected
%events(:, 2) = events(:, 2)./1000000; % change time scale to seconds


events(:, 7) = events(:, 7) - events(1, 7);
ts = events(1, 7); % starting timestamp 
deltat = 0.2; % seconds

figure(1); clf;
%hold on; % remove hold on if you want to visualize only data in the current window 
set(gca, 'xlim', [0 304]);
set(gca, 'ylim', [0 240]);
set(gca, 'zlim', [events(1, 7) events(end, 7)]);
xlabel('x');
ylabel('y');
zlabel('ts (s)');
grid on;
    
for i = 1:size(events, 1)
    
    batch = events(events(:, 7) >= ts & (events(:, 7) <= ts + deltat), :); % define a window of events which fall in the temporal window defined by deltat
    
    if ~ishandle(1); break; end;
    scatter3(batch(:, 4), batch(:, 5), batch(:, 7), 1, 'm'); % visualize events in the 3D space with colour depending on timestamp
    drawnow;
    pause
    
    ts = ts + deltat; % update timestamp
    
end

 
