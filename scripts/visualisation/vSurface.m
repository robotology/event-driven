% Displays the surface associated to the events

filename = '~/DataBackup/DATASETS/corner_extDVS/corner1.txt';
events = importdata(filename);

% select polarity and channel 
sel_pol = 1;
sel_channel = 0;

% leave all events with the selected channel and polarity 
events_left_on = events(events(:, 1) == sel_channel & events(:, 3) == sel_pol, :);
events_left_on(:, 2) = events_left_on(:, 2) - events_left_on(1, 2);

figure(1); 
clf;

vSurf = zeros(128, 128);

for i=1:size(events_left_on, 1)
    
    if ~ishandle(1); break; end;
    % associate to each incoming event its timestamp in vSurf 
    vSurf(events_left_on(i, 4) + 1, events_left_on(i, 5) + 1) = events_left_on(i, 2);
    if mod(i, 100) == 0
        mesh(vSurf);
        xlabel('x');
        ylabel('y');
        zlabel('ts (s)');
        grid on;
        drawnow;
    end
    
end

    