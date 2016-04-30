% Displays the surface associated to the events

filename = '/home/aglover/workspace/datasets/iros_circle/fasthand/dvs/data.log.txt';
events = importdata(filename);

% select polarity and channel 
sel_pol = 1;
sel_channel = 0;

% leave all events with the selected channel and polarity 
%events_left_on = events(events(:, 1) == sel_channel & events(:, 3) == sel_pol, :);
events_left_on = events(events(:, 1) == sel_channel, :);
events_left_on(:, 2) = events_left_on(:, 2) - events_left_on(1, 2);

figure(1); 
clf;
set(gcf, 'CurrentCharacter', 'a')
paused = false;
recordvideo = false;

if(recordvideo)
    vidwriter = VideoWriter('uncompressed.avi', 'uncompressed avi');
    open(vidwriter);
end

for i=1:size(events_left_on, 1)
    
    
    % associate to each incoming event its timestamp in vSurf
    
    vSurf(events_left_on(i, 4) + 1, events_left_on(i, 5) + 1) = events_left_on(i, 2);
    
    if mod(i, 100) == 0
        if ~ishandle(1); break; end;
        cp = campos;
        mesh(vSurf);
        campos(cp);
        xlabel('x');
        ylabel('y');
        zlabel('ts (s)');
        zlim1 = max(events_left_on(i, 2)-2e5, 0);
        set(gca, 'zlim', [zlim1, zlim1+2e5]);
        grid on;
        campos([-694.21       426.36    1.376e+08]);
        drawnow;
        if(recordvideo)
            writeVideo(vidwriter, getframe(gcf));
        end
        c = get(gcf , 'CurrentCharacter');
        if(c == 32) %space
            %pause
            paused = true;
        elseif(c == 13) %enter
            %continue
            paused = false;
        elseif(c == 27) %ESC
            %close(1);
            break;
        end
        if(paused)
            waitforbuttonpress;
        end
            
    end
    
end

    