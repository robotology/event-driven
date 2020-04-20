if(~exist('winsize', 'var'))
    disp('Please specify the window size');
    return;
end

if(~exist('rate', 'var'))
    disp('Please specify the frame rate');
    return;
end

if(~exist('channel', 'var'))
    disp('Please specify the channel');
    return;
end

if(~exist('randomised', 'var'))
    disp('Please specify the parameter "randomised"');
    return;
end

if(~exist('counter_clock', 'var'))
    disp('Please specify the clock period');
    return;
end

if(~exist('sensor_height', 'var'))
    disp('Please specify the height of the sensor');
    return;
end

if(~exist('sensor_width', 'var'))
    disp('Please specify the width of the sensor');
    return;
end

if(~exist('GTdataset', 'var'))
    disp('Please specify the path to the dataset in parameter "GTdataset"');
    return;
end

if(~exist('GTresultfile', 'var'))
    GTresultfile = [GTdataset(1:find(GTdataset == '.', 1, 'last')) 'GT'];
    disp(strcat('Please specify the resulting file "GTdataset", setting to: ', GTresultfile));
end

if(~exist('runHough', 'var'))
    runHough = false;
    h_score = 0;
    disp(strcat('Please specify the "runHough" parameter, setting to: ', runHough));
end

disp('Loading data...');
%CH TS POL X Y
GTevents = importdata(GTdataset); % import data

GTevents(GTevents(:, 1) ~= channel, :) = [];
GTevents(:, 2) = GTevents(:, 2) * counter_clock; % change time scale to seconds
start_offset = (GTevents(end, 2) - GTevents(1, 2))*start_offset + GTevents(1, 2);

disp('Press Esc to quit');
disp('Press Enter to log position');
disp('Press Space to skip position');
disp('Press +/- to change circle size');
disp('Move circle with arrows');
disp('alternatively, use "a,d,w,s" to move, "k,j" to change radius, "h" to log');
disp(['Approximately ' int2str((GTevents(end, 2) - start_offset) / rate + 0.5) ...
    ' frames to GT']);

figure(2); clf; hold on;

try
    plot(GTevents([1 end], 2), 1, 'o');
    title('GT Distribution');
    xlabel('Time (s)');
    %legend('GT Point', 'location', 'northeastoutside');
    prevGT = dlmread(GTresultfile);
    disp(['Previously ' int2str(size(prevGT, 1)) ' GT points']);
    plot(prevGT(:, 1), ones(length(prevGT), 1), 'bx');
end


figure(1); clf;

if(randomised)
    cts = GTevents(round(rand(1) * length(GTevents)), 2);
    ci = find(GTevents(1:end, 2) > cts, 1) - 1;
else
    cts = start_offset + winsize;
    ci = find(GTevents(1:end, 2) > cts, 1) - 1;
    cts = GTevents(ci, 2);
end

cputs = GTevents(ci, 7);

%res_i = 1;
x = 152; y = 120; r = 30;

finishedGT = false;
initialised = false;

while(~finishedGT)
    
    wini = ci - 2000;
    %wini  = find(GTevents(:, 2) > cts-winsize, 1);
    %if(ci - wini) < 2000; wini = ci - 2000; end
    if(wini < 1); wini = 1; end
    finishedLOG = false;
    
    while(~finishedLOG)
        
        window = GTevents(wini:ci, :);
        
        figure(1); clf; hold on;
        plot(window(window(:, 3) == 0, 4), window(window(:, 3) == 0, 5), 'g.');
        plot(window(window(:, 3) ==  1, 4), window(window(:, 3) ==  1, 5), 'm.');
        
        rectangle('curvature', [1 1], 'position', [x-r y-r r*2 r*2]);
        axis([0 sensor_width 0 sensor_height]);
        drawnow;
        
        if(h_score > 350)
            c = 13;
        else
            rectangle('curvature', [1 1], 'position', [x-r y-r r*2 r*2], 'edgecolor', 'r');
            drawnow;
            try
                c = waitforbuttonpress;
                if c
                    c = get(1, 'CurrentCharacter');
                    %uint32(get(1, 'currentcharacter'))
                else
                    mousep = get(gca, 'currentpoint');
                    y = round(mousep(2, 2));
                    x = round(mousep(2, 1));
                    c = -1;
                end
            catch
                c = -1;
                finishedLOG = true;
                finishedGT = true;
            end
        end
        
        if c == 13  || c == 'h'%enter
            if ~initialised
                xold = x;
                yold = y;
                rold = r;
                initialised = true;
            end
            
            finishedLOG = true;
            %result(res_i, :) = [cts, x, y, r];
            dlmwrite(GTresultfile, [cts, x, y, r, cputs], '-append', 'delimiter', ' ', 'precision', '%0.6f'); 
            figure(2); hold on; plot(cts, 1, 'gx');
            
            if(runHough)
                [x, y, r, h_score] = event_hough(window, r-5, r+5, 240, 304);
            else
                dx = x - xold;
                dy = y - yold;
                dr = r - rold;
                xold = x;
                yold = y;
                rold = r;
                x = x + dx;
                y = y + dy;
            end
            
        elseif c == 32 %space
            finishedLOG = true;
            
            if(runHough)
                [x, y, r, h_score] = event_hough(window, r-5, r+5, 240, 304);
            else
                if initialised
                    dx = x - xold;
                    dy = y - yold;
                    dr = r - rold;
                    xold = x;
                    yold = y;
                    rold = r;
                    x = x + dx;
                    y = y + dy;
                end
            end
        elseif c == 27 %ESC
            finishedLOG = true;
            finishedGT = true;
        elseif c == '+' || c == '=' || c == 'k'
            r = r + sensitivity;
        elseif c == '-' || c == '_' || c == 'j'
            r = r - sensitivity;
            if r == 0; r = 1; end
        elseif c == 30  || c == 'w'%up
            y = y + sensitivity;
            if y > sensor_height; y = sensor_height; end
        elseif c == 31  || c == 's'%down
            y = y - sensitivity;
            if y == 0; y = 1; end
        elseif c == 29  || c == 'd'%right
            x = x + sensitivity;
            if x > sensor_width; x = sensor_width; end
        elseif c == 28  || c == 'a'%left
            x = x - sensitivity;
            if x == 0; x = 1; end
        end
        
    end
   
    %res_i = res_i + 1;
    cts = cts + rate;
    if(randomised)
        cts = GTevents(round(rand(1) * length(GTevents)), 2);
    end
        
    ci = find(GTevents(1:end, 2) > cts, 1);
    if isempty(ci)
        finishedGT = true;
    else
        cts = GTevents(ci, 2);
        cputs = GTevents(ci, 7);
    end
    

    if cts >= GTevents(end, 2)
        finishedGT = true;
    end

end

try
    allgt = dlmread(GTresultfile);
    [y, ia, ic] = unique(allgt(:, 1));
    %[y, i] = sort(allgt(:, 1), 1, 'ascend');
    dlmwrite(GTresultfile, allgt(ia, :), 'delimiter', ' ', 'precision', '%0.6f');
    disp([int2str(size(allgt, 1)) ' ground truth points']);
    figure(2); clf; hold on;
    plot(allgt(ia, 1), ones(length(ia), 1), 'bx');
    plot(GTevents([1 end], 2), 1, 'o');
    title('GT Distribution');
    xlabel('Time (s)');
    legend('GT Point', 'location', 'outsideeast');
catch
    disp('No Ground Truth Points');
end

disp('Finished Ground-truthing');
try close(1); end
%try close(2); end;

