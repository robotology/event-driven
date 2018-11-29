%% set these options!!
winsize = 0.05; %seconds
rate = 0.1; %seconds
channel = 0;
start_offset = 0.0;
randomised = 0;
counter_clock = 0.00000008;
sensor_height = 240;
sensor_width = 304;

%%
if(~exist('GTdataset', 'var'))
    disp('Please specify the path to the dataset in parameter "GTdataset"');
    return;
end

GTresultfile = [GTdataset(1:find(GTdataset == '.', 1, 'last')) 'GT'];



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

disp(['Approximately ' int2str((GTevents(end, 2) - start_offset) / rate + 0.5) ...
    ' frames to GT']);

figure(2); clf; hold on;

try
    plot(GTevents([1 end], 2), 1, 'o');
    title('GT Distribution');
    xlabel('Time (s)');
    legend('GT Point', 'location', 'outsideeast');
    disp(['Previously ' int2str(size(prevGT, 1)) ' GT points']);
    prevGT = dlmread(GTresultfile);
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

while(~finishedGT)
    
    wini = ci - 1000;
    %wini  = find(GTevents(:, 2) > cts-winsize, 1);
    %if(ci - wini) < 2000; wini = ci - 2000; end
    if(wini < 1); wini = 1; end
    finishedLOG = false;
    runHough = true;
    
    while(~finishedLOG)
        
        window = GTevents(wini:ci, :);
        
        if(runHough)
            runHough = false;
            [x, y, r] = event_hough(window, r-5, r+5, 240, 304);
        end
        
        figure(1); clf; hold on;
        plot(window(window(:, 3) == 0, 4), window(window(:, 3) == 0, 5), 'g.');
        plot(window(window(:, 3) ==  1, 4), window(window(:, 3) ==  1, 5), 'm.');
        
        rectangle('curvature', [1 1], 'position', [x-r y-r r*2 r*2]);
        axis([0 sensor_width 0 sensor_height]);
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
        
        if c == 13 %enter
            runHough = true;
            finishedLOG = true;
            %result(res_i, :) = [cts, x, y, r];
            dlmwrite(GTresultfile, [cts, x, y, r, cputs], '-append', 'delimiter', ' ', 'precision', '%0.6f'); 
            figure(2); hold on; plot(cts, 1, 'gx');
        elseif c == 32 %space
            runHough = true;
            finishedLOG = true;
        elseif c == 27 %ESC
            finishedLOG = true;
            finishedGT = true;
        elseif c == '+' | c == '='
            r = r + 1;
        elseif c == '-' | c == '_'
            r = r - 1;
            if r == 0; r = 1; end;
        elseif c == 30 %up
            y = y + 1;
            if y > sensor_height; y = sensor_height; end;
        elseif c == 31 %down
            y = y - 1;
            if y == 0; y = 1; end;
        elseif c == 29 %right
            x = x + 1;
            if x > sensor_width; x = sensor_width; end;
        elseif c == 28 %left
            x = x - 1;
            if x == 0; x = 1; end;
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
try close(1); end;
%try close(2); end;

