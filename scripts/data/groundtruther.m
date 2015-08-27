
if(~exist('GTdataset', 'var'))
    display('Please specify the path to the dataset in parameter "GTdataset"');
    return;
end

GTresultfile = [GTdataset(1:find(GTdataset == '.', 1, 'last')) 'GT'];

% if(~exist('GTresultfile', 'var'))
%     display('Please specify the path to the dataset in parameter "GTresultfile"');
%     return;
% end
%GTdataset = '~/DataBackup/DATASETS/redballtrackdataset/withperson/ball_ExtDVS.txt';
%GTresultfile = '~/DataBackup/DATASETS/redballtrackdataset/withperson/GT.txt';
winsize = 1.0; %seconds
rate = 1.0; %seconds
channel = 0;
if(~exist('GTevents', 'var'))
    
    display('Loading data...');
    %CH TS POL X Y
    GTevents = importdata(GTdataset); % import data
    
    GTevents(GTevents(:, 1) ~= channel, :) = [];
    GTevents(:, 2) = GTevents(:, 2) / 1000000; % change time scale to seconds
    
else 
    display('Using pre-loaded data');
end

result = zeros(ceil((GTevents(end, 2) - GTevents(1, 2)) / rate), 4);

display('Press Esc to quit');
display('Press Enter to log position');
display('Press Space to skip position');
display('Press +/- to change circle size');
display('Move circle with arrows');

display(['Approximately ' int2str((GTevents(end, 2) - GTevents(1, 2)) / rate + 0.5) ...
    ' frames to GT']);

figure(1); clf;

cts = GTevents(1, 2) + winsize;
ci = find(GTevents(1:end, 2) > cts, 1) - 1;
cts = GTevents(ci, 2);

res_i = 1;
x = 64; y = 64; r = 20; 

while cts < GTevents(end, 2)
    
    wini  = find(GTevents(1:end, 2) > cts-winsize, 1);
    finished = false;
    
    while(~finished)
        clf; hold on;
        window = GTevents(wini:ci, :);       
        plot(window(window(:, 3) == 0, 4), window(window(:, 3) == 0, 5), 'g.');
        plot(window(window(:, 3) ==  1, 4), window(window(:, 3) ==  1, 5), 'm.');
        rectangle('curvature', [1 1], 'position', [x-r/2 y-r/2 r r]);
        axis([0 128 0 128]);
        drawnow;
        try
            c = waitforbuttonpress;
            if c
                c = get(1, 'CurrentCharacter');
                %uint32(get(1, 'currentcharacter'))
            end
        catch
            return;
        end
        
        if c == 13 %enter
            finished = true;
            result(res_i, :) = [cts, x, y, r];
        elseif c == 32 %space
            finished = true;
        elseif c == 27 %ESC
            close(1);
            return;
        elseif c == '+' || c == '='
            r = r + 1;
        elseif c == '-' || c == '_'
            r = r - 1;
            if r == 0; r = 1; end;
        elseif c == 30 %up
            y = y + 1;
            if y == 129; y = 128; end;
        elseif c == 31 %down
            y = y - 1;
            if y == 0; y = 1; end;
        elseif c == 29 %right
            x = x + 1;
            if x == 129; x = 128; end;
        elseif c == 28 %left
            x = x - 1;
            if x == 0; x = 1; end;
        end
        
    end
   
    res_i = res_i + 1;
    cts = cts + rate;
    ci = find(GTevents(1:end, 2) > cts, 1) - 1;
    cts = GTevents(ci, 2);
    dlmwrite(GTresultfile, result, 'delimiter', ' ', 'precision', '%0.6f'); 
    
end

display('Finished Ground-truthing');
close(1);
    
   
