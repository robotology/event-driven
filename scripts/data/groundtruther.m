
dataset = '~/DataBackup/DATASETS/redballtrackdataset/withperson/ball_ExtDVS.txt';
resultfile = '~/DataBackup/DATASETS/redballtrackdataset/withperson/GT.txt';
winsize = 0.05; %seconds
rate = 0.2; %seconds
channel = 0;

%CH TS POL X Y
events = importdata(dataset); % import data

events(events(:, 1) ~= channel, :) = [];
events(:, 2) = events(:, 2) / 1000000; % change time scale to seconds

result = zeros(ceil((events(end, 2) - events(1, 2)) / rate), 4);

figure(1); clf;

cts = events(1, 2) + winsize;
ci = find(events(1:end, 2) > cts, 1) - 1;
cts = events(ci, 2);

res_i = 1;
x = 64; y = 64; r = 20; 

while cts < events(end, 2)
    
    wini  = find(events(1:end, 2) > cts-winsize, 1);
    finished = false;
    
    while(~finished)
        clf; hold on;
        window = events(wini:ci, :);       
        plot(window(window(:, 3) == -1, 4), window(window(:, 3) == -1, 5), 'g.');
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
    ci = find(events(1:end, 2) > cts, 1) - 1;
    cts = events(ci, 2);
    dlmwrite(resultfile, result, 'delimiter', ' ', 'precision', '%0.6f'); 
    
end
    
    
    
