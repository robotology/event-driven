%% set these options!!
winsize = 2; %seconds
rate = 2; %seconds
channel = 0;
start_offset = 0;
randomised = 0;

%%
if(~exist('GTdataset', 'var'))
    display('Please specify the path to the dataset in parameter "GTdataset"');
    return;
end

GTresultfile = [GTdataset(1:find(GTdataset == '.', 1, 'last')) 'GT'];



display('Loading data...');
%CH TS POL X Y
GTevents = importdata(GTdataset); % import data

GTevents(GTevents(:, 1) ~= channel, :) = [];
GTevents(:, 2) = GTevents(:, 2) / 1000000; % change time scale to seconds
start_offset = (GTevents(end, 2) - GTevents(1, 2))*start_offset + GTevents(1, 2);



display('Press Esc to quit');
display('Press Enter to log position');
display('Press Space to skip position');
display('Press +/- to change circle size');
display('Move circle with arrows');

display(['Approximately ' int2str((GTevents(end, 2) - start_offset) / rate + 0.5) ...
    ' frames to GT']);

figure(2); clf; hold on;

try
    prevGT = dlmread(GTresultfile);
    plot(prevGT(:, 1), ones(length(prevGT), 1), 'bx');
    plot(GTevents([1 end], 2), 1, 'o');
    title('GT Distribution');
    xlabel('Time (s)');
    legend('GT Point');
    display(['Previously ' int2str(size(prevGT, 1)) ' GT points']);
end


figure(1); clf;

cts = start_offset + winsize;
ci = find(GTevents(1:end, 2) > cts, 1) - 1;
cts = GTevents(ci, 2);

if(randomised)
    cts = GTevents(round(rand(1) * length(GTevents)), 2);
    ci = find(GTevents(1:end, 2) > cts, 1) - 1;
end
    

%res_i = 1;
x = 64; y = 64; r = 10;
%linecoords = [x x y y];
linecoords = [];

finishedGT = false;

while(~finishedGT)
    
    wini  = find(GTevents(:, 2) > cts-winsize, 1);
    %if(ci - wini) < 2000; wini = ci - 2000; end
    if(wini < 1); wini = 1; end;
    finishedLOG = false;
    
    while(~finishedLOG)
        figure(1); clf; hold on;
        window = GTevents(wini:ci, :);       
        plot(window(window(:, 3) == 0, 4), window(window(:, 3) == 0, 5), 'g.');
        plot(window(window(:, 3) ==  1, 4), window(window(:, 3) ==  1, 5), 'm.');
        %rectangle('curvature', [1 1], 'position', [x-r y-r r*2 r*2]);
        %plot lines
        axis([0 128 0 128]);
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
            finishedLOG = true;
            %result(res_i, :) = [cts, x, y, r];
            %calculate how many pixels are in the lines
            %save that with the cts
            dlmwrite(GTresultfile, [cts, x, y, r], '-append', 'delimiter', ' ', 'precision', '%0.6f'); 
            figure(2); hold on; plot(cts, 1, 'gx');
        elseif c == 32 %space
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
   
    %res_i = res_i + 1;
    cts = cts + rate;
    if(randomised)
        cts = GTevents(round(rand(1) * length(GTevents)), 2);
    end
    
    try
        ci = find(GTevents(1:end, 2) > cts, 1) - 1;
        cts = GTevents(ci, 2);
    catch
        finishedGT = true;
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
    display([int2str(size(allgt, 1)) ' ground truth points']);
    figure(2); clf; hold on;
    plot(allgt(ia, 1), ones(length(ia), 1), 'bx');
    plot(GTevents([1 end], 2), 1, 'o');
    title('GT Distribution');
    xlabel('Time (s)');
    legend('GT Point');
catch
    display('No Ground Truth Points');
end

display('Finished Ground-truthing');
try close(1); end;
try close(2); end;

