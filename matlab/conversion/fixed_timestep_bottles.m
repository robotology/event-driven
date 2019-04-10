%i need to load a file of vBottles (data.log) and the corresponding timing
%(info.log). Then I need to sum the amount of time within a bottle and when
%I reach 2ms I add it to the bottle, increasing the time by the changed
%amout
RATE = 0.004;
MAXTS = 2^24;
TSSCALER = 0.000000080;
directory = 'C:\Users\AGlover\Documents\VVV-eventdriven\3\ATIS\';

%get the start times
display('STARTING FIX...');
info    = fopen([directory 'info.log'], 'r');
if(info < 1)
    display('Could not open info file');
    return;
end

l = fgetl(info);
l = fgetl(info);
ts_start = sscanf(l, '[%f]');
l = fgetl(info);
ts_end = sscanf(l, '[%f]');
fclose(info);

%get the data
vbottle = fopen([directory 'data.log'], 'r');
if(vbottle < 1)
    display('Could not open input bottles');
    return;
end
vbottle_out = fopen([directory 'data.' num2str(RATE) '.log'], 'w');
if(vbottle_out < 1)
    display('Could not open output bottles');
    return;
end

ts_cur = ts_start;
bottle_number = 0;
out_events = [];
DT = 0;
nbottlesin = 0;

l = fgetl(vbottle);
%display(l)

l2 = l(find(l == '(')+1:find(l == ')')-1);
data = sscanf(l2, '%i');
t1 = data(1);

while(l > 0)
%for i = 1:10
    nbottlesin = nbottlesin +1;
    %calculated bottle dt
    l2 = l(find(l == '(')+1:find(l == ')')-1);
    data = sscanf(l2, '%i');
    dt = data(end-1) - t1;
    if(dt < 0)
        dt = dt + MAXTS;
    end
    dt = dt * TSSCALER;
    
    %add to multiple bottle dt's
    t1 = data(end-1);
    DT = DT + dt;
    out_events = [out_events; data];
    
    if(DT > RATE)
        %save bottle
        %make header
        %bottle number, bottle time, AE ( out_events )
        %header = [int2str(bottle_number) ' ' num2str(ts_cur) ' AE (']
        fprintf(vbottle_out, '%i %.6f AE (', bottle_number, ts_cur);
        fprintf(vbottle_out, '%i ', out_events);
        fseek(vbottle_out, -1, 'cof');
        fprintf(vbottle_out, ')\n');
        
        bottle_number = bottle_number + 1;       
        ts_cur = ts_cur + DT;
        DT = 0;
        out_events = [];
    end      
    
    l = fgetl(vbottle);
    
end

fclose(vbottle_out);
fclose(vbottle);

display(['Compressed ' int2str(nbottlesin) ' bottles into ' int2str(bottle_number) ' bottles']);
display(['Dataset length = ' num2str(ts_cur - ts_start) ' seconds, with the port' ...
    ' open for ' num2str(ts_end - ts_start) ' seconds']);



