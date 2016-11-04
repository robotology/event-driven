file = ['/home/aglover/DataBackup/DATASETS/redballtrackdataset/' ...
    'withperson/ball_ExtDVS.txt'];

vBotSize = 1000;
ch = 1;
ts = 2;
pol = 3;
x = 5;
y = 4;

%channel, stamp, polarity, x, y
data = dlmread(file);
data(data(:, pol) == -1, pol) = 0;

%timestamp 4 bytes
%(26) 1 byte and (stamp) in 3 bytes
w1 = bitor(bitshift(32, 26, 'int32'), int32(data(:, ts)));

w2 = int32(zeros(size(data, 1), 1));
w2 = bitor(w2, int32(bitshift(data(:, ch), 15, 'int32')), 'int32');
w2 = bitor(w2, int32(bitshift(data(:, y),  8, 'int32')), 'int32');
w2 = bitor(w2, int32(bitshift(data(:, x), 1, 'int32')), 'int32');
w2 = bitor(w2, int32(bitshift(data(:, pol), 0, 'int32')), 'int32');

datafile = fopen('data.log', 'w+');
for i = data(1, ts) : vBotSize : data(end, ts)
   
    indicies = data(:, ts) >= i & data(:, ts) < (i + vBotSize);
    
    if(sum(indicies) == 0) continue; end;
    
    %write the vBottle of events
    fprintf(datafile, '-1 %0.6f AE (', i / 1000000);
    for j = find(indicies)'
        fprintf(datafile, '%i %i ', w1(j), w2(j));
    end
    fseek(datafile, -1, 'cof'); 
    fprintf(datafile, ')\n');

end

fseek(datafile, -1, 'cof');
fprintf(datafile, '');
fclose(datafile);

logfile  = fopen('info.log', 'w+');
fprintf(logfile, 'Type: Bottle;\n');
fprintf(logfile, '[%0.6f] /aexGrabber/vBottle:o [connected]\n', data(1, ts) / 1000000);
fprintf(logfile, '[%0.6f] /aexGrabber/vBottle:o [disconnected]', data(end, ts) / 1000000);


fclose(logfile);
    
