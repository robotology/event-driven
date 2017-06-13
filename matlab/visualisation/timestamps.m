if(~exist('infilename', 'var'))
    display('Please specify the path to the dataset in parameter "infilename"');
    return;
end

display(['Reading from file ' infilename]);

TSBITS = int32(2^31-1); %all but most significant bit

MAXSTAMP = 2^24-1;

infile = fopen(infilename, 'r');
if(infile < 0)
    display('Could not open file');
    return;
end
fseek(infile, 0, 'eof');
totch = ftell(infile);
frewind(infile);

dth = 0.1;
bi = 0;
vi = 0;
wraps = 0;
pts = 0;

stamplist = [];

l = fgetl(infile);
while(ischar(l))
    bi = bi + 1;
    
    temp = sscanf(l, '%d %d', 2);
    bnum = temp(1);
    yts = temp(2);
    
    l = l(find(l == '(')+1:find(l == ')')-1);
    bottle = sscanf(l, '%i %i');
    temp = length(bottle) / 2;
    vi = vi + length(bottle) / 2;
    bottle = reshape(bottle, 2, length(bottle)/2)';
    
    stamplist = [stamplist; [ones(temp, 1)*bnum ones(temp, 1)*yts bitand(int32(bottle(:, 1)), TSBITS)]];
    
      
        
    if(ftell(infile) / totch > dth)
        display([int2str(100 * ftell(infile) / totch) '% done']);
        dth = dth + 0.1;
    end
    
    l = fgetl(infile);

end

fclose(infile);

display(['100% done (' int2str(bi) ' bottles and ' int2str(vi) ' events)']);

% vBotSize = 1000;
% ch = 1;
% ts = 2;
% pol = 3;
% x = 5;
% y = 4;
% 
% %channel, stamp, polarity, x, y
% data = dlmread(file);
% data(data(:, pol) == -1, pol) = 0;
% 
% %timestamp 4 bytes
% %(26) 1 byte and (stamp) in 3 bytes
% w1 = bitor(bitshift(32, 26, 'int32'), int32(data(:, ts)));
% 
% w2 = int32(zeros(size(data, 1), 1));
% w2 = bitor(w2, int32(bitshift(data(:, ch), 15, 'int32')), 'int32');
% w2 = bitor(w2, int32(bitshift(data(:, y),  8, 'int32')), 'int32');
% w2 = bitor(w2, int32(bitshift(data(:, x), 1, 'int32')), 'int32');
% w2 = bitor(w2, int32(bitshift(data(:, pol), 0, 'int32')), 'int32');

%datafile = fopen('data.log', 'w+');
% for i = data(1, ts) : vBotSize : data(end, ts)
%    
%     indicies = data(:, ts) >= i & data(:, ts) < (i + vBotSize);
%     
%     if(sum(indicies) == 0) continue; end;
%     
%     %write the vBottle of events
%     fprintf(datafile, '-1 %0.6f AE (', i / 1000000);
%     for j = find(indicies)'
%         fprintf(datafile, '%i %i ', w1(j), w2(j));
%     end
%     fseek(datafile, -1, 'cof'); 
%     fprintf(datafile, ')\n');
% 
% end
% 
% fseek(datafile, -1, 'cof');
% fprintf(datafile, '');
% fclose(datafile);
% 
% logfile  = fopen('info.log', 'w+');
% fprintf(logfile, 'Type: Bottle;\n');
% fprintf(logfile, '[%0.6f] /aexGrabber/vBottle:o [connected]\n', data(1, ts) / 1000000);
% fprintf(logfile, '[%0.6f] /aexGrabber/vBottle:o [disconnected]', data(end, ts) / 1000000);
% 
% 
% fclose(logfile);
    
