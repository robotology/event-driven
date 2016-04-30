if(~exist('infilename', 'var'))
    display('Please specify the path to the dataset in parameter "infilename"');
    return;
end
outfilename = [infilename '.txt'];

display(['Reading from file ' infilename]);
display(['Writing to file ' outfilename]);

CH = 1;
TS = 2;
POL = 3;
X = 5;
Y = 4;

POLSH = 0;
CHSH  = 15;
XSH   = 1;
YSH   = 8;

TSBITS = int32(2^31-1); %all but most significant bit
POLBIT = int32(2^POLSH); %0th bit
CHBIT  = int32(2^CHSH); %15th bit
XBITS  = bitshift(int32(2^7-1), XSH);
YBITS  = bitshift(int32(2^7-1), YSH);

MAXSTAMP = 2^24-1;

infile = fopen(infilename, 'r');
if(infile < 0)
    display('Could not open file');
    return;
end
fseek(infile, 0, 'eof');
totch = ftell(infile);
frewind(infile);

delete(outfilename);

dth = 0.1;
bi = 0;
vi = 0;
wraps = 0;
pts = 0;

l = fgetl(infile);
while(ischar(l))
    bi = bi + 1;
    
    l = l(find(l == '(')+1:find(l == ')')-1);
    bottle = sscanf(l, '%i %i');
    temp = length(bottle) / 2;
    vi = vi + length(bottle) / 2;
    bottle = reshape(bottle, 2, length(bottle)/2)';
    errors = bottle(:, 1) > 0 | bottle(:, 2) < 0 | bottle(:, 2) > 65535;
    if(sum(errors))
        display(['Bottle Data Corrupt (' int2str(sum(errors)) ') in ' ...
            'bottle index: ' int2str(bi)]);
        bottle = bottle(~errors, :);
    end
    
    textformat = zeros(size(bottle, 1), 6);
    textformat(:, TS) = bitand(int32(bottle(:, 1)), TSBITS) + wraps * MAXSTAMP;
    textformat(:, CH) = bitshift(bitand(int32(bottle(:, 2)), CHBIT), -CHSH);
    textformat(:, POL) = bitshift(bitand(int32(bottle(:, 2)), POLBIT), -POLSH);
    textformat(:, X) = bitshift(bitand(int32(bottle(:, 2)), XBITS), -XSH);
    textformat(:, Y) = bitshift(bitand(int32(bottle(:, 2)), YBITS), -YSH);
    textformat(1, 6) = 1;
    
    %check for wraps
    %between two bottles
    if textformat(1, TS) < pts
       %there was a wrap here
       wraps = wraps + 1;
       textformat(:, TS) = textformat(:, TS) + MAXSTAMP;
       
    end
    pts = textformat(end, TS);
    
    %within bottle
    wrapcheck = diff(textformat(:, TS)) < 0;
    if sum(wrapcheck)
       %there was a wrap here
       wraps = wraps + 1;
       wi = find(wrapcheck);
       textformat(wi+1:end, TS) = textformat(wi+1:end, TS) + MAXSTAMP;
    end
    

    if(size(textformat, 1) ~= temp)
        display('error');
    end
    dlmwrite(outfilename, textformat, '-append', 'delimiter', ' ', 'precision', '%i');
    
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
    
