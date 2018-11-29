if(~exist('infilename', 'var'))
    disp('Please specify the path to the dataset in parameter "infilename"');
    return;
end
outfilename = [infilename '.txt'];

if(~exist('CODEC_TYPE', 'var'))
    disp('Please specify CODEC_TYPE:');
    disp('0 - DVS_128x128');
    disp('1 - ATIS_20BIT');
    disp('2 - ATIS_24BIT');
    return;
end

if(~exist('TIMESTAMP_BITS', 'var'))
   disp('Please set TIMESTAMP_BITS (24 -> 31)');
   return;
end

if(~exist('CLOCK_PERIOD', 'var'))
   disp('Please set CLOCK_PERIOD (e.g. 0.000000080)');
   return;
end

disp(['Reading from file ' infilename]);
disp(['Writing to file ' outfilename]);

MAXSTAMP = 2^TIMESTAMP_BITS-1;

if(CODEC_TYPE == 0)
    
    CH = 1; TS = 2; POL = 3; X = 5;  Y = 4; BOTTLE_START = 6; YARP_TS = 7;
    
    POLSH = 0;
    CHSH  = 15;
    XSH   = 1;
    YSH   = 8;
    
    TSBITS = int32(2^TIMESTAMP_BITS-1); %all but most significant bit
    POLBIT = int32(2^POLSH); %0th bit
    CHBIT  = int32(2^CHSH); %15th bit
    XBITS  = bitshift(int32(2^7-1), XSH);
    YBITS  = bitshift(int32(2^7-1), YSH);

elseif(CODEC_TYPE == 1)
    
    CH = 1; TS = 2; POL = 3; X = 4; Y = 5; BOTTLE_START = 6; YARP_TS = 7;
    
    POLSH = 0;
    CHSH  = 20;
    XSH   = 1;
    YSH   = 10;
    
    TSBITS = int32(2^TIMESTAMP_BITS-1); %all but most significant bit
    POLBIT = int32(2^POLSH); %0th bit
    CHBIT  = int32(2^CHSH);
    XBITS  = bitshift(int32(2^9-1), XSH);
    YBITS  = bitshift(int32(2^8-1), YSH);
    
elseif(CODEC_TYPE == 2)
    
    CH = 1; TS = 2; POL = 3; X = 4; Y = 5; BOTTLE_START = 6; YARP_TS = 7;
    
    POLSH = 0;
    CHSH  = 22;
    XSH   = 1;
    YSH   = 12;
    
    TSBITS = int32(2^TIMESTAMP_BITS-1); %all but most significant bit
    POLBIT = int32(2^POLSH); %0th bit
    CHBIT  = int32(2^CHSH);
    XBITS  = bitshift(int32(2^9-1), XSH);
    YBITS  = bitshift(int32(2^8-1), YSH);
    
else
    
    display(['Incorrect CODEC_TYPE ', int2str(CODEC_TYPE)]);
    
end



infile = fopen(infilename, 'r');
if(infile < 0)
    display('Could not open file');
    return;
end
fseek(infile, 0, 'eof');
totch = ftell(infile);
frewind(infile);

if isfile(outfilename)
    delete(outfilename);
end

dth = 0.1;
bi = 0;
vi = 0;
wraps = 0;
pts = 0;

disp('Conversion starting ...');
l = fgetl(infile);
% for i = 1:20000
%     l = fgetl(infile);
% end
while(ischar(l))
    bi = bi + 1;
    
    yarp_stamp = sscanf(l, '%f', 2); yarp_stamp = yarp_stamp(2);
    l = l(find(l == '(')+1:find(l == ')')-1);
    bottle = sscanf(l, '%i %i');
    temp = length(bottle) / 2;
    vi = vi + length(bottle) / 2;
    bottle = reshape(bottle, 2, length(bottle)/2)';
    
    if(CODEC_TYPE == 0)
        errors = bottle(:, 1) > 0 | bottle(:, 2) < 0 | bottle(:, 2) > 65535;
        if(sum(errors))
            display(['Bottle Data Corrupt (' int2str(sum(errors)) ') in ' ...
                'bottle index: ' int2str(bi)]);
            bottle = bottle(~errors, :);
        end
    end
    
    textformat = zeros(size(bottle, 1), 7);
    textformat(:, TS) = bitand(int32(bottle(:, 1)), TSBITS) + wraps * MAXSTAMP;
    textformat(:, CH) = bitshift(bitand(int32(bottle(:, 2)), CHBIT), -CHSH);
    textformat(:, POL) = bitshift(bitand(int32(bottle(:, 2)), POLBIT), -POLSH);
    textformat(:, X) = bitshift(bitand(int32(bottle(:, 2)), XBITS), -XSH);
    textformat(:, Y) = bitshift(bitand(int32(bottle(:, 2)), YBITS), -YSH);    
    textformat(1, BOTTLE_START) = 1;
    
    %check for wraps
    %between two bottles
    if textformat(1, TS) < pts
       %there was a wrap here
       wraps = wraps + 1;
       orig_ts = textformat(1, TS);
       textformat(:, TS) = textformat(:, TS) + MAXSTAMP;
       disp(['Wrap occured between packets: ' int2str(pts) ' ' int2str(orig_ts) ' -> ' int2str(textformat(1, TS))]);
    end
    pts = textformat(end, TS);
    
    %within bottle
    wrapcheck = diff(textformat(:, TS)) < 0;
    if sum(wrapcheck)
       %there was a wrap here
       wraps = wraps + 1;
       wi = find(wrapcheck);
       orig_ts = textformat(wi+1, TS);
       textformat(wi+1:end, TS) = textformat(wi+1:end, TS) + MAXSTAMP;
       disp(['Wrap occured within a packet: correct to ' int2str(textformat(wi, TS)) ' ' int2str(orig_ts) ' -> ' int2str(textformat(wi+1, TS))]);
    end
    
    %post process the yarp_stamps
    time_deltas = textformat(:, TS) - textformat(end, TS);
    textformat(:, YARP_TS) = time_deltas * CLOCK_PERIOD + yarp_stamp;
    
    if(size(textformat, 1) ~= temp)
        display('error');
    end
    
    dlmwrite(outfilename, textformat, '-append', 'delimiter', ' ', 'precision', 20);
    
    if(ftell(infile) / totch > dth)
        display([int2str(100 * ftell(infile) / totch) '% done']);
        dth = dth + 0.1;
    end
    
    l = fgetl(infile);
    
end

fclose(infile);

display(['100% done (' int2str(bi) ' bottles and ' int2str(vi) ' events)']);
