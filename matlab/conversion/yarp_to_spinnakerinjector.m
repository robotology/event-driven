if(~exist('infilename', 'var'))
    display('Please specify the path to the dataset in parameter "infilename"');
    return;
end
outfilename = [infilename '.spiking.txt'];

display(['Reading from file ' infilename]);
display(['Writing to file ' outfilename]);

    CH = 1;
    TS = 2;
    POL = 3;
    X = 4;
    Y = 5;
    
    POLSH = 0;
    CHSH  = 20;
    XSH   = 1;
    YSH   = 10;
    
    TSBITS = int32(2^31-1); %all but most significant bit
    POLBIT = int32(2^POLSH); %0th bit
    CHBIT  = int32(2^CHSH);
    XBITS  = bitshift(int32(2^9-1), XSH);
    YBITS  = bitshift(int32(2^8-1), YSH);


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

clear spike_train;
spike_train = cell(240, 304);
ts0 = -1;

l = fgetl(infile);
while(ischar(l))
    bi = bi + 1;
    
    l = l(find(l == '(')+1:find(l == ')')-1);
    bottle = sscanf(l, '%i %i');
    temp = length(bottle) / 2;
    vi = vi + length(bottle) / 2;
    bottle = reshape(bottle, 2, length(bottle)/2)';

    
    for i = 1:temp
       
        x = bitshift(bitand(int32(bottle(i, 2)), XBITS), -XSH)+1;
        y = bitshift(bitand(int32(bottle(i, 2)), YBITS), -YSH)+1;
        ts = bitand(int32(bottle(i, 1)), TSBITS) + wraps * MAXSTAMP;
        
        %check for wraps
        if(ts < pts)
            wraps = wraps + 1;
            ts = ts + MAXSTAMP;
        end
        pts = ts;
        if(ts0 < 0)
            ts0 = ts;
        end
        
        spike_train{y, x} = [spike_train{y, x} ((ts-ts0)*0.000080)];
        
    end    
    
    if(ftell(infile) / totch > dth)
        display([int2str(100 * ftell(infile) / totch) '% done']);
        dth = dth + 0.1;
    end
    
    l = fgetl(infile);
end

fclose(infile);

display(['100% done (' int2str(bi) ' bottles and ' int2str(vi) ' events)']);
display('saving...');

outfile = fopen(outfilename, 'w+');
if(outfile == -1)
    display('COuld not open outputfile');
    return;
end

for y = 1:240
    for x = 1:304
        str = [sprintf('%i;%i;', x, y) sprintf('%d,', spike_train{y, x})];
        str = str(1:end-1);
        %display(str)
        fprintf(outfile, '%s\n', str);
    end
end

fclose(outfile);
