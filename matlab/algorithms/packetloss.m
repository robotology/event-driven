if(~exist('vbottlefilename', 'var'))
    display('Please specify data file in "vbottlefilename" variable');
    return;
end

fid = fopen(vbottlefilename);
if(fid < 0)
    display('Could not open file');
    return;
end
fseek(fid, 0, 'eof');
totch = ftell(fid);
frewind(fid);
dth = 0.1;

bottleindicies = [];
l = fgetl(fid);
while(ischar(l))
    
    bi = sscanf(l, '%i', 1);
    bottleindicies = [bottleindicies bi];
    
    if(ftell(fid) / totch > dth)
        display([int2str(100 * ftell(fid) / totch) '% done']);
        dth = dth + 0.1;
    end
    
    l = fgetl(fid);
    
end

display('100% done');
fclose(fid);

figure(1); clf; hold on;
plot(diff(bottleindicies));
dropsec = sum(diff(bottleindicies)-1) / (bottleindicies(end) - bottleindicies(1));
display([num2str(dropsec) '% bottles dropped']);
