% Converts and saves events from aedat format to txt
% Author: Valentina Vasco

filenameIn = 'test.aedat';
filenameOut = 'test.txt';

[allAddr,allTs] = loadaerdat(filenameIn); % loads events from a .dat file
[x, y, pol, eye] = extractStereoRetina128EventsFromAddr(allAddr); % extracts retina events from addr vector

fid = fopen(filenameOut, 'wt');
for i=1: size(x,1)
    % print (channel, timestamp, polarity, x, y) 
    fprintf(fid, '%1i %1i %1i %1i %1i\n', eye(i), allTs(i), pol(i), x(i), y(i));
end
fclose(fid);
