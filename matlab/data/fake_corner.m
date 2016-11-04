% Creates a fake corner starting from P0-P1 and P0-P2 lines
% P0 = (x0, y0)
% P1 = (x0, yf)
% P2 = (xf, y0)
% Author: Valentina Vasco

E = zeros(100000, 5);

fid = fopen('fake_corner.txt', 'wt');

% starting positions
x0 = 20;
y0 = 20;
xf = 80;
yf = 80;

y1 = y0:(yf - 1);
y1 = y1';
x1 = x0*ones(length(y1), 1);

x2 = x0:(xf - 1);
x2 = x2';
y2 = y0*ones(length(y1), 1);

t = 1000000*ones(length(y1), 1);
ch = ones(length(y1), 1);
pol = ones(length(y1), 1);

E = [ch t pol x1 y1; ch t pol x2 y2];

while (x1 < 128 & y1 < 128)
    x1 = x1 + 1;
    y1 = y1 + 1;
    x2 = x2 + 1;
    y2 = y2 + 1;
    t = t + 100000;
    E = [E; ch t pol x1 y1; ch t pol x2 y2];
            
    plot(E(:, 4), E(:, 5),'.');
    set(gca, 'xlim', [0 128]);
    set(gca, 'ylim', [0 128]);
    drawnow;
    
end

% print a txt file with (channel, timestamp, polarity, x, y)
for i=1:size(E, 1)
    fprintf(fid, '%1i %1i %1i %1i %1i\n', E(i, 1), E(i, 2), E(i, 3), E(i, 4), E(i, 5));
end

fclose(fid);





