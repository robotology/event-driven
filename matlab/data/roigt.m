%% Authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
close all;

infilename = '/home/leandro/data/data/data.log';
CODEC_TYPE = 2;
TIMESTAMP_BITS = 31;
CLOCK_PERIOD =  0.000000080;
convertedfilename = [infilename '.txt'];

if ~isfile(convertedfilename)
    run('../conversion/vbottletotxt_AE.m')
else 
    disp('file already converted')
end

GTdataset = convertedfilename;
GTresultfile = strcat(convertedfilename,'.GT');

winsize = 0.1; %seconds
rate = 0.1; %seconds
channel = 1;
start_offset = 0;
randomised = 0;
counter_clock = 0.00000008;
sensor_height = 240;
sensor_width = 304;
sensitivity = 2;

if ~isfile(GTresultfile)
    run('./groundtruther.m')
else
    disp('already groundtruthed')
end
clearvars -except GTevents GTdataset counter_clock GTresultfile sensor_width sensor_height;

disp('loading ground truth file')

if ~exist('gtdata', 'var')
    gtdata = importdata(GTresultfile);
end

cts = gtdata(:, 1);
x = gtdata(:, 2);
y = gtdata(:, 3);
r = gtdata(:, 4);
cputs  = gtdata(:, 5);

%calculate the coefficients of the polynomyals with cubic spline
cx = spline(cts, x);
cy = spline(cts, y);
cr = spline(cts, r);

% loading the converted data again to avoid changing the groundtruther
% script for the sake of this script
if(~exist('GTevents', 'var'))
    disp('reloading converted file')
    GTevents = importdata(GTdataset); % import data
end
events_ts = GTevents(:, 2);
events_times = events_ts * counter_clock;

% handles the contour conditions in the interpolation
idx = events_times >= cts(1) & events_times <= cts(end);
events_ts = events_ts(idx);
events_times = events_times(idx);

%calculate the interpolated x,y,r
xx = ppval(cx, events_times);
yy = ppval(cy, events_times);
rr = ppval(cr, events_times);

%plot interpolation for debugging
figure(1)
hold on;

axis([0 sensor_width 0 sensor_height]);
% plot the interpolated points
pos = [xx-rr yy-rr 2*rr 2*rr];
[rows, ~] = size(pos);
for i=1:rows
    rectangle('Position',pos(i,:),'Curvature',[1 1], 'FaceColor', '#77AC30', 'Edgecolor','none');
end
plot(xx, yy, '-')

% plot the manually set points
plot(x, y, 'xr');
pos = [x-r y-r 2*r 2*r];
[rows, ~] = size(pos);
for i=1:rows
    rectangle('Position',pos(i,:),'Curvature',[1 1], 'FaceColor', 'none', 'Edgecolor','red');
end
exportgraphics(gcf,'/home/leandro/results/interpolation_test1.png','Resolution',300)
hold off;

disp('done =)')