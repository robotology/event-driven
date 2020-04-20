%% Authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
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
clearvars -except convertedfilename

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

if ~isfile(GTresultfile)
    run('./groundtruther.m')
else
    disp('already groundtruthed')
end
clearvars -except GTdataset GTresultfile counter_clock sensor_height sensor_width = 304;

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
disp('reloading converted file')
GTevents = importdata(GTdataset); % import data
events_ts = GTevents(:, 2) * counter_clock;

%calculate the interpolated x,y,r
xx = ppval(cx, events_ts);
yy = ppval(cy, events_ts);
rr = ppval(cr, events_ts);