%% Authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
close all;
roi_log_file = '/home/leandro/results/RegionOfInterest/6DOF_advanced_ego/roiLog.txt';
scores_file = '/home/leandro/results/RegionOfInterest/6DOF_advanced_ego/scores.txt'; 
infilename = '/home/leandro/dumpATIS/6DOF/flyingCube_independent_001_converted/ATIS/rightdvs/data.log';

CODEC_TYPE = 2;
TIMESTAMP_BITS = 31;
CLOCK_PERIOD =  0.000000080;
convertedfilename = [infilename '.txt'];

if ~isfile(convertedfilename)
    run('../conversion/vbottletotxt_AE.m')
else 
    disp('file already converted')
end

%filter out events that were filtered out
% Load RoI log file
roi_log_data = importdata(roi_log_file);
filter_data = roi_log_data(:, [2, 3, 6]);
filter_data(1:10, :)'

converted_file = [convertedfilename '.conv'];
if ~isfile(converted_file)
    if( ~exist('GTevents', 'var') )
        GTevents = importdata(GTdataset); % import data
    end
    to_filter = GTevents(:, [4, 5, 2]);
    to_filter(1:30, :)'
    [idx, num] = ismember(to_filter, filter_data, 'rows');
    idx(1:30)'
    num(1:30)'
    filtered_events = GTevents(idx,:);
end

return;
GTdataset = converted_file;
GTresultfile = strcat(converted_file,'.GT');

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

clearvars -except GTevents GTdataset counter_clock GTresultfile ...
    sensor_width sensor_height  roi_log_file scores_file roi_log_data;

plotRoI = false;

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

if(plotRoI)
    plotRate = 1000;
    %plot interpolation for debugging
    figure(1)
    axis ij
    hold on;

    axis([0 sensor_width 0 sensor_height]);
    % plot the interpolated points
    pos = [xx-rr yy-rr 2*rr 2*rr];
    [rows, ~] = size(pos);
    for i=1:plotRate:rows
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
    exportgraphics(gcf,'/home/leandro/results/GroundTruther/interpolation_test1.png','Resolution',300)
    hold off;
end



disp('done =)')