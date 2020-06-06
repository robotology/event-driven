%% Authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>

close all;

% these are the ones you need to set up
% probably wirte a higher level script which iterates in between these
roi_log_file = '/home/leandro/results/RegionOfInterest/6DOF/flyingCube_independent_001_converted/roiLog.txt';
benchmark_folder = '/home/leandro/dumpATIS/6DOF/flyingCube_independent_001_converted';

%folder to save the results
processed_data_folder = [benchmark_folder '/processedData'];
results_folder = [benchmark_folder '/results'];

if ~exist(processed_data_folder, 'dir')
   mkdir(processed_data_folder)
end

if ~exist(results_folder, 'dir')
   mkdir(results_folder)
end


%% load the RoI data and filter out the events which were filtered by vPreProcess
% Prepare and run the groundthruther

%This is the part that we need to make by parts
% Load RoI log file
if( ~exist('events_data', 'var') )
    disp('loading roi log data');
    %loads {class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_cont} from roiLog
    %events_data = importdata(roi_log_file);
end

GTdataset = roi_log_file;
GTresultfile = [processed_data_folder '/groundTruth.txt'];

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

clearvars -except counter_clock GTresultfile sensor_width sensor_height ...
    roi_log_file events_data filtered_events results_folder;

%% Apply cubic spline to interpolate ground truth
disp('loading ground truth file')

if ~exist('gtdata', 'var')
    %{ts, x, y, r}
    gtdata = importdata(GTresultfile);
end

cts = gtdata(:, 1);
x = gtdata(:, 2);
y = gtdata(:, 3);
r = gtdata(:, 4);

%calculate the coefficients of the polynomyals with cubic spline
cx = spline(cts, x);
cy = spline(cts, y);
cr = spline(cts, r);

%{class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_cont}
if( ~exist('events_data', 'var') )
    disp('Loading events data...')
    events_data = importdata(roi_log_file); % import data
end
events_ts = events_data(:, 7); % in clocks
events_times = events_data(:, 7)*counter_clock; % in seconds

% handles the contour conditions in the interpolation
idx = events_times >= cts(1) & events_times <= cts(end);
events_ts = events_ts(idx);
events_times = events_times(idx);

%calculate the interpolated x,y,r
xx = ppval(cx, events_times);
yy = ppval(cy, events_times);
rr = ppval(cr, events_times);

%% Plot the interpolation for for debugging
plotRoI = true;

if(plotRoI)
    close all;
    plotRate = 1000;
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
    exportgraphics(gcf,[results_folder '/interpolation.png'],'Resolution',300)
    hold off;
    clear pos rows
end

close all;

%% compare events_data [class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_const]
% against [xx yy rr events_times] 

distance_from_centre = sqrt((events_data(idx, 3)-xx).^2 + (events_data(idx, 4)-yy).^2);
ground_classification = distance_from_centre <= rr;
classification = events_data(idx, 1);

gt = ground_classification == 1 ;
gf = ground_classification == 0;
ct = classification == 1;
cf = classification == 0;

tp = gt & ct;
fp = gf & ct;
tn = gf & cf;
fn = gt & cf;

%confmatrix = 100*[sum(tp), sum(fp); sum(fn), sum(tn)]./numel(xx)

close all;
plotconfusion(classification', ground_classification')
xlabel('Pipeline classification')
ylabel('Ground Truth classification')
title(['1/0 - inside/outside RoI'])
exportgraphics(gcf,[results_folder '/confusion_matrix.png'])

figure(2)
average_confusion_evo = [cumsum(tp), cumsum(fp), cumsum(fn), cumsum(tn)]./[1:1:numel(xx)]';
plot(events_times, average_confusion_evo)
xlabel('time (s)')
ylabel('(%)')
legend(['tp'; 'fp'; 'fn'; 'tn'], 'Orientation', 'horizontal', 'Location','NorthOutside')
title('considering RoI delay');
exportgraphics(gcf,[results_folder '/confusion_matrix_x_time.png'],'Resolution',300)
hold off;

clearvars -except counter_clock GTresultfile sensor_width sensor_height ...
    roi_log_file events_data filtered_events results_folder undelayed_data ...
    events_times idx xx yy rr;

close all;

%% Make a video

video_name = [results_folder '/video.avi'];

if(exist(video_name, 'file'))
    disp('Video aready exists');
    return;
end

videoObj = VideoWriter(video_name);

videoObj.FrameRate = 30;
frameTime = 1/videoObj.FrameRate;
open(videoObj);

%{class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_cont}
events = events_data(idx,:);
pos = [xx-rr yy-rr 2*rr 2*rr];

lb = 1;
ti = events_times(1);
figure('visible','off'); hold on;

oldTime = 0;

while(ti < events_times(end))
    ub = find(events_times > ti+frameTime, 1);
    if(isempty(ub))
        ub = numel(events_times);
        te = events_times(end);
    else
        te = events_times(ub - 1);
    end
    
    %{class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_cont}
    temp_events = events(lb:ub, 3:4);
    temp_classification = events(lb:ub, 1);
    inside  = temp_events(temp_classification == 1, :);
    outside = temp_events(temp_classification == 0, :);
    
    clf;
    hold on;
    [r, ~] = size(outside);
    if( r > 0)
        h(1) = plot(outside(:, 1), outside(:, 2), '.k');
    end
    
    [r, ~] = size(inside);
    if( r > 0)
        h(2) = plot(inside(:, 1), inside(:, 2), '.b');
    end
    
    %legend(['classified as outside'; 'classified as inside '], 'Orientation', 'horizontal', 'Location','NorthOutside');
    rectangle('Position',pos(ub,:), 'Curvature',[1 1], 'FaceColor', 'none', 'Edgecolor','red');
    axis ij;
    axis([0 sensor_width 0 sensor_height]);
    frame = getframe;
    writeVideo(videoObj, frame);
    
    currTime = floor(events_times(ub));
    if(currTime > oldTime)
        disp(['processed ' num2str(currTime) ' seconds']);
        oldTime = currTime;
    end
    
    lb = ub;
    ti = events_times(ub);
end

close(videoObj);

clearvars -except counter_clock GTresultfile sensor_width sensor_height ...
    roi_log_file events_data filtered_events results_folder undelayed_data ...
    events_times idx xx yy rr;
close all;
disp('done =)')