%% Authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>

close all;

% these are the ones you need to set up
% probably wirte a higher level script which iterates in between these
roi_log_file = '/home/leandro/results/RegionOfInterest/6DOF_advanced_ego/RoundTable/roiLog.txt';
benchmark_folder = '/home/leandro/dumpATIS/6DOF/roundTableCube_6DOF_001_convertedCropped';

%folder to save the results
processed_data_folder = [benchmark_folder '/processedData'];
results_folder = [benchmark_folder '/results'];

if ~exist(processed_data_folder, 'dir')
   mkdir(processed_data_folder)
end

if ~exist(results_folder, 'dir')
   mkdir(results_folder)
end


%% preprare and  run the conversion script

CODEC_TYPE = 2;
TIMESTAMP_BITS = 31;
CLOCK_PERIOD =  0.000000080;

infilename = [benchmark_folder '/ATIS/rightdvs/data.log'];
outfilename = [processed_data_folder '/converted.txt'];

if ~isfile(outfilename)
    run('./vbottletotxt_AE.m')
else 
    disp('file already converted')
end

%% load the RoI data and filter out the events which were filtered by vPreProcess

converted_file_name = outfilename;

% Load RoI log file
if( ~exist('roi_log_data', 'var') )
    roi_log_data = importdata(roi_log_file);
end

%loads {pol, x, y, ts(continuous)} from roiLog
filter_data = roi_log_data(:, [2, 3, 4, 7]);

filtered_file = [processed_data_folder '/filtered.txt'];

if( ~exist('filtered_events', 'var') )
    if isfile(filtered_file)
        disp('loading filtered data');
        filtered_events = importdata(filtered_file);
    else
        disp('filtering events')
        if( ~exist('GTevents', 'var') )
            GTevents = importdata(converted_file_name); % import data
        end

        %loads {pol, x, y, ts(continuous)} from GTevents
        to_filter = GTevents(:, [3, 4, 5, 2]);

        % filters the events that are not present in the RoI events dump
        [idx, num] = ismember(to_filter, filter_data, 'rows');
        filtered_events = GTevents(idx,:);

        %removes repeated elements
        if( size(filter_data, 1) ~= size(filtered_events, 1) )
            disp('applying unique filter');
            [filtered_events, ife, ic] = unique(filtered_events, 'rows');

            % this is just to study the repreated events
    %         disp('looking for repeated events')
    %         to_check = GTevents(:, [4, 5, 2]);
    %         [idx, num] = ismember(to_check, to_check, 'rows');
    %         member_idx = num(idx);
    %         [idc, val] = hist(member_idx, unique(member_idx));
    %         repeated_filter = val(idc>1);
    %         repeated_idx = ismember(to_check, to_check(repeated_filter,:), 'rows');
    %         repeated_data = to_check(repeated_idx, :)
    %         return;
        end

        dlmwrite(filtered_file, filtered_events, 'delimiter', ' ','precision', 20);
        disp('filtered events saved');
    end
end

%% Prepare and run the groundthruther

GTdataset = filtered_file;
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

clearvars -except GTevents GTdataset counter_clock GTresultfile ...
    sensor_width sensor_height roi_log_file roi_log_data filtered_events ...
    results_folder outfilename;

%% Apply cubic spline to interpolate ground truth
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

events_ts = filtered_events(:, 2); % in clocks
events_times = filtered_events(:, 7); % in seconds

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


%% compare roi_log_data [class, pol, x, y, ts, roi_ts, ts_cont, roi_ts_const]
% against [xx yy rr events_times] 

%check timestamp alignment
if all(roi_log_data(idx, 7) == events_ts)
    disp('timestamps are aligned')
else
    disp('timestamps are not aligned')
    return;
end

distance_from_centre = sqrt((roi_log_data(idx, 3)-xx).^2 + (roi_log_data(idx, 4)-yy).^2);
ground_classification = distance_from_centre <= rr;
classification = roi_log_data(idx, 1);

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

%% align RoI with ES

% add the x y r according to the roi_ts to compensate the delay
undelayed_data = [roi_log_data(idx, :) zeros(numel(xx), 3)];
roits = undelayed_data(:, 8);

old = -1;
for row=1:numel(xx)
    if(roits(row) ~= old)
        old = roits(row);
        idtime = find( events_ts == old, 1, 'first' );
    end
    
    if(~isempty(idtime))
        undelayed_data(idtime, 9) = xx(idtime);
        undelayed_data(idtime, 10) = yy(idtime);
        undelayed_data(idtime, 11) = rr(idtime);
    end
end

nzero = undelayed_data(:, 9) > 0;
undelayed_data = undelayed_data(nzero, :);  

distance_from_centre = sqrt((undelayed_data(:, 3)-undelayed_data(:, 9)).^2 + ...
    (undelayed_data(:, 4)-undelayed_data(:, 10)).^2);
uground_classification = distance_from_centre <= undelayed_data(:, 11);
uclassification = undelayed_data(:, 1);

ugt = uground_classification == 1 ;
ugf = uground_classification == 0;
uct = uclassification == 1;
ucf = uclassification == 0;

utp = ugt & uct;
ufp = ugf & uct;
utn = ugf & ucf;
ufn = ugt & ucf;

%confmatrix = 100*[sum(tp), sum(fp); sum(fn), sum(tn)]./numel(xx)

close all;
plotconfusion(uclassification', uground_classification')
xlabel('Pipeline classification removing RoI delay')
ylabel('Ground Truth classification')
title(['1/0 - inside/outside RoI'])
exportgraphics(gcf,[results_folder '/undelayed_confusion_matrix.png'])

figure(2)
uaverage_confusion_evo = [cumsum(utp), cumsum(ufp), cumsum(ufn), cumsum(utn)]./[1:1:numel(uclassification)]';
plot(undelayed_data(:, 7)*counter_clock, uaverage_confusion_evo)
xlabel('time (s)')
ylabel('(%)')
legend(['tp'; 'fp'; 'fn'; 'tn'], 'Orientation', 'horizontal', 'Location','NorthOutside')
title('Removing RoI delay');
exportgraphics(gcf,[results_folder '/undelayed_confusion_matrix_x_time.png'],'Resolution',300)
hold off;

close all;
clearvars -except GTevents GTdataset counter_clock GTresultfile ...
    sensor_width sensor_height roi_log_file roi_log_data filtered_events ...
    results_folder undelayed_data idx xx yy rr;


%% Make a vide

videoObj = VideoWriter([results_folder '/video.avi']);
videoObj.FrameRate = 30;
frameTime = 1/videoObj.FrameRate;
open(videoObj);

% xx yy rr - roi at events
% filtered_events(idx,:) events data
% roi_log_data(idx, 1) - classification
% iltered_events(:, [4, 5, 2]) {x, y, ts(continuous)}

events = filtered_events(idx,:);
classification = roi_log_data(idx, 1);
ts = filtered_events(idx,2)*counter_clock;
pos = [xx-rr yy-rr 2*rr 2*rr];

lb = 1;
ti = ts(1);
figure('visible','off'); hold on;

oldTime = 0;
while(ti < ts(end))
    ub = find(ts > ti+frameTime, 1);
    if(isempty(ub))
        ub = numel(ts);
        te = ts(end);
    else
        te = ts(ub - 1);
    end
    
    temp_events = events(lb:ub, 4:5);
    temp_classification = classification(lb:ub);
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
    
    currTime = floor(ts(ub));
    if(currTime > oldTime)
        disp(['processed ' num2str(currTime) ' seconds']);
        oldTime = currTime;
    end
    
    lb = ub;
    ti = ts(ub);
end

close(videoObj);
close all;
disp('done =)')