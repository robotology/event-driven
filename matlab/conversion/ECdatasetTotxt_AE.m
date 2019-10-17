%% Initialization
CLOCK_PERIOD_YARP = 0.000000080; % 80 nano-seconds
BOTTLE_RATE = 0.004; % 4 ms

%% Read an events.txt from the Event-Camera Dataset
% Event-Camera Dataset web page: http://rpg.ifi.uzh.ch/davis_data.html
% The events in the dataset of Scaramuzza's group has the following format:
% TS X Y POL
infilename = '/home/adinale/Documents/Event-Driven Dataset/scaramuzza/shapes_rotation/events.txt';

data = dlmread(infilename);
[rows, cols] = size(data);

%% Re-arrange the input events as 'vbottletotxt_AE.m' does
% The desired format is the following: 
% CH TS POL X Y BOTTLE_START YARP_TS
CH = 1; TS = 2; POL = 3; X = 4; Y = 5; BOTTLE_START = 6; YARP_TS = 7;

dataOrdered = zeros(rows, 7);
dataOrdered(:,CH) = ones(rows, 1); % CHANNEL = 1
dataOrdered(:,TS) = round(data(:,1)/CLOCK_PERIOD_YARP);
dataOrdered(:,POL) = data(:,4);
dataOrdered(:,X:Y) = data(:,2:3);
dataOrdered(:,YARP_TS) = data(:,1);

% YARP_TS is a number growing coherently with TS
% BOTTLE_START is a flag indicating when a packet of information starts

%% Set the BOTTLE_START flag depending on the desired RATE
dataOrdered(1, BOTTLE_START) = 1;
countBottles = 1;
for i = 2:rows    
    if (dataOrdered(i, YARP_TS) >= BOTTLE_RATE * countBottles)
        countBottles = countBottles + 1;
        dataOrdered(i, BOTTLE_START) = 1;        
    end    
end

%% Write the converted data into a new file
outfilename = 'eventsLog.txt';

if isfile(outfilename)
    delete(outfilename);
end

fid = fopen(outfilename ,'w+');
for i = 1:rows
    fprintf(fid,'%g ',dataOrdered(i,CH));
    fprintf(fid,'%10.f ',dataOrdered(i,TS));
    fprintf(fid,'%g ',dataOrdered(i,POL:BOTTLE_START));
    fprintf(fid,'%10.10f\n',dataOrdered(i,YARP_TS));          
end
fclose(fid);
