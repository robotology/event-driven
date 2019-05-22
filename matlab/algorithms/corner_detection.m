%% creating the harris scores %%

clc;
clear;

%input file
filename = '/home/vvasco/Dropbox/IROS2016/datasets/checker/0-10.txt';

% %output file
% resultfile = '/home/vvasco/dev/egomotion/new datasets/testing data/teabox/0_2/corners_teabox.txt';
% fid = fopen(resultfile, 'wt');

%sensor's parameters
width = 304;
height = 240;

%number of events to process
N = 2000;

%size of the spatial window: 2*L+1
L = 7; 
filterSize = 7;

%load in events
events = importdata(filename);

%remove unneeded events (polarity etc)
% events(events(:, 1) ~= 0, :) = [];
% events(events(:, 3) ~= 1, :) = [];

%convert to seconds
% events(:, 2) = (events(:, 2) - events(1, 2))./1000000;

%initialise data structures
vSurfON = zeros(height, width);
vSurfOFF = zeros(height, width);

%Sobel Kernels
for i = 1 : filterSize
    Sx(i) = factorial((filterSize - 1))/((factorial((filterSize - 1) - (i - 1)))*(factorial(i - 1)));
    Dx(i) = Pasc(i - 1, filterSize - 2) - Pasc(i - 2, filterSize - 2);
end
Sy = Sx';
Dy = Dx';
Gx = Sy(:)*Dx;
Gy = Gx';
Gx = Gx/max(max(Gx));
Gy = Gy/max(max(Gy));

%gaussian kernel
sigma = 1;
A = 1/(2*pi*sigma^2);
hsize = 2*L + 1 - filterSize + 1;
[xw, yw] = meshgrid(-(hsize-1)/2:(hsize-1)/2, -(hsize-1)/2:(hsize-1)/2); 
% [xw, yw] = meshgrid(-round((L - 1)/2):round((L - 1)/2), -round((L - 1)/2):round((L - 1)/2)); 
h = A * exp(-(xw.^2 + yw.^2) / (2*sigma^2));
h = h/sum(sum(h));

perc = 10;
for i = 1:size(events, 1)

    %current event
    xi = events(i, 4);
    yi = events(i, 5);
    tsi = events(i, 2);
    poli = events(i, 3);
    
    %process differently for different polarities 
    if(poli == 0) %if ON event
        
        %update the surface
        vSurfON(yi + 1, xi + 1) = tsi;
        
        %if it contains more than N events, remove the oldest
        if(sum(sum(vSurfON > 0)) > N)
            vSurfON(vSurfON == min(vSurfON(vSurfON ~= 0))) = 0;
        end
        
        %the computation is not done on these events, but they are still
        %added to the surface and thus used to detect corner events
        if(xi <= L || xi >= width - L || yi <= L || yi >= height - L)
            continue;
        end
        
        %PROCESS THE HARRIS ON THIS LOCATION
        windEvts = vSurfON(yi + 1 - L : yi + 1 + L, xi + 1 - L : xi + 1 + L) > 0;
                
        dx = conv2(double(windEvts), Gx, 'valid'); %/sum(sum(abs(Gx)));
        dy = conv2(double(windEvts), Gy, 'valid'); %/sum(sum(abs(Gy)));
        
        %square derivatives
        dx2 = dx.^2;
        dy2 = dy.^2;
        dxy = dx.*dy;
        
        %apply gaussian
        dx2 = dx2.*h;
        dy2 = dy2.*h;
        dxy = dxy.*h;
        
        %create harris matrix
        a = sum(sum(dx2));
        d = sum(sum(dy2));
        b = sum(sum(dxy));
        c = b;
        
        M = [a b;
            c d];
        score = (det(M) - 0.04*(trace(M) ^ 2));
        
    else %if OFF event
        
        %update the surface
        vSurfOFF(yi + 1, xi + 1) = tsi;
        
        %if it contains more than N events, remove the oldest
        if(sum(sum(vSurfOFF > 0)) > N)
            vSurfOFF(vSurfOFF == min(vSurfOFF(vSurfOFF ~= 0))) = 0;
        end
        
        %the computation is not done on these events, but they are still
        %added to the surface and thus used to detect corner events
        if(xi <= L || xi >= width - L || yi <= L || yi >= height - L)
            continue;
        end
        
        %PROCESS THE HARRIS ON THIS LOCATION
        windEvts = vSurfOFF(yi + 1 - L : yi + 1 + L, xi + 1 - L : xi + 1 + L) > 0;
                  
        dx = conv2(double(windEvts), Gx, 'valid'); %/sum(sum(abs(Gx)));
        dy = conv2(double(windEvts), Gy, 'valid'); %/sum(sum(abs(Gy)));
        
        %square derivatives
        dx2 = dx.^2;
        dy2 = dy.^2;
        dxy = dx.*dy;
        
        %apply gaussian
        dx2 = dx2.*h;
        dy2 = dy2.*h;
        dxy = dxy.*h;
        
        %create harris matrix
        a = sum(sum(dx2));
        d = sum(sum(dy2));
        b = sum(sum(dxy));
        c = b;
        
        M = [a b;
            c d];
        score = (det(M) - 0.04*(trace(M) ^ 2));
    end
  
    %display percentage
    if( ( ( i / size(events, 1) ) * 100 ) > perc )
        disp([int2str( (i / size(events, 1)) * 100 ) '% done']);
        perc = perc + 10;
    end
      
    %attach Harris score to this event in an array
    events(i, 7) = score;
         
end

%save events with scores 
dlmwrite(resultfile, events, 'delimiter', ' ');


%%%%%%%%%%%%%%%%%%%%%
%  local functions  %
%%%%%%%%%%%%%%%%%%%%%
function P=Pasc(k,n)
if (k>=0)&&(k<=n)
    P=factorial(n)/(factorial(n-k)*factorial(k));
else
    P=0;
end
end