if(~exist('data', 'var'))
    if(~exist('filename', 'var'))
        display('Please specify the path to the dataset in parameter "filename"');
        return;
    end
    
    data = dlmread(filename);
    
    data(data(:, 1) == 0, :) = [];
    
else
    display('Using previous data loaded');
end


figure(1); clf;


p = 800;

for i = 1:p:size(data, 1)
    
    plot(data(i:i+p, 4), data(i:i+p, 5), '.k');
    set(gca, 'xlim', [0 304]);
    set(gca, 'ylim', [0 240]);
    
    drawnow;
    %pause(0.1);
    
    
    
end