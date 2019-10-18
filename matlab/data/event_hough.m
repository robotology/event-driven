function [x_best, y_best, r_best, v_best] = event_hough(input_events, r_min, r_max, height, width)
%EVENT_HOUGH Summary of this function goes here
%   Detailed explanation goes here
    v_best = -1e10;
    
    for R = r_min:r_max
        image = zeros(height + R*2, width + R*2);
        template = zeros(R*2+1, R*2+1);
        
        %create the template
        for x = -R:R
            for y = -R:R
                if abs(sqrt(x*x + y*y) - R) < 2
                    template(y+R+1, x+R+1) = 1;
                end
                if sqrt(x*x + y*y) < (R - 2)
                    template(y+R+1, x+R+1) = -0.5;
                end
            end
        end
        
        %for each event add the shifted template to the image       
        for i = 1:size(input_events, 1)           
            x = input_events(i, 4)+1; y = input_events(i, 5)+1;
            image( y:y+2*R, x:x+2*R) = image( y:y+2*R, x:x+2*R) + template;
        end            
        
        for x = R:size(image, 2)-R
            for y = R:size(image, 1)-R
                if image(y, x) > v_best
                    x_best = x -R;
                    y_best = y - R;
                    r_best = R;
                    v_best = image(y, x);
%                      figure(10); clf; hold on;
%                      imagesc(image(R:R+height, R:R+width));
%                      plot(x_best, y_best, 'xk', 'markersize', 10);
                end
            end
        end
          
    end


end

