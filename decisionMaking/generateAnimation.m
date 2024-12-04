function generateAnimation(filename)

    load(filename, 'highwaySim');
    outfile                 = filename;
    outfile(end-2:end)      = 'gif';
    
    X_all   = highwaySim.vizHistory;
    dt      = highwaySim.simParams.dt;
    num_vehicles    = length(highwaySim.Vehicles);
    LaneParams      = highwaySim.LaneParams;
    RoadBound       = LaneParams.RoadBound;
    num_lanes       = length(LaneParams.lane_ids);
    
    progressBar     = waitbar(0, 'Generating Animation');
    pause(0.001);
    for time_i = 1:size(X_all, 2)
        fig     = figure('visible', 'off');
        set(gcf, 'units', 'centimeters', 'position', [0, 0, 40, 6+num_lanes])
        set(gca, 'fontsize', 20);
        
        waitbar(time_i/size(X_all, 2), progressBar, ['Generating Animation',repelem('.', 1, mod(time_i, 4))]); 

        for lane_i = LaneParams.lane_ids
            plot([LaneParams.leftBound, LaneParams.rightBound], LaneParams.lane_width.*lane_i.*[1, 1], '--', 'color', [0.75 0.75 0.75], 'LineWidth', 3);hold on;
            if lane_i == 0                
                plot([LaneParams.leftBound, LaneParams.rampStartX], [0, 0], 'k-', 'LineWidth', 4);
            end
        end
        
        plot(RoadBound(1:2,1),RoadBound(1:2,2),'k-','LineWidth',4); 
        plot(RoadBound(3:end,1),RoadBound(3:end,2),'k-','LineWidth',4);

        states  = X_all(:, time_i);
        strings = [];
        for v_i = 1:num_vehicles
            if highwaySim.ego_id ~= 0 && (v_i == highwaySim.ego_id)
                car_color   = 'r';
            else
                car_color   = 'b';
            end
            w_car   = highwaySim.Vehicles{v_i}.vecWidth;
            l_car   = highwaySim.Vehicles{v_i}.vecLength;
            [x, y, v, theta]    = deal(states(4*v_i-3), states(4*v_i-2), states(4*v_i-1), states(4*v_i));
            rectangles  = [ x-l_car/2*cos(theta)-w_car/2*sin(theta), y-l_car/2*sin(theta)+w_car/2*cos(theta);
                x-l_car/2*cos(theta)+w_car/2*sin(theta), y-l_car/2*sin(theta)-w_car/2*cos(theta);
                x+l_car/2*cos(theta)-w_car/2*sin(theta), y+l_car/2*sin(theta)+w_car/2*cos(theta);
                x+l_car/2*cos(theta)+w_car/2*sin(theta), y+l_car/2*sin(theta)-w_car/2*cos(theta);
                x+(l_car/2-1)*cos(theta)-w_car/2*sin(theta), y+(l_car/2-1)*sin(theta)+w_car/2*cos(theta);
                x+(l_car/2-1)*cos(theta)+w_car/2*sin(theta), y+(l_car/2-1)*sin(theta)-w_car/2*cos(theta)];

            plot([rectangles(1,1) rectangles(2,1)],[rectangles(1,2) rectangles(2,2)],'-','LineWidth',2,'Color',car_color);
            plot([rectangles(1,1) rectangles(3,1)],[rectangles(1,2) rectangles(3,2)],'-','LineWidth',2,'Color',car_color);
            plot([rectangles(3,1) rectangles(4,1)],[rectangles(3,2) rectangles(4,2)],'-','LineWidth',2,'Color',car_color);
            plot([rectangles(2,1) rectangles(4,1)],[rectangles(2,2) rectangles(4,2)],'-','LineWidth',2,'Color',car_color);
            plot([rectangles(5,1) rectangles(6,1)],[rectangles(5,2) rectangles(6,2)],'-','LineWidth',2,'Color',car_color);
            text(x-0.4*l_car, y, [num2str(v_i)]);
            if num_vehicles > 4 % otherwise window overflow
                text(x-l_car, y+w_car, sprintf('%0.1f m/s', v));
            end
            strings     = [strings, sprintf('v_{%1d} = %0.1f m/s    ', v_i, v)];
        end
        
        axis([LaneParams.leftBound, LaneParams.rightBound,...
            LaneParams.RoadBound(3,2)-LaneParams.lane_width,...
            LaneParams.RoadBound(1,2)+LaneParams.lane_width])
        xlabel('[m]')
        ylabel('[m]')
        
        annotation('textbox', [0.133 0.85 0.09 0.12],...
            'String', {['t = ' num2str((time_i-1)*dt, '%2.2f') ' s']},...
            'FontSize', 20, 'FontName', 'Arial',...
            'LineStyle', '-', 'LineWidth',2,...
            'EdgeColor', [0.85 0.85 0.85],...
            'BackgroundColor', [0.95  0.95 0.95],...
            'Color','k');
        
        if num_vehicles <= 4 % otherwise window overflow
            annotation('textbox', [0.3 0.85 0.13*num_vehicles 0.12],...
                'String', strings,...
                'FontSize', 20, 'FontName', 'Arial',...
                'LineStyle', '-', 'LineWidth', 2,...
                'EdgeColor', [0.85 0.85 0.85],...
                'BackgroundColor', [0.95  0.95 0.95],...
                'Color', 'k');
        end
        
        frame       = getframe(fig);
        im          = frame2im(frame);
        [imind, cm] = rgb2ind(im,256);
        if time_i == 1
            imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'loopcount', inf);
        else
            imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'writemode', 'append');
        end
        close(fig);
    end
    close(progressBar)
end