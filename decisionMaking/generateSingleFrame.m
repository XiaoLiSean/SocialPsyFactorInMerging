function generateSingleFrame(initialParams, figname)
    
    x0_vehicles     = initialParams.x0_vehicles(:,1);
    num_vehicles    = length(x0_vehicles);
    LaneParams      = initialParams.LaneParams;
    RoadBound       = LaneParams.RoadBound;
    num_lanes       = length(LaneParams.lane_ids);
    

    fig     = figure('visible', 'off');
    set(gcf, 'units', 'centimeters', 'position', [0, 0, 40, 6+num_lanes])
    set(gca, 'fontsize', 20);    

    for lane_i = LaneParams.lane_ids
        plot([LaneParams.leftBound, LaneParams.rightBound], LaneParams.lane_width.*lane_i.*[1, 1], '--', 'color', [0.75 0.75 0.75], 'LineWidth', 3);hold on;
        if lane_i == 0                
            plot([LaneParams.leftBound, LaneParams.rampStartX], [0, 0], 'k-', 'LineWidth', 4);
        end
    end
    
    plot(RoadBound(1:2,1),RoadBound(1:2,2),'k-','LineWidth',4); 
    plot(RoadBound(3:end,1),RoadBound(3:end,2),'k-','LineWidth',4);

    for v_i = 1:num_vehicles
        state   = x0_vehicles{v_i};
        w_car   = initialParams.vehiclesDims(2, v_i);
        l_car   = initialParams.vehiclesDims(1, v_i);
        [x, y, v, theta]    = deal(state(1), state(2), state(3), 0);
        rectangles  = [ x-l_car/2*cos(theta)-w_car/2*sin(theta), y-l_car/2*sin(theta)+w_car/2*cos(theta);
            x-l_car/2*cos(theta)+w_car/2*sin(theta), y-l_car/2*sin(theta)-w_car/2*cos(theta);
            x+l_car/2*cos(theta)-w_car/2*sin(theta), y+l_car/2*sin(theta)+w_car/2*cos(theta);
            x+l_car/2*cos(theta)+w_car/2*sin(theta), y+l_car/2*sin(theta)-w_car/2*cos(theta);
            x+(l_car/2-1)*cos(theta)-w_car/2*sin(theta), y+(l_car/2-1)*sin(theta)+w_car/2*cos(theta);
            x+(l_car/2-1)*cos(theta)+w_car/2*sin(theta), y+(l_car/2-1)*sin(theta)-w_car/2*cos(theta)];

        plot([rectangles(1,1) rectangles(2,1)],[rectangles(1,2) rectangles(2,2)],'-','LineWidth',2,'Color','b');
        plot([rectangles(1,1) rectangles(3,1)],[rectangles(1,2) rectangles(3,2)],'-','LineWidth',2,'Color','b');
        plot([rectangles(3,1) rectangles(4,1)],[rectangles(3,2) rectangles(4,2)],'-','LineWidth',2,'Color','b');
        plot([rectangles(2,1) rectangles(4,1)],[rectangles(2,2) rectangles(4,2)],'-','LineWidth',2,'Color','b');
        plot([rectangles(5,1) rectangles(6,1)],[rectangles(5,2) rectangles(6,2)],'-','LineWidth',2,'Color','b');
        text(x-0.4*l_car, y, [num2str(v_i)]);
        text(x-l_car, y+w_car, sprintf('%0.1f m/s', v));
    end
    
    axis([LaneParams.leftBound, LaneParams.rightBound,...
        LaneParams.RoadBound(3,2)-LaneParams.lane_width,...
        LaneParams.RoadBound(1,2)+LaneParams.lane_width])
    xlabel('[m]')
    ylabel('[m]')    
    saveas(fig, figname, 'png');
    close(fig);

end