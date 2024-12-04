function generateHighDAnimation(params, vizHistory, scene_num, which_side, startFrame, endFrame)

    outfile     = sprintf('./HighDExperiment/HighDResults/highD_%s_%s_%d_%d.gif', scene_num, which_side, startFrame, endFrame);
    dt          = params.simParams.dt;
    num_vecs    = size(vizHistory, 2);
    LaneParams  = params.LaneParams;
    RoadBound   = LaneParams.RoadBound;
    num_lanes   = length(LaneParams.lane_ids);
    
    progressBar     = waitbar(0, 'Generating Animation');
    pause(0.001);
    for frame_i = startFrame:endFrame
        fig     = figure('visible', 'off');
        set(gcf, 'units', 'centimeters', 'position', [0, 0, 60, 6+num_lanes])
        set(gca, 'fontsize', 20);
        
        waitbar((frame_i - startFrame)/(endFrame - startFrame),...
            progressBar, ['Generating Animation', repelem('.', 1, mod(frame_i, 4))]); 

        for lane_i = LaneParams.lane_ids
            plot([LaneParams.leftBound, LaneParams.rightBound], LaneParams.lane_width.*lane_i.*[1, 1], '--', 'color', [0.75 0.75 0.75], 'LineWidth', 3);hold on;
            if lane_i == 0                
                plot([LaneParams.leftBound, LaneParams.rampStartX], [0, 0], 'k-', 'LineWidth', 4);
            end
        end
        
        plot(RoadBound(1:2,1),RoadBound(1:2,2),'k-','LineWidth',4); 
        plot(RoadBound(3:end,1),RoadBound(3:end,2),'k-','LineWidth',4);
        for v_i = 1:num_vecs
            if isempty(vizHistory{frame_i, v_i})
                continue;
            end
            l_car   = params.vehiclesDims(1, v_i);
            w_car   = params.vehiclesDims(2, v_i);
            state   = vizHistory{frame_i, v_i};
            [x, y, v]   = deal(state(1),state(2),state(3));
            rectangles  = [ x-l_car/2, y+w_car/2; x-l_car/2, y-w_car/2;
                            x+l_car/2, y+w_car/2; x+l_car/2, y-w_car/2;
                            x+(l_car/2-1), y+w_car/2; x+(l_car/2-1), y-w_car/2];

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
        
        annotation('textbox', [0.133 0.8 0.09 0.12],...
            'String', {['t = ' num2str((frame_i-1)*dt, '%2.2f') ' s']},...
            'FontSize', 20, 'FontName', 'Arial',...
            'LineStyle', '-', 'LineWidth',2,...
            'EdgeColor', [0.85 0.85 0.85],...
            'BackgroundColor', [0.95  0.95 0.95],...
            'Color','k');
        
        frame       = getframe(fig);
        im          = frame2im(frame);
        [imind, cm] = rgb2ind(im,256);
        if frame_i == startFrame
            imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'loopcount', inf);
        else
            imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'writemode', 'append');
        end
        close(fig);
    end
    close(progressBar)
end