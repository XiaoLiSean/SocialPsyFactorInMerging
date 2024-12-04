function generateFilteringHistory(filename)
    load(filename, 'highwaySim');
    num_vec = length(highwaySim.Vehicles);
    filterHistory   = highwaySim.Vehicles{highwaySim.ego_id}.filterHistory;

    if strcmp(highwaySim.ego_type, 'EGO') || strcmp(highwaySim.ego_type, 'EGOAtt')
        filterUpdated   = highwaySim.Vehicles{highwaySim.ego_id}.filterUpdated;
        for inferStep = 1:size(filterUpdated, 2)        
            for car_i = 1:num_vec
                if car_i == highwaySim.ego_id || isequal(highwaySim.Vehicles{car_i}.state, [-inf;0;0])
                    continue
                end
                fig     = figure('visible', 'off');
                set(gcf,'color','w');
                set(gcf,'Position',[0 0 500 700]);
                
                % Parameter Distributions
                likelihoods     = filterHistory{car_i, inferStep};
                likelihoods(1, 2:end)   = NaN; % altruistic don't care self reward 
                h           = heatmap(likelihoods');
                if inferStep == 1
                    h.Title     = sprintf('Likelihood: car %d (initial)', car_i);
                else
                    h.Title     = sprintf('Likelihood: car %d with %d sec learning seg.', car_i, inferStep-1);
                end
                h.XLabel    = 'SVO';
                h.YLabel    = sprintf('Weight Case i of %d', size(likelihoods, 2));
                YDisplayLabels          = h.YDisplayLabels;
                rmIdx                   = setdiff(1:size(likelihoods, 2), round(linspace(1, size(likelihoods, 2), 10)));
                YDisplayLabels(rmIdx)   = {''};
                h.YDisplayLabels        = YDisplayLabels;
                h.XDisplayLabels        = {'altruistic', 'prosocial', 'egoistic', 'competitive'};           
                saveas(fig, sprintf('./simulationHistory/filter/car_%d_%d_sec', car_i, inferStep-1), 'png');
                close(fig)
            end
        end
    elseif strcmp(highwaySim.ego_type, 'EGOLF')
        for car_i = 1:num_vec
            if car_i == highwaySim.ego_id || isequal(highwaySim.Vehicles{car_i}.state, [-inf;0;0])
                continue
            end
            fig     = figure('visible', 'off');
            set(gcf,'color','w');
            set(gcf,'Position',[0 0 500 200]);
            likelihoods     = filterHistory{car_i};
            plot(0:(size(likelihoods, 2)-1), likelihoods(1,:), 'r', 'LineWidth', 2); grid on;
            xlabel('$t$ [sec]', 'Interpreter', 'latex'); 
            ylabel('$P_{leader}$', 'Interpreter', 'latex'); ylim([0,1]);
            set(gca, 'FontSize', 12);
            saveas(fig, sprintf('./simulationHistory/filter_lf/car_%d_leader', car_i), 'png');
            close(fig)
        end
    end
end