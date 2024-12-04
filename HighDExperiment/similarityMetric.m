function likelihood = similarityMetric(actualTraj, predTraj, params)
    likelihood 	= 1;
    scale       = 4e3; % avoid zero likelihood
    lane_width  = params.LaneParams.lane_width;
    car_l_avg   = mean(params.vehiclesDims(1,:));
    Q           = diag([2*car_l_avg, 0.25*lane_width, 3].^2);
    for i = 1:size(predTraj,2)
        x       = predTraj(:,i);
        x_hat   = actualTraj(:,i);
        p       = scale*mvnpdf(x-x_hat, zeros(length(x), 1), Q);
        likelihood  = likelihood*p;
    end
end