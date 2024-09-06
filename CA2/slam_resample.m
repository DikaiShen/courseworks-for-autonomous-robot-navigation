function [particles] = slam_resample(particles, init_weight)
	
	particles_count = size(particles, 2);
	weight_total = 0;
	
    for i = 1:particles_count
		weight_total = weight_total + particles(i).weight;
    end
   
    newset = particles;
	for i = 1:particles_count
    % Missing codes start here
        random_weight = weight_total*rand(1);
        total = weight_total;
        % Resamples particles based on their weights
        for j = 1:particles_count
            total = total - particles(j).weight;
            if total <= random_weight
                newset(i) = particles(j);
                break
            else
                continue
            end
        end
        % Afterwards, each new particle should be given the same init_weight
        newset(i).weight = init_weight;
        % Missing codes end here
     end
   particles = newset;
end