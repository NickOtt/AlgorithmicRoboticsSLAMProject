function sample = sample_normal_distrib(variance)
    sum = 0;
    for i=1:12
        sum = sum + ((rand()*2*sqrt(variance))-sqrt(variance));
    end
    sample = 0.5*sum;
end

