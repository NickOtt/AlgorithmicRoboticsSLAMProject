function noisy_scan = make_scan_noisy(scan,scan_noise_param)
    for i=1:length(scan)
        noisy_scan(i) = normrnd(scan(i),scan_noise_param);
    end
end

