figure()
error = zeros(1,8);
for o = 1:8
    subplot(3,3,o)
    for t = 0.1:0.05:0.5
        [digital,Octive,fs_Hz] = simulation_world(o,t);
        noise = 2*rand(1,length(digital));
        digital = digital+noise;
        [tempo, peakHistory,xs,rest_indicator] = DetectorFast(digital,Octive,fs_Hz);
        error(round((t-0.1)*20+1)) = abs(t-tempo)/t*100;
    end
    plot(error)
    ylabel('Percent error on tempo prediction');
    xlabel('duration of each beat in seconds');
end