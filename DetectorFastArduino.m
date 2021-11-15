function [tempo] = DetectorFastArduino(digital,Octive,fs_Hz) %#codegen
    %% Create matched filters
    tic
    factor = 2^Octive;
    C = 16.3516 *factor;
    D = 18.35405*factor;
    E = 20.60172*factor;
    F = 21.82676*factor;
    G = 24.49971*factor;
    %A = 27.5    *factor;
    %B = 30.86771*factor;
    high_C = 32.70320*factor;

    %timeSig = 0:(1/fs_Hz):1;	%create a time vector
    numPeriods = 10; %------------------------------------------------------------------Parameter
    
    factor = 1/fs_Hz;
    timeSig = 0:factor:numPeriods/C;
    Cfilter = fliplr(sin(C*timeSig*2*pi));
    timeSig = 0:factor:numPeriods/D;
    Dfilter = fliplr(sin(D*timeSig*2*pi));
    timeSig = 0:factor:numPeriods/E;
    Efilter = fliplr(sin(E*timeSig*2*pi));
    timeSig = 0:factor:numPeriods/F;
    Ffilter = fliplr(sin(F*timeSig*2*pi));
    timeSig = 0:factor:numPeriods/G;
    Gfilter = fliplr(sin(G*timeSig*2*pi));
%     timeSig = 0:factor:numPeriods/A;
%     Afilter = fliplr(sin(A*timeSig*2*pi));
%     timeSig = 0:factor:numPeriods/B;
%     Bfilter = fliplr(sin(B*timeSig*2*pi));
    timeSig = 0:factor:numPeriods/high_C;
    high_Cfilter = fliplr(sin(high_C*timeSig*2*pi));
    %% Calculate frequency interference
    CC = zeros(6);
%     CC(1,1) = max(conv(Afilter,fliplr(Afilter)));
%     CC(1,2) = max(conv(Afilter,fliplr(Bfilter)));
%     CC(1,3) = max(conv(Afilter,fliplr(Cfilter)));
%     CC(1,4) = max(conv(Afilter,fliplr(Dfilter)));
%     CC(1,5) = max(conv(Afilter,fliplr(Efilter)));
%     CC(1,6) = max(conv(Afilter,fliplr(Ffilter)));
%     CC(1,7) = max(conv(Afilter,fliplr(Gfilter)));
%     CC(1,8) = max(conv(Afilter,fliplr(high_Cfilter)));
% 
%     CC(2,1) = max(conv(Bfilter,fliplr(Afilter)));
%     CC(2,2) = max(conv(Bfilter,fliplr(Bfilter)));
%     CC(2,3) = max(conv(Bfilter,fliplr(Cfilter)));
%     CC(2,4) = max(conv(Bfilter,fliplr(Dfilter)));
%     CC(2,5) = max(conv(Bfilter,fliplr(Efilter)));
%     CC(2,6) = max(conv(Bfilter,fliplr(Ffilter)));
%     CC(2,7) = max(conv(Bfilter,fliplr(Gfilter)));
%     CC(2,8) = max(conv(Bfilter,fliplr(high_Cfilter)));

    %CC(3,1) = max(conv(Cfilter,fliplr(Afilter)));
    %CC(3,2) = max(conv(Cfilter,fliplr(Bfilter)));
    CC(1,1) = max(conv(Cfilter,fliplr(Cfilter)));
    CC(1,2) = max(conv(Cfilter,fliplr(Dfilter)));
    CC(1,3) = max(conv(Cfilter,fliplr(Efilter)));
    CC(1,4) = max(conv(Cfilter,fliplr(Ffilter)));
    CC(1,5) = max(conv(Cfilter,fliplr(Gfilter)));
    CC(1,6) = max(conv(Cfilter,fliplr(high_Cfilter)));

    %CC(4,1) = max(conv(Dfilter,fliplr(Afilter)));
    %CC(4,2) = max(conv(Dfilter,fliplr(Bfilter)));
    CC(2,1) = max(conv(Dfilter,fliplr(Cfilter)));
    CC(2,2) = max(conv(Dfilter,fliplr(Dfilter)));
    CC(2,3) = max(conv(Dfilter,fliplr(Efilter)));
    CC(2,4) = max(conv(Dfilter,fliplr(Ffilter)));
    CC(2,5) = max(conv(Dfilter,fliplr(Gfilter)));
    CC(2,6) = max(conv(Dfilter,fliplr(high_Cfilter)));

    %CC(5,1) = max(conv(Efilter,fliplr(Afilter)));
    %CC(5,2) = max(conv(Efilter,fliplr(Bfilter)));
    CC(3,1) = max(conv(Efilter,fliplr(Cfilter)));
    CC(3,2) = max(conv(Efilter,fliplr(Dfilter)));
    CC(3,3) = max(conv(Efilter,fliplr(Efilter)));
    CC(3,4) = max(conv(Efilter,fliplr(Ffilter)));
    CC(3,5) = max(conv(Efilter,fliplr(Gfilter)));
    CC(3,6) = max(conv(Efilter,fliplr(high_Cfilter)));

    %CC(6,1) = max(conv(Ffilter,fliplr(Afilter)));
    %CC(6,2) = max(conv(Ffilter,fliplr(Bfilter)));
    CC(4,1) = max(conv(Ffilter,fliplr(Cfilter)));
    CC(4,2) = max(conv(Ffilter,fliplr(Dfilter)));
    CC(4,3) = max(conv(Ffilter,fliplr(Efilter)));
    CC(4,4) = max(conv(Ffilter,fliplr(Ffilter)));
    CC(4,5) = max(conv(Ffilter,fliplr(Gfilter)));
    CC(4,6) = max(conv(Ffilter,fliplr(high_Cfilter)));

    %CC(7,1) = max(conv(Gfilter,fliplr(Afilter)));
    %CC(7,2) = max(conv(Gfilter,fliplr(Bfilter)));
    CC(5,1) = max(conv(Gfilter,fliplr(Cfilter)));
    CC(5,2) = max(conv(Gfilter,fliplr(Dfilter)));
    CC(5,3) = max(conv(Gfilter,fliplr(Efilter)));
    CC(5,4) = max(conv(Gfilter,fliplr(Ffilter)));
    CC(5,5) = max(conv(Gfilter,fliplr(Gfilter)));
    CC(5,6) = max(conv(Gfilter,fliplr(high_Cfilter)));

    %CC(8,1) = max(conv(high_Cfilter,fliplr(Afilter)));
    %CC(8,2) = max(conv(high_Cfilter,fliplr(Bfilter)));
    CC(6,1) = max(conv(high_Cfilter,fliplr(Cfilter)));
    CC(6,2) = max(conv(high_Cfilter,fliplr(Dfilter)));
    CC(6,3) = max(conv(high_Cfilter,fliplr(Efilter)));
    CC(6,4) = max(conv(high_Cfilter,fliplr(Ffilter)));
    CC(6,5) = max(conv(high_Cfilter,fliplr(Gfilter)));
    CC(6,6) = max(conv(high_Cfilter,fliplr(high_Cfilter)));
    
%     figure()
%     for i = 1:8
%         subplot(3,3,i)
%         plot(CC(i,:))
%     end

    InterdependencyMatrix = linsolve(CC,diag(ones(6,1)));

    %% Filter dignal for notes      %Do the envelope detection
    Cir = conv(digital,Cfilter);
    Dir = conv(digital,Dfilter);
    Eir = conv(digital,Efilter);
    Fir = conv(digital,Ffilter);
    Gir = conv(digital,Gfilter);
    %Air = conv(digital,Afilter);
    %Bir = conv(digital,Bfilter);
    high_Cir = conv(digital,high_Cfilter);

    %% Filter interdependencies
    smoothLen = 200;%ceil(100*fs_ratio);
    trimLen = length(high_Cir);
    %Air2 = conv(abs(Air(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    %Bir2 = conv(abs(Bir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    Cir2 = conv(abs(Cir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    Dir2 = conv(abs(Dir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    Eir2 = conv(abs(Eir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    Fir2 = conv(abs(Fir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    Gir2 = conv(abs(Gir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
    high_Cir2 = conv(abs(high_Cir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);

    %figure();
%     rowNum = 1;
%     Air3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
%     subplot(3,3,rowNum)
%     plot(Air3)
%     rowNum = 2;
%     Bir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
%     subplot(3,3,rowNum)
%     plot(Bir3)
    rowNum = 1;
    Cir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(Cir3)
    rowNum = 2;
    Dir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(Dir3)
    rowNum = 3;
    Eir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(Eir3)
    rowNum = 4;
    Fir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(Fir3)
    rowNum = 5;
    Gir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(Gir3)
    rowNum = 6;
    high_Cir3 = InterdependencyMatrix(rowNum,1)*Cir2+InterdependencyMatrix(rowNum,2)*Dir2+InterdependencyMatrix(rowNum,3)*Eir2+InterdependencyMatrix(rowNum,4)*Fir2+InterdependencyMatrix(rowNum,5)*Gir2+InterdependencyMatrix(rowNum,6)*high_Cir2;
    %subplot(3,3,rowNum)
    %plot(high_Cir3)

    %subplot(3,3,9)
    %plot(Cir3+Dir3+Eir3+Fir3+Gir3+high_Cir3)

%     figure()
%     subplot(3,3,3)
%     plot((Cir3>0.5*max(Cir3)))
%     subplot(3,3,4)
%     plot((Dir3>0.5*max(Dir3)))
%     subplot(3,3,5)
%     plot((Eir3>0.5*max(Eir3)))
%     subplot(3,3,6)
%     plot((Fir3>0.5*max(Fir3)))
%     subplot(3,3,7)
%     plot((Gir3>0.5*max(Gir3)))
%     subplot(3,3,8)
%     plot((high_Cir3>0.5*max(high_Cir3)))
%     subplot(3,3,9)
    pulses = (Cir3>0.5*max(Cir3))+(Dir3>0.5*max(Dir3))+(Eir3>0.5*max(Eir3))+(Fir3>0.5*max(Fir3))+(Gir3>0.5*max(Gir3))+(high_Cir3>0.5*max(high_Cir3));
%     plot(pulses)
%     ylim([-1,2]);
    

    %% threshold the impulse responses
%     IMPthresh = 1; %------------------------------------------------------------------Parameter
% 
%     Air(abs(Air)<IMPthresh) = 0;
%     Bir(abs(Bir)<IMPthresh) = 0;
%     Cir(abs(Cir)<IMPthresh) = 0;
%     Dir(abs(Dir)<IMPthresh) = 0;
%     Eir(abs(Eir)<IMPthresh) = 0;
%     Fir(abs(Fir)<IMPthresh) = 0;
%     Gir(abs(Gir)<IMPthresh) = 0;
%     high_Cir(abs(high_Cir)<IMPthresh) = 0;

    %% plot note charts
%     maxlength = length(Cir); %------------------------------------------------------------------Parameter
% 
%     figure()
% %     subplot(3,3,1)
% %     plot(Air)
% %     subplot(3,3,2)
% %     plot(Bir)
%     subplot(3,3,3)
%     plot(Cir)
%     subplot(3,3,4)
%     plot(Dir)
%     subplot(3,3,5)
%     plot(Eir)
%     subplot(3,3,6)
%     plot(Fir)
%     subplot(3,3,7)
%     plot(Gir)
%     subplot(3,3,8)
%     plot(high_Cir)
%     subplot(3,3,9)
%     tot_ir = padarray(Cir,[0 maxlength-length(Cir)],'post')+padarray(Dir,[0 maxlength-length(Dir)],'post')+padarray(Eir,[0 maxlength-length(Eir)],'post')+padarray(Fir,[0 maxlength-length(Fir)],'post')+padarray(Gir,[0 maxlength-length(Gir)],'post')+padarray(high_Cir,[0 maxlength-length(high_Cir)],'post');
%     plot(tot_ir)

    %rest_indicator = conv(abs(tot_ir),[.1 .1 .1 .1 .1 .1 .1 .1 .1 .1]); %------------------------------------------------------------------Parameter
    %rest_indicator(rest_indicator>1)=1;
    rest_indicator = conv(pulses,[-1,1]); %------------------------------------------------------------------Parameter

    % filter out HF noise
    locs = find(rest_indicator);
    %truelocs = zeros(1,length(locs));
    %truelocs(1) = locs(1);
    j = 2;
    for i = 2:(length(locs))
        if(locs(i)-locs(i-1)>100)
            locs(j) = locs(i);
            j = j+1;
        end
    end
    locs = locs(locs>0);
    ri = zeros(length(rest_indicator),1);
    ri(locs) = rest_indicator(locs);
    
%     figure()
%     plot(rest_indicator)
%     ylim([-1,2]);
    
%     figure()
%     plot(ri)
%     ylim([-1,2]);
    
    rest_indicator = ri;


    %% Find the Tempo
    samples = 30;
    peakHistory = zeros(1,samples);
    upPeak = 1;
    tempo = zeros(1,samples);
    i = 1;
    go = 1;
    while(go)
        upPeak2 = upPeak + find(rest_indicator(upPeak:end)==1,1);
        peakHistory(i) = upPeak2-upPeak;
        upPeak=upPeak2;
        temp = find(rest_indicator(upPeak:end)==-1,1)/fs_Hz;
        if(not(isempty(temp)))
            tempo(i) = temp;
            i= i+1;
        else
            go = 0;
        end
    end
    peakHistory = peakHistory(peakHistory>0);
%     disp("peak History: ")
%     disp(peakHistory(2:end))
%     disp("tempo guesses: ")
%     disp(tempo(tempo>0.01))
    %disp("ave tempo guess: ")
    %disp(mean(tempo(tempo>0.01)))

    %% linear regression for slope estimate 
    
    % if Octive is less than 4 filter the peakHistory signal to reduce high
    % frequency noise from signal edge detection
    if(Octive<4)
%         ri_spikes = find(rest_indicator);
%         true_spikes = zeros(1,length(ri_spikes));
%         true_spikes(1) = ri_spikes(1);
%         thresh = 100;
%         j = 1;
%         for i = 2:length(ri_spikes)
%             if((ri_spikes(i)-ri_spikes(i-1))>thresh)
%                 j=j+1;
%                 true_spikes(j) = ri_spikes(i);
%             end
%         end    
        true_spikes = locs(1:2:end);
        spikeDifs = true_spikes(2:end)-true_spikes(1:end-1);
        spikeDifs = spikeDifs(spikeDifs>0);
        xs = round(spikeDifs/(mean(tempo(tempo>0.01))*fs_Hz));
        coefs = polyfit(xs(xs<5),spikeDifs(xs<5),1);
        %disp("tempo guess from slope: ")
        %disp(coefs(1)/fs_Hz)
        tempo = coefs(1)/fs_Hz;
    else
        xs = round(peakHistory(2:end)/(mean(tempo(tempo>0.01))*fs_Hz));
        coefs = polyfit(xs(1:end-1),peakHistory(2:end-1),1);
        %disp("tempo guess from slope: ")
        %disp(coefs(1)/fs_Hz)
        tempo = coefs(1)/fs_Hz;
    end
    %% Sync the time

    %actual_time_offset_s = time_s(time_offset) %print out the actual start time to compare.
    toc
end