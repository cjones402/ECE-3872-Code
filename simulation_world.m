%% Simulation World
% Engineer: Tim Brothers
% Overview
%    This is a simulation for the Robotic Orchestra
% Design Name:   The Conductor
% File Name:     simulation_world.m
%
% Inputs: 
%		Tempo: controls the tempo of the song. This is a number between 0 and 1
%		Octive: The ocive of the song (normally 4)
%		fs_kHz: sampling rate in kHz.
%       tempo_resolution: this is a value between 0 and 1. 1 being full 
%           resolution. Be careful setting this too high. 1 will lock up
%           your computer.
%       time_offset: This is the delay to the start of the song. This is an
%           index to the time scale, so the actual delay amount is
%           determined by multiple factors
%
% History:       4 January 2020 File created
%
%-----------------------------------------------------

function [digital,Octive,fs_Hz] = simulation_world(OctiveIn,tempo_sIn)
    %% Clear the Variables
%     clc
%     clear
%     close all

    %% Set up the world
    fs_kHz = 5;

    % Set the tempo and the octive for the conductor
    Octive = OctiveIn;
    tempo_s = tempo_sIn;% Rn use 0.6 or lower!!! %This is the time in seconds for each note

    %% Adjust fs for target octive
    if(Octive>4)
        fs_ratio = 2^(Octive-4);
    else
        fs_ratio = 1; % added for testing
    end
    fs_kHz = fs_kHz*fs_ratio;

    %% Parameters for code simulation
    tempo_resolution = .05; %this is a value between 0 and 1. 1 being full resolution
    time_offset = 60;   % This is the delay to the start of the song.

    %% Create the song given the tempo
    [song_freq_Hz, song_duration_s] = conductor_simulation(tempo_s,Octive);

    % form up the cumulative durations
    cumulative_duration_s = zeros(1, length(song_duration_s));
    cumulative_duration_s(1) =  song_duration_s(1);
    for i=2:length(song_duration_s)
        cumulative_duration_s(i) = song_duration_s(i) + cumulative_duration_s(i-1);
    end

    total_duration_s = max(cumulative_duration_s);
    fs_Hz = fs_kHz*1000; %Convert kHz to Hz
    time_s = 0:(1/fs_Hz):total_duration_s;	%create a time vector

    % There is some rounding error happening sometimes, so we are going to make
    % the end value exact
    time_s(length(time_s)) = total_duration_s;


    %% Create the digital signal
    min_time_index = 1; %start the min time index at the start
    % loop through all the freqs and generate a sin wave for each freq
    for i=1:length(song_freq_Hz)
        max_time_index = find(time_s >= cumulative_duration_s(i),1); %find the times that correspond to this duration of time
        digital_note = sin(2*pi*song_freq_Hz(i)*time_s(min_time_index:max_time_index)); %create the actual note
        min_time_index = max_time_index + 1; %shift to the next time region.
        if(i == 1)
            digital = digital_note;
        else
            digital = [digital, digital_note];
        end

    end

    %% Create a signal to test the system
    %%Put two copies of the song together
    signal = [digital, digital];
    signal = circshift(signal, time_offset);


%     % Plot the Time Domain
%     plot(time_s,digital)
%     xlabel("time (s)")
%     ylabel("Amplitude")
%     title("Row Row Row Your Boat")
% 
%     % Plot the frequency
%     figure()
%     freq_digital = fft(digital);
%     freq_digital = freq_digital(1:floor(length(digital)/2));
%     freq_axis_kHz = linspace(0, fs_kHz/2, length(freq_digital));
%     plot(freq_axis_kHz, abs(freq_digital))
%     xlabel("Frequency (kHz)")
%     ylabel("Amplitude")
%     title("Spectrum of Row Row Row Your Boat")
end
% %% Create matched filters
% tic
% C = 16.3516 *(2^Octive);
% D = 18.35405*(2^Octive);
% E = 20.60172*(2^Octive);
% F = 21.82676*(2^Octive);
% G = 24.49971*(2^Octive);
% A = 27.5    *(2^Octive);
% B = 30.86771*(2^Octive);
% high_C = 32.70320*(2^Octive);
% 
% timeSig = 0:(1/fs_Hz):1;	%create a time vector
% numPeriods = 10; %------------------------------------------------------------------Parameter
% 
% timeSig = 0:(1/fs_Hz):numPeriods/C;
% Cfilter = fliplr(sin(C*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/D;
% Dfilter = fliplr(sin(D*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/E;
% Efilter = fliplr(sin(E*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/F;
% Ffilter = fliplr(sin(F*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/G;
% Gfilter = fliplr(sin(G*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/A;
% Afilter = fliplr(sin(A*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/B;
% Bfilter = fliplr(sin(B*timeSig*2*pi));
% timeSig = 0:(1/fs_Hz):numPeriods/high_C;
% high_Cfilter = fliplr(sin(high_C*timeSig*2*pi));
% %% Calculate frequency interference
% CC = zeros(8);
% CC(1,1) = max(conv(Afilter,fliplr(Afilter)));
% CC(1,2) = max(conv(Afilter,fliplr(Bfilter)));
% CC(1,3) = max(conv(Afilter,fliplr(Cfilter)));
% CC(1,4) = max(conv(Afilter,fliplr(Dfilter)));
% CC(1,5) = max(conv(Afilter,fliplr(Efilter)));
% CC(1,6) = max(conv(Afilter,fliplr(Ffilter)));
% CC(1,7) = max(conv(Afilter,fliplr(Gfilter)));
% CC(1,8) = max(conv(Afilter,fliplr(high_Cfilter)));
% figure();
% subplot(3,3,1)
% plot(CC(1,:))
% 
% 
% CC(2,1) = max(conv(Bfilter,fliplr(Afilter)));
% CC(2,2) = max(conv(Bfilter,fliplr(Bfilter)));
% CC(2,3) = max(conv(Bfilter,fliplr(Cfilter)));
% CC(2,4) = max(conv(Bfilter,fliplr(Dfilter)));
% CC(2,5) = max(conv(Bfilter,fliplr(Efilter)));
% CC(2,6) = max(conv(Bfilter,fliplr(Ffilter)));
% CC(2,7) = max(conv(Bfilter,fliplr(Gfilter)));
% CC(2,8) = max(conv(Bfilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,2)
% plot(CC(2,:))
% 
% 
% CC(3,1) = max(conv(Cfilter,fliplr(Afilter)));
% CC(3,2) = max(conv(Cfilter,fliplr(Bfilter)));
% CC(3,3) = max(conv(Cfilter,fliplr(Cfilter)));
% CC(3,4) = max(conv(Cfilter,fliplr(Dfilter)));
% CC(3,5) = max(conv(Cfilter,fliplr(Efilter)));
% CC(3,6) = max(conv(Cfilter,fliplr(Ffilter)));
% CC(3,7) = max(conv(Cfilter,fliplr(Gfilter)));
% CC(3,8) = max(conv(Cfilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,3)
% plot(CC(3,:))
% 
% 
% CC(4,1) = max(conv(Dfilter,fliplr(Afilter)));
% CC(4,2) = max(conv(Dfilter,fliplr(Bfilter)));
% CC(4,3) = max(conv(Dfilter,fliplr(Cfilter)));
% CC(4,4) = max(conv(Dfilter,fliplr(Dfilter)));
% CC(4,5) = max(conv(Dfilter,fliplr(Efilter)));
% CC(4,6) = max(conv(Dfilter,fliplr(Ffilter)));
% CC(4,7) = max(conv(Dfilter,fliplr(Gfilter)));
% CC(4,8) = max(conv(Dfilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,4)
% plot(CC(4,:))
% 
% 
% CC(5,1) = max(conv(Efilter,fliplr(Afilter)));
% CC(5,2) = max(conv(Efilter,fliplr(Bfilter)));
% CC(5,3) = max(conv(Efilter,fliplr(Cfilter)));
% CC(5,4) = max(conv(Efilter,fliplr(Dfilter)));
% CC(5,5) = max(conv(Efilter,fliplr(Efilter)));
% CC(5,6) = max(conv(Efilter,fliplr(Ffilter)));
% CC(5,7) = max(conv(Efilter,fliplr(Gfilter)));
% CC(5,8) = max(conv(Efilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,5)
% plot(CC(5,:))
% 
% CC(6,1) = max(conv(Ffilter,fliplr(Afilter)));
% CC(6,2) = max(conv(Ffilter,fliplr(Bfilter)));
% CC(6,3) = max(conv(Ffilter,fliplr(Cfilter)));
% CC(6,4) = max(conv(Ffilter,fliplr(Dfilter)));
% CC(6,5) = max(conv(Ffilter,fliplr(Efilter)));
% CC(6,6) = max(conv(Ffilter,fliplr(Ffilter)));
% CC(6,7) = max(conv(Ffilter,fliplr(Gfilter)));
% CC(6,8) = max(conv(Ffilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,6)
% plot(CC(6,:))
% 
% 
% CC(7,1) = max(conv(Gfilter,fliplr(Afilter)));
% CC(7,2) = max(conv(Gfilter,fliplr(Bfilter)));
% CC(7,3) = max(conv(Gfilter,fliplr(Cfilter)));
% CC(7,4) = max(conv(Gfilter,fliplr(Dfilter)));
% CC(7,5) = max(conv(Gfilter,fliplr(Efilter)));
% CC(7,6) = max(conv(Gfilter,fliplr(Ffilter)));
% CC(7,7) = max(conv(Gfilter,fliplr(Gfilter)));
% CC(7,8) = max(conv(Gfilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,7)
% plot(CC(7,:))
% 
% CC(8,1) = max(conv(high_Cfilter,fliplr(Afilter)));
% CC(8,2) = max(conv(high_Cfilter,fliplr(Bfilter)));
% CC(8,3) = max(conv(high_Cfilter,fliplr(Cfilter)));
% CC(8,4) = max(conv(high_Cfilter,fliplr(Dfilter)));
% CC(8,5) = max(conv(high_Cfilter,fliplr(Efilter)));
% CC(8,6) = max(conv(high_Cfilter,fliplr(Ffilter)));
% CC(8,7) = max(conv(high_Cfilter,fliplr(Gfilter)));
% CC(8,8) = max(conv(high_Cfilter,fliplr(high_Cfilter)));
% %figure();
% subplot(3,3,8)
% plot(CC(8,:))
% 
% InterdependencyMatrix = linsolve(CC,diag(ones(8,1)));
% 
% %% Do the envelope detection
% Cir = conv(digital,Cfilter);
% Dir = conv(digital,Dfilter);
% Eir = conv(digital,Efilter);
% Fir = conv(digital,Ffilter);
% Gir = conv(digital,Gfilter);
% Air = conv(digital,Afilter);
% Bir = conv(digital,Bfilter);
% high_Cir = conv(digital,high_Cfilter);
% 
% %% Filter interdependencies
% smoothLen = 200;%ceil(100*fs_ratio);
% trimLen = length(high_Cir);
% Air2 = conv(abs(Air(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Bir2 = conv(abs(Bir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Cir2 = conv(abs(Cir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Dir2 = conv(abs(Dir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Eir2 = conv(abs(Eir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Fir2 = conv(abs(Fir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% Gir2 = conv(abs(Gir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% high_Cir2 = conv(abs(high_Cir(1:trimLen)),ones(smoothLen,1)*1/smoothLen);
% 
% figure();
% rowNum = 1;
% Air3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Air3)
% rowNum = 2;
% Bir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Bir3)
% rowNum = 3;
% Cir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Cir3)
% rowNum = 4;
% Dir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Dir3)
% rowNum = 5;
% Eir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Eir3)
% rowNum = 6;
% Fir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Fir3)
% rowNum = 7;
% Gir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(Gir3)
% rowNum = 8;
% high_Cir3 = InterdependencyMatrix(rowNum,1)*Air2+InterdependencyMatrix(rowNum,2)*Bir2+InterdependencyMatrix(rowNum,3)*Cir2+InterdependencyMatrix(rowNum,4)*Dir2+InterdependencyMatrix(rowNum,5)*Eir2+InterdependencyMatrix(rowNum,6)*Fir2+InterdependencyMatrix(rowNum,7)*Gir2+InterdependencyMatrix(rowNum,8)*high_Cir2;
% subplot(3,3,rowNum)
% plot(high_Cir3)
% 
% subplot(3,3,9)
% plot(Air3+Bir3+Cir3+Dir3+Eir3+Fir3+Gir3+high_Cir3)
% 
% figure()
% subplot(3,3,3)
% plot((Cir3>0.5*max(Cir3)))
% subplot(3,3,4)
% plot((Dir3>0.5*max(Dir3)))
% subplot(3,3,5)
% plot((Eir3>0.5*max(Eir3)))
% subplot(3,3,6)
% plot((Fir3>0.5*max(Fir3)))
% subplot(3,3,7)
% plot((Gir3>0.5*max(Gir3)))
% subplot(3,3,8)
% plot((high_Cir3>0.5*max(high_Cir3)))
% subplot(3,3,9)
% pulses = (Cir3>0.5*max(Cir3))+(Dir3>0.5*max(Dir3))+(Eir3>0.5*max(Eir3))+(Fir3>0.5*max(Fir3))+(Gir3>0.5*max(Gir3))+(high_Cir3>0.5*max(high_Cir3));
% plot(pulses)
% 
% %% threshold the impulse responses
% IMPthresh = 1; %------------------------------------------------------------------Parameter
% 
% Air(abs(Air)<IMPthresh) = 0;
% Bir(abs(Bir)<IMPthresh) = 0;
% Cir(abs(Cir)<IMPthresh) = 0;
% Dir(abs(Dir)<IMPthresh) = 0;
% Eir(abs(Eir)<IMPthresh) = 0;
% Fir(abs(Fir)<IMPthresh) = 0;
% Gir(abs(Gir)<IMPthresh) = 0;
% high_Cir(abs(high_Cir)<IMPthresh) = 0;
% 
% %% plot note charts
% maxlength = length(Cir); %------------------------------------------------------------------Parameter
% 
% figure()
% subplot(3,3,1)
% plot(Air)
% subplot(3,3,2)
% plot(Bir)
% subplot(3,3,3)
% plot(Cir)
% subplot(3,3,4)
% plot(Dir)
% subplot(3,3,5)
% plot(Eir)
% subplot(3,3,6)
% plot(Fir)
% subplot(3,3,7)
% plot(Gir)
% subplot(3,3,8)
% plot(high_Cir)
% subplot(3,3,9)
% tot_ir = padarray(Air,[0 maxlength-length(Air)],'post')+padarray(Bir,[0 maxlength-length(Bir)],'post')+padarray(Cir,[0 maxlength-length(Cir)],'post')+padarray(Dir,[0 maxlength-length(Dir)],'post')+padarray(Eir,[0 maxlength-length(Eir)],'post')+padarray(Fir,[0 maxlength-length(Fir)],'post')+padarray(Gir,[0 maxlength-length(Gir)],'post')+padarray(high_Cir,[0 maxlength-length(high_Cir)],'post');
% plot(tot_ir)
% 
% rest_indicator = conv(abs(tot_ir),[.1 .1 .1 .1 .1 .1 .1 .1 .1 .1]); %------------------------------------------------------------------Parameter
% rest_indicator(rest_indicator>1)=1;
% rest_indicator = conv(pulses,[-1,1]); %------------------------------------------------------------------Parameter
% 
% figure()
% plot(rest_indicator)
% ylim([-1,2]);
% 
% 
% %% Find the Tempo
% samples = 15;
% upPeak = 1;
% tempo = zeros(1,samples);
% for i = 1:samples
%     upPeak = upPeak + find(rest_indicator(upPeak:end)==1,1);
%     tempo(i) = find(rest_indicator(upPeak:end)==-1,1)/fs_Hz;
% end
% disp("tempo guesses: ")
% disp(tempo)
% disp("ave tempo guess: ")
% disp(mean(tempo))
% 
% %% Sync the time
% 
% actual_time_offset_s = time_s(time_offset) %print out the actual start time to compare.
% toc