% for tracking IMU drift 
data = readtable('15X10X40.CSV');
%%
%start = 700;
%ennd = 1050;
start = 1;
ennd = size(data,1);

acceleration_data = table2array(data(start:ennd, 4:6));  % Assuming columns 1, 2, and 3 correspond to X, Y, and Z axes
time = table2array( data(start:ennd,1))./1000  %-  table2array(data(start, 1))/1000;

acceleration_data(:,4) = acceleration_data(:,3) - mean(acceleration_data(:,3));
gyro_data = table2array(data(start:ennd, 7:9));
compass_data = table2array(data(start:ennd, 10:12));


figure(2); clf
subplot(3,1,1)
plot(time, acceleration_data);
title('IMU Acceleration Over Time');
xlabel('Time (seconds)');
ylabel('Acceleration (m/s^2)');
legend('X-axis', 'Y-axis', 'Z-axis');
grid on;

subplot(3,1,2)
plot(time, gyro_data);
ylabel('gryo degrees/s');
legend('x', 'y', 'z')

subplot(3,1,3)
plot(time, compass_data);
ylabel('compass degrees')
legend('x' , 'y', 'z')

%%
find_nearest = @(array, value) array(abs(array - value) == min(abs(array - value)));
%make sure accelerometer is taking burst samples 0.25 second sampling rate over a 60 second burst
% make sure sampling is evenly spaced -- good clock
%make sure data is telemetered
%fourier analysis:
%   x-axis: frequency
%   y-axis: velocity squared / frequency

% use fast fourier transform (fft) 
% need timeseries data, bins of frequency bands (1hz - 0.05hz for ocean waves)
% nyquist frequency of 1hz,
% minimum frequency of 2hz or more at inner bound
% minimum frequency of 0.05 hz at lower outer band

sampling_rate = mean(diff(time));
%Drewtime = ([Drew.Datenum] - Drew.Datenum(1));
%Dsampling = mean(diff(Drewtime));
dt = sampling_rate/1000;
%Dt =  0.025;%Dsampling/1000000;
nbands = 5;
alpha = 0.8;
%%
figure(3); clf

%[DSx,DSu,DSl,Df,Dfsel] = autospectrum(dataTable.RawX, Dt, nbands, alpha);   %
[Sx,Su,Sl,f,fsel] = autospectrum(acceleration_data(:,1), dt, nbands, alpha);
loglog(f,Sx)
%hold on;
%loglog(Df,DSx)
xlabel('Frequency (hz)','FontSize',14)
ylabel('Acceleration Spectrum (m^2/s^3)','Fontsize',14)
grid on;
hold on
%[DSy,DSu,DSl,Df,Dfsel] = autospectrum(dataTable.RawY, Dt, nbands, alpha);   %
[Sy,Su,Sl,f,fsel] = autospectrum(acceleration_data(:,2), dt, nbands, alpha);
loglog(f,Sy)
hold on;
%loglog(Df,DSy)
%hold on

%[DSz,DSu,DSl,Df,Dfsel] = autospectrum(dataTable.RawZ, Dt, nbands, alpha);   %
[Sz,Su,Sl,f,fsel] = autospectrum(acceleration_data(:,3), dt, nbands, alpha);
loglog(f,Sz)
%hold on;
%loglog(Df,DSz)

legend('Acc_X', 'Drew_RawX' , 'Acc_Y', 'Drew_RawY', 'Acc_Z','Drew_RawZ', 'location', 'best');

%%
figure(7)
dt = mean(diff(time));
fs = 1/dt*1000;
%m = 1:(length(time)-1)/2 ; %
m = 'octave';
[avar, tau] = allanvar(acceleration_data,m, fs) ;
adev = sqrt(avar);
loglog(tau, adev)
xlabel('\tau (s)')
ylabel('Deviation (g/s)')  %('\sigma^2 (\tau)')
title('Allan Deviation')
legend('X', 'Y', 'Z', 'location','best');
grid on

disp('Allan Deviation')
dev = find_nearest(tau, 1);
disp(['Deviation:, ' num2str(adev(tau == dev)) ' (g/s)']);

%%
decim = 2;
Fs = dt;
fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);

q = fuse(acceleration_data(:,1:3),gyro_data);

time = (0:decim:size(acceleration_data,1)-1)/Fs;
figure(4);clf
plot(time,eulerd(q,'ZYX','frame'))
title('Orientation Estimate')
legend('Z-axis', 'Y-axis', 'X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')

%%
% compute the variance of the spectra
% Define filter specifications
Fs = dt; % Sampling frequency (Hz)
Fc = 0.2; % Cut-off frequency (Hz)
N = 5; % Filter order

% Design lowpass Butterworth filter
[b, a] = butter(N, Fc, 'low');

% Generate sample signal (replace with your actual signal)
t = time./1000; % Time vector
aX = acceleration_data(:,1);
aY = acceleration_data(:,2);

% Apply the filter to the signal
filtered_X = filter(b, a, aX);
filtered_Y = filter(b, a, aY);

% Plot original and filtered signals
figure(4); clf
subplot(3,1,1);
plot(t, aX);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3,1,2);
plot(t, filtered_X);
hold on 
plot(t, filtered_Y);
hold on
plot(t, -acceleration_data(:,4))
title('Filtered Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3,1,3)
[Sx,Su,Sl,f,fsel] = autospectrum(filtered_X, dt, 1, 0.1);
loglog(f,Sx)
hold on
[Sx,Su,Sl,f,fsel] = autospectrum(filtered_Y, dt, 1, 0.1);
loglog(f,Sx)
hold on
[Sx,Su,Sl,f,fsel] = autospectrum(acceleration_data(:,4), dt, 1, 0.1);
loglog(f,Sx)

legend('X', 'Y', 'Z')

%%
% apply lowpass filter based on X peak

fpass = findpeaks(Sx);
fs = dt;
x = acceleration_data(:,1);
Xfilt = lowpass(x, fpass,fs);


%%
% gps

% needs to aqcuire data in bursts over a certain frequency 
function [S,Su,Sl,f,fsel] = autospectrum(x,dt,nbands,alpha)
%Computes the autospectrum of input time series x
%Input:
%x: time series
%dt: sample period
%nbands: number of frequency bands to smooth
%alpha: (1-alpha)*100 confidence intervals
%Output:
%S: autospectrum
%Su: 95% confidence interval upper bound
%Sl: 95% confidence interval lower bound
%f: frequency, units = cycles/dt
%ensure that length of x is an even number
N = length(x);
if(mod(N,2)~=0)
N = N-1;
x = x(1:N);
end
%Fourier transform
X = fft(x);
%one-sided frequencies, ignore the mean
f = (1:N/2)'/(N*dt);
fsel = find(abs(f)>1/000);
%one-sided spectral density
S = (abs(X(2:N/2+1)).^2)*dt/N;
%double amplitudes except Nyquist
S(1:end-1) = 2*S(1:end-1);
%smooth with a running mean
%w = ones(nbands,1)/nbands;
%S = conv(S,w,'valid');
%f = conv(f,w,'valid');
%confident intervals
nu = 2*nbands;
Su = S*nu/chi2inv(alpha/2,nu);
Sl = S*nu/chi2inv(1-alpha/2,nu);

%inverse fft
%I = ifft(x)


end
