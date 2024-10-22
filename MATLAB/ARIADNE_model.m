%%%%%%%%%% ARIADNE LPM SIMULATION %%%%%%%%%%%%%%
% 
% This code implements a lumped parameter model 
% of an ARIADNE actuator on the skin, and compares
% the model output to sample experimental data.
%
% Dependent functions: double_SDS
% 
% Author: Nataliya Rokhmanova
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. Load sample data
% SUBJECT: 3
% LOC: lat
% AMP: 100%
%load('dataExp.mat')

%% 2. Set global and tuned subject-specific parameters

bu = 0.01085;               % Nsec/m (suspension damping)
ku = 1004;                  % N/m (suspension stiffness) 
mm = 0.00143;               % kg (magnet mass) 
mc = 0.00353;               % kg (case mass) 

ks = 2370;                  % N/m (skin stiffness)
bs = 0.81;                  % Nsec/m (skin damping)

A = 0.000897 * signal_amp;  % N (applied force magnitude)

%% 3. Run LPM 

% a. create Ft
Fs = 26667;                     % samples per second (27 kHz)
dt = 1/Fs;                      % seconds per sample

t = (0:dt:(length(dataExp)-1)/Fs);
t1 = (0:dt:0.15-dt)';           % seconds (0.15s sec of steady state)
t2 = (0.15:dt:0.25-dt)';        % seconds (0.10 sec of vibration)
t3 = (0.25:dt:t(end))';         % seconds (remaining time: settling)

Ft = [t1; t2; t3];

% b. create Fa
% i. find exact frequency from 10 largest signal peaks (should be almost exactly 177Hz)
[pks, locs] = findpeaks(dataExp, 'MinPeakProminence', 1.5, 'SortStr', 'descend', 'NPeaks', 10);
Ff = 1/mean(diff(t(sort(locs))));

% ii. put together Fa
F1 = 0.*t1;                     % amplitude (steady state)
F2 = -A*sin(Ff*t2*2*pi-0.675);    % amplitude (vibration), offset to smooth sine start
F3 = 0.*t3;                     % amplitude (settling)
Fa = [F1; F2; F3];

% c. set up ode45

sim_t = [0 Ft(end)];            % seconds (simulation runtime)
ICs = [0; 0; 0; 0];             % Initial conditions: m, m, m/s, m/s (y_a, y_c, y_a', y_c')

[t,y] = ode45(@(t,y) double_SDS(t, y, Ft, Fa, bu, ku, mm, bs, ks, mc), sim_t, ICs);

%% 4. Align model output to experimental data

% a. interpolate y output to match Ft length
y_interp = interp1(t, y, Ft);

% b. compute case acceleration using EoMs
yc_acc = (1/mc).*(Fa + bu.*y_interp(:,3) - (bu+bs).*y_interp(:,4)  + ku.*y_interp(:,1) - (ku+ks).*y_interp(:,2) );

% c. align signals
[pks,locs] = (findpeaks(dataExp, 'MinPeakHeight', max(dataExp)/4));
pks_ind = abs(diff(pks(round(length(pks)/2):end)))>0.05; % take back half
pks_ind = [false; pks_ind];
locs_ind = locs(round(length(locs)/2):end);
% find first pks_ind=1
first_pks_ind = locs_ind(pks_ind);
first_pks_ind = first_pks_ind(1);
    
[pks,locs] = (findpeaks(yc_acc, 'MinPeakHeight', max(yc_acc)/4));
pks_ind_exp = abs(diff(pks(round(length(pks)/2):end)))>0.05; % take back half
pks_ind_exp = [false; pks_ind_exp];
locs_ind_exp = locs(round(length(locs)/2):end);
% find first pks_ind=1
first_pks_ind_exp = locs_ind_exp(pks_ind_exp);
first_pks_ind_exp = first_pks_ind_exp(1);
    
% align difference:
offset = first_pks_ind - first_pks_ind_exp;
    
if offset >=0
    yc_acc_t = [zeros(offset,1); yc_acc];
    dataExp_t = dataExp;
else
    dataExp_t = [zeros(offset,1); dataExp];
    yc_acc_t = yc_acc;
end

%
yc_acc_t = yc_acc_t(1:min(length(yc_acc_t), length(Ft)));
dataExp_t = dataExp_t(1:min(length(yc_acc_t), length(Ft)));

% d. plot
figure; 
sgtitle('Subject 3, Lateral, 100% Amplitude')
subplot(2,1,1)
% position of model
plot(t,y(:,1)); hold on; 
plot(t,y(:,2)); 
title('Model: position of two masses')
legend('Mc','Mm')
xlabel('Time (s)')
ylabel('Position (m)')

subplot(2,1,2)
% acc model vs. real
plot(Ft, dataExp_t); hold on;
plot(Ft, yc_acc_t)
legend('Experimental', 'Model')
title('Experimental Data vs. Model Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (g)')


%% 5. Plot frequency response

s = tf('s');
G = mm*s^4 / ( (mm*s^2 + bu*s + ku) * (mc*s^2 + (bu+bs)*s + ks + ku) - (bu*s + ku)^2 );

bode(G)
hold on;
xline(177*2*pi)