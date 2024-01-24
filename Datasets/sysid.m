clear all;
close all;
clc;

%% Load the dataset
raw_data = readtable('4_sinusoids_1.csv');

% Split into vectors
t = raw_data.(1)./10e6;
voltage = raw_data.(2);
speed = raw_data.(3);
angle = raw_data.(4);

% Time step
Ts = 10e-3;     % NB: this is not exact: it is a bit more, but unpredictable

%% Plot to be sure it is correct
figure; hold on;
plot(t, voltage);
plot(t, speed);
plot(t, angle);
%legend("Voltage", "Speed", "Position")

%% Create the data object and preprocess it
u = voltage;
y = angle;

data = iddata(y, u, Ts);

% Detrend
mu = getTrend(data,0)
data_detrended = detrend(data, mu);

% Delay estimation
nk = delayest(data_detrended)

%% Kernel based PEM

% Practical lengths
nA = 50;
nB = 50;
orders_arxReg = [nA, nB, nk];

% Parametric empirical Bayes method 
option = arxRegulOptions('RegulKernel', 'DI'); % other types: 'TC', 'SS'
[Lambda, R] = arxRegul(data_detrended, orders_arxReg, option);
opt = arxOptions;
opt.Regularization.Lambda = Lambda;
opt.Regularization.R = R; 

% Kernel-based estimation 
m_arxReg = arx(data_detrended, orders_arxReg, opt);

figure; bodeplot(m_arxReg)

%% ARX model 
% We approximate the motor to a simple DC motor: Omega/V is a 2nd order tf
% with no zeros
nA = 3;
nB = 1;
orders_arx = [nA, nB, nk];

m_arx = arx(data_detrended, orders_arx);

bodeplot(m_arx)

%% OE model 
% We approximate the motor to a simple DC motor: Omega/V is a 2nd order tf
% with no zeros
nF = 2;
nB = 1;
orders_oe = [nB, nF, nk];

m_oe = arx(data_detrended, orders_oe);

bodeplot(m_oe)

%% Validate on the other datasets

% Load
validation_raw_data = readtable('4_sinusoids_2.csv');
voltage = validation_raw_data.(2);
speed = validation_raw_data.(3);
angle = validation_raw_data.(4);
% iddata object and detrend
u = voltage;
y = angle;
validation_data = iddata(y, u, Ts);
validation_data_detrended = detrend( validation_data, getTrend(validation_data) );

% Compare prediction with real output
m_to_evaluate = m_arx;
k = 1;
opt = compareOptions('InitialCondition','z');
figure; compare(validation_data_detrended, m_to_evaluate, k, opt)

%% Best model
m_best = m_arx;
bodeplot(m_arx);

sysMotor = tf(m_arx);
step(sysMotor);











