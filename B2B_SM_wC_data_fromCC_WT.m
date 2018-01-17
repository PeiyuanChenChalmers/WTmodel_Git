%%
clear 
% close all
clc

% Sampling frequency, Hz
fs=5000/1;

% Sampling period, s
Ts=1/fs;

% Switching frequency, Hz
fsw=fs;

% fundamental frequency, Hz
f1=50; 

% % DC link voltage, V
% Vdc=200; 

% % PWM Reference voltage magntidue, V
% Vabc_ctrl=Vdc/2;

%Number of samples for offset 
Noffset=5000; 

%**********************************************************************
% m-file for VSC controller
%**********************************************************************

fn=f1; % Hz, rated frequency
wn=2*pi*fn; % rad/s, rated angular frequency
% Ts=2e-4; % s, simulation step
fpwm=1/Ts;% Hz, PWM frequency
tdb=10*0; % us, IGBT deadband

%**********************************************************************
% Per unit bases                                                      *
%**********************************************************************
S_base = 35000;          % W, 3-phase
% S_base_turbine= 2500;   % W, 3-phase rms
U_ac_base = 230*sqrt(2);% V, phase, peak
U_dc_base = U_ac_base*2; % V   
omega_base = 2*pi*fn; % rad/s
I_ac_base = 2*S_base / (3*U_ac_base); % A, phase, peak
Z_ac_base = U_ac_base / I_ac_base; % ohm
% 151.499V -> 60Vac line-to-line,rms

%**********************************************************************
% Reference values                                                    *
%**********************************************************************
% Pref = 50; % W
% Qref = 0; % VAr
 Vgrid = 0*sqrt(2); % V, peak phase value for sinusoidal grid voltage
 
%**********************************************************************
% Parameter definition                         
%**********************************************************************
% Rf_hat = 0.145; % ohm, measured
% Lf_hat = 0.94/omega_base; % H, measured 
% Rf_hat =0.1086; % ohm, measured
% Lf_hat = 2.1991/omega_base; % H, measured 
Rf_hat =6.2e-3; % ohm, measured
Lf_hat = 2e-3; % H, measured 

% per unit 
Rf_hat_pu = Rf_hat/Z_ac_base;
Lf_hat_pu = Lf_hat*omega_base/Z_ac_base;

% IGBT & Diode voltage drop and resistance
Rce = 28.8e-3;  % ohm, IGBT resistance cell + module level
Vce0 = 0.6831;  % V, IGBT no load voltage drop

Rt = 24.9e-3;  % ohm, diode resistance cell + module level
Vt0 = 0.6481;  % V, diode no load voltage drop

%**********************************************************************
% Control system                                                      *
%**********************************************************************
% % Ts = 1e-4;  % s                       Sampling period. If a smaller Ts is choosen (eg 100 times smaller)
% fs = 1/Ts;  % Hz                      the current and voltage reference values from simulation will better 
omega_s = 2*pi*fs; %  rad/s           match calc values from separate m-file
current_ac_lim = I_ac_base; % A, limit for test cases

% Current controller
alphac = omega_s/10; % 1/s, bandwidth
Ra = (alphac*Lf_hat-Rf_hat)*0.1; %  ohm, active damping                 
Kpc = alphac*Lf_hat; % parameter for proportional controller
Kic = alphac*(Rf_hat + Ra); % parameter for integral controller

% Current controller per unit
Ra_pu  = (alphac*Lf_hat_pu/omega_base-Rf_hat_pu)*0.1;
Kpc_pu = alphac*Lf_hat_pu/omega_base;
Kic_pu = alphac*(Rf_hat_pu + Ra_pu);
current_ac_lim_pu = current_ac_lim/I_ac_base;

% low pass filter in current ref calculation block
alpha_lp = alphac/10; % 1/s, bandwidth
alpha_ff=alphac/5;%bandwith for feedforward of V2

%**********************************************************************
% dc-link controller & Q ref controller                                                   
%**********************************************************************
alpha_dc= alphac/10;        % rad/s, bandwidth (2*pi*50)
C_dc=6080e-06;              % F, capacitance of the DC-link capacitor,6600e-06
Ca=alpha_dc*C_dc*0;         % rad/s*F, active damping term
Kp_dc=alpha_dc*C_dc;        % rad/s*F, Proportional gain
Ki_dc=alpha_dc*Ca*1+Kp_dc/0.3/10;          % (rad/s)^2*F, integral gain, active damping or a simple time constant of 0.3 sec/rad
Igmax=min(I_ac_base,55*sqrt(2));                   % A, maximum grid-side current, phase peak
alpha_Q=alphac/10;          % rad/s, bandwidth (2*pi*50)
%**********************************************************************
% Initial Conditions                                                  *
%**********************************************************************

initial_if_actual = 0;      % A.
pll_initial_phase=0;        % degrees.
pll_initial_freq=50;        % Hz.
Pref=0;            % W.
Qref=0;            % VAr.
I_control_real_init=0;      % Integrator real part, current controller.
I_control_imag_init=0;      % Integrator real part, current controller.

%% Synchronous machine parameter
%**********************************************************************
% Per unit bases - Stator, field and damper windings                                                      
%**********************************************************************
Sb = S_base;        % VA, stator 3-phase power
Vb = U_ac_base;   % 230*sqrt(2);   % V, stator AC phase, peak
wb = wn;            % rad/s, rotating frequency base
Ib = Sb/(3/2*Vb);   % A, stator AC phase, peak
Zb = Vb/Ib;         % ohm, stator impedance base
Lb=Zb/wb;           % H, stator inductance base
Psi_b=Vb/wb;        % Wb, stator flux base


Jtot = 2.0364;                  % kg.m^2, inertia of SM and IM rotor and shaft
btot = 0.073*1;                 % Nm/(rad/s), mechanical friction constant
pp=2;                           % number of pole pair
Htot= 0.5*Jtot*(wb/pp)^2/Sb;    % s, inertia constant
Tb=Sb/(wb/pp);                  % Nm, torque base
btot_pu=btot/(Tb/(wb/pp));      % p.u., mechanical friction constant

Nfs=7;              % Nf/Ns, field/stator winding turns ratio
Ifn=27;             % A, rated field current at field side(not the same as Ifb)
Vfn=50;             % V, rated field voltage at field side (not the same as Vfb)

Ifb=3/2/Nfs*Ib;     % A, (21.8704) field current base (equal power base and flux base)
Vfb=Sb/Ifb;         % V, (2.0576e+3) field voltage base
Zfb=Vfb/Ifb;        % ohm, (94.08) field impedance base
Lfb=Zfb/wb;         % H, (0.2995) field inductance base
Psif_b=Vfb/wb;       % Wb, stator flux base

%**********************************************************************
% Stator, field and damper winding parameters in ohm or H or A                                               
%**********************************************************************
Rs=60e-3;        % ohm, stator resistance
Lsl=0.48e-3;     % H, stator leakage inductance
Lmd=(7.4e-3)*1.4;  % H, (3/2*Vb/Ifn/wb/Nfs=,Lb*Ifb/Ifn) d-axis mutual inductance (flux linkage induced by rated field current through Lmd equals the one by base field current through Lb)  
Lmq=(4.1e-3)*1.4;     % H, q-axis mutual inductance
Lsd=Lsl+Lmd;     % H, d-axis stator self inductance   
Lsq=Lsl+Lmq;     % H, q-axis stator self inductance

Rf0=1.4;        % ohm, field resistance at field side
Rf=Rf0/Zfb*Zb;  % ohm, (42.9e-3)field resistance at stator side
Lfl=0.99e-3;    % H, field leakage inductance at stator side

Rkd=0.04;       % ohm, d-axis damper resistance at stator side
Lkdl=0.67e-3;   % H, d-axis damper leakage at stator side
Rkq=0.2;        % ohm, q-axis damper resistance at stator side
Lkql=4.62e-3;   % H, q-axis damper leakage at stator side

%**********************************************************************
% Stator, field and damper winding parameters in p.u.                                               
%**********************************************************************
Rs_pu=Rs/Zb;        % 0.0208
Lsl_pu=Lsl/Lb;      % 0.0524
Lmd_pu=Lmd/Lb;      % 0.8100
Lmq_pu=Lmq/Lb;      % 0.4472

Rf_pu=Rf/Zb;        % 0.0149
Lfl_pu=Lfl/Lb;      % 0.1080

Rkd_pu=Rkd/Zb;      % 0.0139
Lkdl_pu=Lkdl/Lb;    % 0.0731
Rkq_pu=Rkq/Zb;      % 0.0694
Lkql_pu=Lkql/Lb;    % 0.5040

%**********************************************************************
% Stator open-circuit test: steady state                                              
%**********************************************************************
Vf0= 20*Rf0;  %15.4;    % V, actual field voltage at field side
If0=Vf0/Rf0;            % A, (11) actual field current at field side
Ifs0=If0/Ifb*Ib;        % A, field current covertered to stator side
phi_r_pu=Lmd_pu*If0/Ifb;% pu, (0.4074) mutual flux in per unit 
Psi_r0=Lmd_pu*Lfb*If0;  % Wb, (2.66854) phase peak, mutual flux linkage at field side
Psi_r=Psi_r0/Nfs;       % Wb, (0.38122) phase peak, mutual flux linkage at stator side
Edq=wb*Psi_r;           % V, (119.75) phase peak, open circuit back emf @ rated speed at stator side 

%% Synchronous machine current controller
%**********************************************************************
% Current Control parameters                                                     
%**********************************************************************
% Ksb=0.1;                    % active damping coefficient
% alpha_SC= 2*pi*fs/50;       % rad/s, bandwidth (2*pi*500)
% Bsd=(Lsd*alpha_SC-Rs)*Ksb;        % ohm, active damping resistance, d-axis
% Kpd_SC= alpha_SC*Lsd;       % rad/s*H, Proportional gain, d-axis
% % Kid_SC= alpha_SC*Rs*20;        % rad/s*ohm, integral gain, d-axis
% Kid_SC= alpha_SC*(Rs+Bsd);        % rad/s*ohm, integral gain, d-axis
% % Kid_SC= alpha_SC*(Rs*20+Bsd);        % rad/s*ohm, integral gain, d-axis
% 
% Kpq_SC= alpha_SC*Lsq;       % rad/s*H, Proportional gain, q-axis
% Bsq=(Lsq*alpha_SC-Rs)*Ksb;        % ohm, active damping resistance, q-axis
% % Kiq_SC= alpha_SC*Rs*20;        % rad/s*ohm, integral gain, q-axis
% Kiq_SC= alpha_SC*(Rs+Bsq);        % rad/s*ohm, integral gain, q-axis
% % Kiq_SC= alpha_SC*(Rs*20+Bsq);        % rad/s*ohm, integral gain, q-axis

Ksb=0.05;                    % active damping coefficient
alpha_SC= 2*pi*fs/50;       % rad/s, bandwidth (2*pi*500)
% alpha_SC= 2*pi*60;
Bsd=(Lsd*alpha_SC-Rs)*Ksb;        % ohm, active damping resistance, d-axis
Kpd_SC= alpha_SC*Lsd;       % rad/s*H, Proportional gain, d-axis
% Kid_SC= alpha_SC*Rs*20;        % rad/s*ohm, integral gain, d-axis
% Kid_SC= alpha_SC*(Rs+Bsd);        % rad/s*ohm, integral gain, d-axis
Kid_SC= alpha_SC*(Rs*20+Bsd);        % rad/s*ohm, integral gain, d-axis

Kpq_SC= alpha_SC*Lsq;       % rad/s*H, Proportional gain, q-axis
Bsq=(Lsq*alpha_SC-Rs)*Ksb;        % ohm, active damping resistance, q-axis
% Kiq_SC= alpha_SC*Rs*20;        % rad/s*ohm, integral gain, q-axis
% Kiq_SC= alpha_SC*(Rs+Bsq);        % rad/s*ohm, integral gain, q-axis
Kiq_SC= alpha_SC*(Rs*20+Bsq);        % rad/s*ohm, integral gain, q-axis

%% Synchronous machine speed controller
%**********************************************************************
% Speed Control parameters: in rad/s and p.u.                                                      
%**********************************************************************
% alpha_SW= alpha_SC/500;    % rad/s, bandwidth (2*pi*2)
% % alpha_SW= alpha_SC/100/5;    % rad/s, bandwidth (2*pi*2)
% Ba_SW= (2*Htot*alpha_SW-btot/(Tb/(wb/pp)))*1;  % rad, active damping 
% % Kp_SW= 2*Htot*alpha_SW/2;        % rad, Proportional gain, d-axis
% % Ki_SW= 2*Htot*alpha_SW^2/10; % alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW);      % rad^2/s, integral gain, d-axis
% Kp_SW= 2*Htot*alpha_SW;        % rad, Proportional gain, d-axis
% % Ki_SW= 2*Htot*alpha_SW^2; % alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW)
% Ki_SW= alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW);
% % Ki_SW=Kp_SW/3;

alpha_SW= alpha_SC/100;    % rad/s, bandwidth (2*pi*2)
% alpha_SW= alpha_SC/500;    % rad/s, bandwidth (2*pi*2)
Ba_SW= (2*Htot*alpha_SW-btot/(Tb/(wb/pp)))*1;  % rad, active damping 
% Kp_SW= 2*Htot*alpha_SW/2;        % rad, Proportional gain, d-axis
% Ki_SW= 2*Htot*alpha_SW^2/10; % alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW);      % rad^2/s, integral gain, d-axis
Kp_SW= 2*Htot*alpha_SW;        % rad, Proportional gain, d-axis
% Ki_SW= 2*Htot*alpha_SW^2; % alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW)
Ki_SW= alpha_SW*(btot/(Tb/(wb/pp))+Ba_SW);
% Ki_SW=Kp_SW/3;

% initial rotor speed that activates speed controller
f_ref_init=10;

%% Induction machine
Te_rate_IM=55000/(2*pi*50/2);

%% Wind turbine model and power/pitch regulators
% load the optimal power-speed curve
load wPrefpu.txt

% load the optimal wind-power curve
load VPoptpu.txt

% load the optimal speed - wind curve
load wVoptpu.txt

% WT rated active power & speed
% Pb=31500;       % W, active power base (0.9 power factor)
Pb=Sb;       % W, active power base (0.9 power factor)
wb_WT=wb/1;   % rad/s, rated rotor speed at the high speed shaft
Htot_WT= 0.5*Jtot*(wb_WT/pp)^2/Sb;    % s, inertia constant
% Htot_WT= 4.18;    % s, inertia constant

% GE full converter WT - 2.5 MW model
Vin=3;          % m/s, cut in wind speed
Vout=25;        % m/s, cut out wind speed
% Ka=0.00159; % kg/m, 1/2*rho*Ar, defined directly inside function 'Pwind'
Kb=56.6;    % m*rad/s,

% WT max min rotor speed
wmax_WT_pu=1.2; % p.u.
wmin_WT_pu=0.1; % p.u.

% Speed reference filter
Tf_wref=1;     % s/rad (60), speed reference filter

% Initial conditions for simulation
Vw0=8;      % m/s, initial wind speed
w0_WT_pu = min(interp1(wVoptpu(:,2),wVoptpu(:,1),Vw0), wmax_WT_pu); % p.u., initial rotor speed

%**********************************************************************
% Pitch regulator & compensator                                                  
%**********************************************************************
% alpha_CC= 2*pi*fs/10;     % rad/s, bandwidth (2*pi*500)
Kp_beta= 150;               % pu, Proportional gain, pitch regualtor
Ki_beta= 25;                % rad/s, integral gain, pitch regualtor
beta_max=27;                % deg, maximum pitch angle
beta_min=0;                 % deg, minimum pitch angle
beta_ramp_max=10;           % deg/s, maximum pitch ramp-up rate
beta_ramp_min=-10;          % deg/s, maximum pitch ramp-down angle

Kp_comp= 3;                 % pu, Proportional gain, pitch compensator
Ki_comp= 30;                % rad/s, integral gain, pitch compensator

Tp_actuator=0.3;            % s/rad, pitch actuator time constant





