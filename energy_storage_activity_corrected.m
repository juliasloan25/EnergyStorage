%% SCRIPT OF THE ENERGY STORAGE ACTIVITY
% ELECTRICAL POWER SYSTEM
% Original author: Francisco Diaz-Gonzalez, 06.11.2023
% Syntax corrected and formatting improved.
% Original electrical parameters and controller equations have been retained.

clearvars;
clc;

%% MAIN SYSTEM RATINGS
AC_VOLTAGE_SOURCE.Ug = 380;              % (V) RMS line-to-line voltage
AC_VOLTAGE_SOURCE.F = 50;                % (Hz) Grid frequency

ACDC_CONVERTER_1.P = 10e3;              % (W) Rated power
DCDC_CONVERTER_1.P = 10e3;              % (W) Rated power

BATTERY_1.P = 10e3;                     % (W) Rated power
BATTERY_1.E = 10e3;                     % (Wh) Rated energy capacity
BATTERY_1.CELL = load('DATA_LI_ION_CELL.mat');

AC_LOAD_1.P = 10e3;                     % (W) Rated power
AC_LOAD_1.P_T = load('DATA_LOAD_TIME_STEP_900.mat');
AC_LOAD_1.P_T.DATA(:,2) = AC_LOAD_1.P_T.DATA(:,2) ...
    / max(AC_LOAD_1.P_T.DATA(:,2)) * AC_LOAD_1.P;

PV_1.P = 10e3;                          % (W) Rated power
PV_1.P_T = load('DATA_IRRADIANCE_TIME_STEP_3600.mat');
PV_1.P_T.DATA(:,2) = -PV_1.P_T.DATA(:,2) ...
    / max(PV_1.P_T.DATA(:,2)) * PV_1.P;

%% SYSTEM ARCHITECTURE
% 1: Battery charger (AC/DC: Q and U_OUT; DC/DC: U_OUT)
% 2: Energy storage  (AC/DC: P and Q; DC/DC: U_IN)
ARCHITECTURE = 1;

switch ARCHITECTURE
    case 1
        ACDC_CONVERTER_CONTROLLER_1.CM = 1;
        DCDC_CONVERTER_CONTROLLER_1.CM = 1;
    case 2
        ACDC_CONVERTER_CONTROLLER_1.CM = 2;
        DCDC_CONVERTER_CONTROLLER_1.CM = 2;
    otherwise
        error('ARCHITECTURE must be either 1 or 2.');
end

%% VOLTAGE AND CURRENT RATINGS OF COMPONENTS
ACDC_CONVERTER_1.U_IN = AC_VOLTAGE_SOURCE.Ug;
ACDC_CONVERTER_1.U_OUT = ACDC_CONVERTER_1.U_IN ...
    * 2 * sqrt(2) / sqrt(3) * 1.3;
ACDC_CONVERTER_1.I_IN = ACDC_CONVERTER_1.P / ACDC_CONVERTER_1.U_IN;

DC_CAPACITIVE_FILTER_1.U = ACDC_CONVERTER_1.U_OUT;

DCDC_CONVERTER_1.U_IN = DC_CAPACITIVE_FILTER_1.U;
DCDC_CONVERTER_1.U_OUT = DCDC_CONVERTER_1.U_IN;
DCDC_CONVERTER_1.I_OUT = DCDC_CONVERTER_1.P ...
    / (DCDC_CONVERTER_1.U_OUT * 0.5);

BATTERY_1.U = DCDC_CONVERTER_1.U_OUT * 0.7;

%% PARAMETERS FOR PASSIVE COMPONENTS
AC_INDUCTIVE_FILTER_1.L = 0.005;       % (H) Inductance
AC_INDUCTIVE_FILTER_1.LAMBDA = 0.01;   % (H/Ohm) L/R ratio
AC_INDUCTIVE_FILTER_1.R = AC_INDUCTIVE_FILTER_1.L ...
    / AC_INDUCTIVE_FILTER_1.LAMBDA;     % (Ohm) Resistance
AC_INDUCTIVE_FILTER_1.F = AC_VOLTAGE_SOURCE.F;

DC_CAPACITIVE_FILTER_1.C = 2 * 0.3 * ACDC_CONVERTER_1.P ...
    / (DC_CAPACITIVE_FILTER_1.U^2 ...
    - 0.9 * DC_CAPACITIVE_FILTER_1.U^2);  % (F) Capacitance: original equation

DC_INDUCTIVE_FILTER_1.L = 0.005;       % (H) Inductance

%% PARAMETERS FOR BATTERIES
BATTERY_1.Ns = ceil(BATTERY_1.U / mean(BATTERY_1.CELL.OCV_LUT));
                                              % Number of cells in series
BATTERY_1.Np = ceil(BATTERY_1.E ...
    / (BATTERY_1.Ns * mean(BATTERY_1.CELL.OCV_LUT) ...
    * BATTERY_1.CELL.C / 3600));               % Number of parallel cells

BATTERY_1.R = BATTERY_1.CELL.R * BATTERY_1.Ns / BATTERY_1.Np; % (Ohm)
BATTERY_1.C = BATTERY_1.CELL.C * BATTERY_1.Np;                % (A*s)

% Use column vectors for both look-up tables to make interp1 robust.
BATTERY_1.OCV_LUT = BATTERY_1.Ns * BATTERY_1.CELL.OCV_LUT(:);
BATTERY_1.SOC_LUT = BATTERY_1.CELL.SOC_LUT(:);

BATTERY_1.Ub_exp = max(BATTERY_1.OCV_LUT); % (V) Maximum pack OCV
BATTERY_1.Ub_min = min(BATTERY_1.OCV_LUT); % (V) Minimum pack OCV
BATTERY_1.EFF = 0.95;                     % (-) Efficiency
BATTERY_1.CEQ = BATTERY_1.C * (BATTERY_1.U + BATTERY_1.Ub_min) ...
    / (BATTERY_1.U^2 - BATTERY_1.Ub_min^2);

%% PARAMETERS FOR CONTROLLERS
switch ARCHITECTURE
    case 1  % BATTERY CHARGER
        % AC/DC converter controller: Q and U_OUT
        ACDC_CONVERTER_CONTROLLER_1.P_IN_SET = 0;
        ACDC_CONVERTER_CONTROLLER_1.U_OUT_SET = ACDC_CONVERTER_1.U_OUT;

        ACDC_CONVERTER_CONTROLLER_1.WU = 2;
        ACDC_CONVERTER_CONTROLLER_1.XU = 1;
        ACDC_CONVERTER_CONTROLLER_1.KPU = ...
            4 * DC_CAPACITIVE_FILTER_1.C * ACDC_CONVERTER_CONTROLLER_1.XU ...
            * ACDC_CONVERTER_CONTROLLER_1.WU / (3 / (2 * 1.3));
        ACDC_CONVERTER_CONTROLLER_1.KIU = ...
            2 * DC_CAPACITIVE_FILTER_1.C * ACDC_CONVERTER_CONTROLLER_1.WU^2 ...
            / (3 / (2 * 1.3));

        ACDC_CONVERTER_CONTROLLER_1.WI = 20;
        ACDC_CONVERTER_CONTROLLER_1.XI = 0.7;
        ACDC_CONVERTER_CONTROLLER_1.KPI = AC_INDUCTIVE_FILTER_1.L ...
            / AC_INDUCTIVE_FILTER_1.LAMBDA; % Proportional gain
        ACDC_CONVERTER_CONTROLLER_1.KII = AC_INDUCTIVE_FILTER_1.R ...
            / AC_INDUCTIVE_FILTER_1.LAMBDA; % Integral gain

        % DC/DC converter controller: U_OUT
        DCDC_CONVERTER_CONTROLLER_1.U_OUT_SET = BATTERY_1.Ub_exp;
        DCDC_CONVERTER_CONTROLLER_1.U_IN_SET = DCDC_CONVERTER_1.U_IN;

        DCDC_CONVERTER_CONTROLLER_1.WU = 1; % (rad/s) Natural frequency
        DCDC_CONVERTER_CONTROLLER_1.XU = 1; % (-) Damping coefficient
        DCDC_CONVERTER_CONTROLLER_1.KPU = ...
            2 * DCDC_CONVERTER_CONTROLLER_1.XU ...
            * DCDC_CONVERTER_CONTROLLER_1.WU * BATTERY_1.CEQ;
        DCDC_CONVERTER_CONTROLLER_1.KIU = ...
            DCDC_CONVERTER_CONTROLLER_1.WU^2 * BATTERY_1.CEQ;

        DCDC_CONVERTER_CONTROLLER_1.WI = 10; % (rad/s) Natural frequency
        DCDC_CONVERTER_CONTROLLER_1.XI = 1;  % (-) Damping coefficient
        DCDC_CONVERTER_CONTROLLER_1.KPI = ...
            2 * DCDC_CONVERTER_CONTROLLER_1.XI ...
            * DCDC_CONVERTER_CONTROLLER_1.WI * DC_INDUCTIVE_FILTER_1.L ...
            / DCDC_CONVERTER_1.U_IN;
        DCDC_CONVERTER_CONTROLLER_1.KII = ...
            DCDC_CONVERTER_CONTROLLER_1.WI^2 * DC_INDUCTIVE_FILTER_1.L ...
            / DCDC_CONVERTER_1.U_IN;

        BATTERY_1.SOC_INI = 0.7;
        BATTERY_1.U_INI = interp1(BATTERY_1.SOC_LUT, ...
            BATTERY_1.OCV_LUT, BATTERY_1.SOC_INI, 'linear', 'extrap');
        DCDC_CONVERTER_CONTROLLER_1.INT_INI = ...
            (BATTERY_1.U_INI / DCDC_CONVERTER_1.U_IN) ...
            - sign((DCDC_CONVERTER_CONTROLLER_1.U_OUT_SET ...
            - BATTERY_1.U_INI) * DCDC_CONVERTER_CONTROLLER_1.KPU) ...
            * min(abs((DCDC_CONVERTER_CONTROLLER_1.U_OUT_SET ...
            - BATTERY_1.U_INI) * DCDC_CONVERTER_CONTROLLER_1.KPU), ...
            DCDC_CONVERTER_1.I_OUT) * DCDC_CONVERTER_CONTROLLER_1.KPI;

    case 2  % GRID-CONNECTED ENERGY STORAGE
        % AC/DC converter controller: P and Q
        ACDC_CONVERTER_CONTROLLER_1.P_IN_SET = 0;
        ACDC_CONVERTER_CONTROLLER_1.U_OUT_SET = ACDC_CONVERTER_1.U_OUT;

        ACDC_CONVERTER_CONTROLLER_1.WU = 2;
        ACDC_CONVERTER_CONTROLLER_1.XU = 1;
        ACDC_CONVERTER_CONTROLLER_1.KPU = ...
            4 * DC_CAPACITIVE_FILTER_1.C * ACDC_CONVERTER_CONTROLLER_1.XU ...
            * ACDC_CONVERTER_CONTROLLER_1.WU / (3 / (2 * 1.3));
        ACDC_CONVERTER_CONTROLLER_1.KIU = ...
            2 * DC_CAPACITIVE_FILTER_1.C * ACDC_CONVERTER_CONTROLLER_1.WU^2 ...
            / (3 / (2 * 1.3));

        ACDC_CONVERTER_CONTROLLER_1.WI = 20;
        ACDC_CONVERTER_CONTROLLER_1.XI = 0.7;
        ACDC_CONVERTER_CONTROLLER_1.KPI = AC_INDUCTIVE_FILTER_1.L ...
            / AC_INDUCTIVE_FILTER_1.LAMBDA; % Proportional gain
        ACDC_CONVERTER_CONTROLLER_1.KII = AC_INDUCTIVE_FILTER_1.R ...
            / AC_INDUCTIVE_FILTER_1.LAMBDA; % Integral gain

        % DC/DC converter controller: U_IN
        DCDC_CONVERTER_CONTROLLER_1.U_IN_SET = DCDC_CONVERTER_1.U_IN;
        DCDC_CONVERTER_CONTROLLER_1.U_OUT_SET = BATTERY_1.Ub_exp;

        DCDC_CONVERTER_CONTROLLER_1.WU = 1; % (rad/s) Natural frequency
        DCDC_CONVERTER_CONTROLLER_1.XU = 1; % (-) Damping coefficient
        DCDC_CONVERTER_CONTROLLER_1.KPU = ...
            2 * DCDC_CONVERTER_CONTROLLER_1.XU ...
            * DCDC_CONVERTER_CONTROLLER_1.WU * DC_CAPACITIVE_FILTER_1.C ...
            / (DCDC_CONVERTER_1.U_OUT / DCDC_CONVERTER_1.U_IN);
        DCDC_CONVERTER_CONTROLLER_1.KIU = ...
            DCDC_CONVERTER_CONTROLLER_1.WU^2 * DC_CAPACITIVE_FILTER_1.C ...
            / (DCDC_CONVERTER_1.U_OUT / DCDC_CONVERTER_1.U_IN);

        DCDC_CONVERTER_CONTROLLER_1.WI = 10; % (rad/s) Natural frequency
        DCDC_CONVERTER_CONTROLLER_1.XI = 1;  % (-) Damping coefficient
        DCDC_CONVERTER_CONTROLLER_1.KPI = ...
            2 * DCDC_CONVERTER_CONTROLLER_1.XI ...
            * DCDC_CONVERTER_CONTROLLER_1.WI * DC_INDUCTIVE_FILTER_1.L ...
            / DCDC_CONVERTER_1.U_OUT;
        DCDC_CONVERTER_CONTROLLER_1.KII = ...
            DCDC_CONVERTER_CONTROLLER_1.WI^2 * DC_INDUCTIVE_FILTER_1.L ...
            / DCDC_CONVERTER_1.U_OUT;

        BATTERY_1.SOC_INI = 0.7;
        BATTERY_1.U_INI = interp1(BATTERY_1.SOC_LUT, ...
            BATTERY_1.OCV_LUT, BATTERY_1.SOC_INI, 'linear', 'extrap');
        DCDC_CONVERTER_CONTROLLER_1.INT_INI = ...
            (BATTERY_1.U_INI / DCDC_CONVERTER_1.U_IN) ...
            - sign((DCDC_CONVERTER_CONTROLLER_1.U_IN_SET ...
            - DCDC_CONVERTER_1.U_IN) * DCDC_CONVERTER_CONTROLLER_1.KPU) ...
            * min(abs((DCDC_CONVERTER_CONTROLLER_1.U_IN_SET ...
            - DCDC_CONVERTER_1.U_IN) * DCDC_CONVERTER_CONTROLLER_1.KPU), ...
            DCDC_CONVERTER_1.I_OUT) * DCDC_CONVERTER_CONTROLLER_1.KPI;
end
