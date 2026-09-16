%% TN-NTN integration baseline 
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

results_dir = fullfile(script_dir, 'Results', 'TN_NTN');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

rng(1, 'twister');

%% Signal and power parameters
fc = 6; %[GHz]
c = 3e8; %[m/s]
num_UE_sector = 10;
num_UE_victim = 3 * num_UE_sector;
num_UE_interferer = 6 * num_UE_victim;
num_UE_NTN_sector = 5;
num_UE_NTN = 3 * num_UE_NTN_sector;
scs = 15; %[kHz]
subcarrier_per_rb = 12; % Per spec
num_rb = 100;
num_rb_FR = num_rb * 0.5;
num_rb_bonus = num_rb * 0.2;
num_rb_PR = num_rb - num_rb_FR - num_rb_bonus;
BW_total = scs * subcarrier_per_rb * num_rb * 1000; %[Hz]
BW_FR_raw = scs * subcarrier_per_rb * num_rb_FR * 1000; %[Hz]
BW_PR_raw = scs * subcarrier_per_rb * num_rb_PR * 1000 * 1 / 3; %[Hz]
BW_bonus = scs * subcarrier_per_rb * num_rb_bonus * 1000 * 1 / 3; %[Hz]
% Channel_BW = 20 MHz
NF = 9; %[dB]
pn_total = BW_total * db2pow(-174 + NF); %[dBm]
pn_total_dB = pow2db(pn_total);
p_BS_dBm = 49; %[dBm]
p_BS = db2pow(p_BS_dBm - 30); %[W]
p_UE_NTN_dBm = 26; %[dBm]
p_UE_NTN = db2pow(p_UE_NTN_dBm - 30); %[W]
g_UE = 1; %[W] (= 30 dBm)
SINR_threshold = 0; %[dB]
SLA = 30; %[dB]
G_E_max = 15; %[dBi]
theta_3dB_BS = deg2rad(65); % Per spec
theta_3dB_UE_NTN = deg2rad(180);
environment_mode = 'UMa'; % Choose among 'RMa', 'UMa', or 'UMi'
h_UE_ground = 1.5; % Ground UE
h_UE_UAV = 300; % Drone
h_UE_plane = 10e3; % Aircraft
ISD_TN = 500; %[m]
ISD_NTN = 2 * 500; %[m]
l_TN = ISD_TN / 3;
l_NTN = ISD_NTN / 3;
default_realizations = 1000;
realizations_override = str2double(getenv('TN_NTN_REALIZATIONS'));
if ~isnan(realizations_override) && realizations_override > 0
    realizations = round(realizations_override);
else
    realizations = default_realizations;
end

%% Speed and Doppler parameters
v_plane = 900; %[km/h]
v_plane = v_plane / 3.6; %[m/s]
v_UAV = 10; %[m/s]
dt = 30;
tMax = ISD_NTN / v_plane; % d = sqrt(13) * l is the max distance an aircraft can remain in a cell, but we consider ISD for simplicity
tVec = 0 : dt : tMax;
tLen = length(tVec);

%% Rain attenuation parameters
polarization_type = 'Circular'; % Choose among 'Horizontal', 'Vertical', or 'Circular'
R_point_01 = 85; % From Recommendation ITU-R P.837-7; An estimate based on Figure 1 (https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.837-7-201706-I!!PDF-E.pdf)
k_H = 0.0007056; % From Recommendation ITU-R P.838 at 6 GHz (https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.838-3-200503-I!!PDF-E.pdf)
k_V = 0.0004878; % From Recommendation ITU-R P.838
alpha_H = 1.5900; % From Recommendation ITU-R P.838
alpha_V = 1.5728; % From Recommendation ITU-R P.838
switch polarization_type
    case 'Horizontal'
        k_rain = k_H;
        alpha_rain = alpha_H;
    case 'Vertical'
        k_rain = k_V;
        alpha_rain = alpha_V;
    case 'Circular'
        k_rain = (k_H + k_V) / 2; % Average; Eq. (4) of Recommendation ITU-R P.838 for circular polarization
        alpha_rain = (k_H * alpha_H + k_V * alpha_V) / (2 * k_rain); % Weighted average; Eq. (5) of Recommendation ITU-R P.838 for circular polarization
end
gamma_R = k_rain * R_point_01 ^ alpha_rain; %[dB/km]

%% Positioning
switch environment_mode
    case 'RMa'
        h_BS = 35;
    case 'UMa'
        h_BS = 25;
    case 'UMi'
        h_BS = 10;
end
theta_downtilt_BS = pi/ 2 + atan((h_BS - h_UE_ground) / (2 * l_TN));
pos_BS_desired = [0, 0, h_BS];
pos_BS_interferer = [cos((0 : 5).' * pi / 3) * ISD_TN, sin((0 : 5).' * pi / 3) * ISD_TN, ones(6, 1) * h_BS];
pos_UE_victim = zeros(num_UE_victim, 3);
pos_UE_interferer = zeros(num_UE_interferer, 3);
pos_UE_NTN = zeros(num_UE_NTN, 3);
vel_UE_NTN = zeros(num_UE_NTN, 1);

range_target = (5 : 5 : 2 * l_TN).';
num_points = length(range_target);
interference_target = zeros(num_points - 1, 1);
SINR_target = zeros(num_points - 1, 1);
tput_target = zeros(num_points - 1, 1);
interference_target_cell = cell(num_points - 1, 1);
SINR_target_cell = cell(num_points - 1, 1);
tput_target_cell = cell(num_points - 1, 1);

pos_UE_sorted_mat = zeros(num_UE_victim, realizations);
interference_BS2UE_sorted_mat = zeros(num_UE_victim, realizations);
SINR_BS2UE_sorted_mat = zeros(num_UE_victim, realizations);
tput_BS2UE_sorted_mat = zeros(num_UE_victim, realizations);

% m1 = zeros(realizations, 1);
% m2 = zeros(realizations, 1);

%% Main Loop
for i = 1 : realizations
    if mod(i, max(1, round(realizations / 10))) == 0
        disp([num2str(round(i / realizations * 100)), '% Completed'])
    end
    [p_x, p_y, v_x_1, v_y_1] = gen_hex(l_TN, num_UE_sector, [l_TN / 2, sqrt(3) / 2 * l_TN], pos_BS_desired(1 : 2)); % Upper right hexagon
    pos_UE_victim(1 : num_UE_sector, 1) = p_x;
    pos_UE_victim(1 : num_UE_sector, 2) = p_y;
    [p_x, p_y, v_x_2, v_y_2] = gen_hex(l_TN, num_UE_sector, [l_TN / 2, -sqrt(3) / 2 * l_TN], pos_BS_desired(1 : 2)); % Lower right hexagon
    pos_UE_victim(1 + num_UE_sector : 2 * num_UE_sector, 1) = p_x;
    pos_UE_victim(1 + num_UE_sector : 2 * num_UE_sector, 2) = p_y;
    [p_x, p_y, v_x_3, v_y_3] = gen_hex(l_TN, num_UE_sector, [-l_TN, 0], pos_BS_desired(1 : 2)); % Middle left hexagon
    pos_UE_victim(1 + 2 * num_UE_sector : 3 * num_UE_sector, 1) = p_x;
    pos_UE_victim(1 + 2 * num_UE_sector : 3 * num_UE_sector, 2) = p_y;
    pos_UE_victim(:, 3) = h_UE_ground;

    [p_x, p_y, v_x_1_NTNUE, v_y_1_NTNUE] = gen_hex(l_NTN, num_UE_NTN_sector, [l_NTN / 2, sqrt(3) / 2 * l_NTN], pos_BS_desired(1 : 2)); % Upper right hexagon
    pos_UE_NTN(1 : num_UE_NTN_sector, 1) = p_x;
    pos_UE_NTN(1 : num_UE_NTN_sector, 2) = p_y;
    [p_x, p_y, v_x_2_NTNUE, v_y_2_NTNUE] = gen_hex(l_NTN, num_UE_NTN_sector, [l_NTN / 2, -sqrt(3) / 2 * l_NTN], pos_BS_desired(1 : 2)); % Lower right hexagon
    pos_UE_NTN(1 + num_UE_NTN_sector : 2 * num_UE_NTN_sector, 1) = p_x;
    pos_UE_NTN(1 + num_UE_NTN_sector : 2 * num_UE_NTN_sector, 2) = p_y;
    [p_x, p_y, v_x_3_NTNUE, v_y_3_NTNUE] = gen_hex(l_NTN, num_UE_NTN_sector, [-l_NTN, 0], pos_BS_desired(1 : 2)); % Middle left hexagon
    pos_UE_NTN(1 + 2 * num_UE_NTN_sector : 3 * num_UE_NTN_sector, 1) = p_x;
    pos_UE_NTN(1 + 2 * num_UE_NTN_sector : 3 * num_UE_NTN_sector, 2) = p_y;

    type_UE_NTN = randi(2, num_UE_NTN, 1); % NTN UE type: 1 for Airplane & 2 for UAV
    % Height of UEs
    pos_UE_NTN(type_UE_NTN == 1, 3) = h_UE_plane;
    pos_UE_NTN(type_UE_NTN == 2, 3) = h_UE_UAV;
    % Velocity of UEs
    vel_UE_NTN(type_UE_NTN == 1) = v_plane;
    vel_UE_NTN(type_UE_NTN == 2) = v_UAV;
    displacement_theta = unifrnd(0, 2 * pi, num_UE_NTN, 1);
    v = [vel_UE_NTN .* cos(displacement_theta), vel_UE_NTN .* sin(displacement_theta), zeros(num_UE_NTN, 1)];
    posUE_victim_displaced = pos_UE_NTN;
    
    range_UE_victim = sqrt(pos_UE_victim(:, 1) .^ 2 + pos_UE_victim(:, 2) .^ 2);
    [range_UE_victim_sorted, range_UE_victim_sorted_ind] = sort(range_UE_victim);
    pos_UE_sorted_mat(:, i) = range_UE_victim_sorted;

    if i == 1 % Plotting the scenario only for the first realization
        h_plot = zeros(6, 1);
        for j = 1 : 6
            [p_x, p_y, v_x_1_interferer, v_y_1_interferer] = gen_hex(l_TN, num_UE_sector, [l_TN / 2, sqrt(3) / 2 * l_TN], pos_BS_interferer(j, 1 : 2)); % Upper right hexagon
            pos_UE_interferer(1 + (j - 1) * num_UE_victim : num_UE_sector + (j - 1) * num_UE_victim, 1) = p_x;
            pos_UE_interferer(1 + (j - 1) * num_UE_victim : num_UE_sector + (j - 1) * num_UE_victim, 2) = p_y;
            [p_x, p_y, v_x_2_interferer, v_y_2_interferer] = gen_hex(l_TN, num_UE_sector, [l_TN / 2, -sqrt(3) / 2 * l_TN], pos_BS_interferer(j, 1 : 2)); % Lower right hexagon
            pos_UE_interferer(1 + num_UE_sector + (j - 1) * num_UE_victim : 2 * num_UE_sector + (j - 1) * num_UE_victim, 1) = p_x;
            pos_UE_interferer(1 + num_UE_sector + (j - 1) * num_UE_victim : 2 * num_UE_sector + (j - 1) * num_UE_victim, 2) = p_y;
            [p_x, p_y, v_x_3_interferer, v_y_3_interferer] = gen_hex(l_TN, num_UE_sector, [-l_TN, 0], pos_BS_interferer(j, 1 : 2)); % Middle left hexagon
            pos_UE_interferer(1 + 2 * num_UE_sector + (j - 1) * num_UE_victim : 3 * num_UE_sector + (j - 1) * num_UE_victim, 1) = p_x;
            pos_UE_interferer(1 + 2 * num_UE_sector + (j - 1) * num_UE_victim : 3 * num_UE_sector + (j - 1) * num_UE_victim, 2) = p_y;
            figure(501)
            hold on
            grid on
            axis square
            plot(v_x_1_interferer, v_y_1_interferer, 'LineWidth', 2, 'Color', 'cyan')
            plot(v_x_2_interferer, v_y_2_interferer, 'LineWidth', 2, 'Color', 'cyan')
            plot(v_x_3_interferer, v_y_3_interferer, 'LineWidth', 2, 'Color', 'cyan')
            h_plot(1) = plot(pos_BS_interferer(:, 1), pos_BS_interferer(:, 2), 'diamond', 'MarkerSize', 12, 'MarkerFaceColor', 'black', 'MarkerEdgeColor', 'black');
            h_plot(2) = plot(pos_UE_interferer(:, 1), pos_UE_interferer(:, 2), '*', 'MarkerSize', 6, 'Color', 'red');
            % h_plot(2) = plot(pos_UE_interferer(:, 1), pos_UE_interferer(:, 2), 'x', 'MarkerSize', 6, 'Color', 'black');
        end
        plot(v_x_1, v_y_1, 'LineWidth', 2, 'Color', 'green')
        plot(v_x_2, v_y_2, 'LineWidth', 2, 'Color', 'green')
        plot(v_x_3, v_y_3, 'LineWidth', 2, 'Color', 'green')
        % plot(v_x_1_NTNUE, v_y_1_NTNUE, 'LineWidth', 2, 'Color', 'green')
        % plot(v_x_2_NTNUE, v_y_2_NTNUE, 'LineWidth', 2, 'Color', 'green')
        % plot(v_x_3_NTNUE, v_y_3_NTNUE, 'LineWidth', 2, 'Color', 'green')
        h_plot(6) = plot(pos_BS_desired(1) + 10, pos_BS_desired(2), 'diamond', 'MarkerSize', 14, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'blue');
        h_plot(3) = plot(pos_BS_desired(1), pos_BS_desired(2), 'diamond', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta', 'MarkerEdgeColor', 'magenta');
        h_plot(4) = plot(pos_UE_NTN(:, 1), pos_UE_NTN(:, 2), 'o', 'MarkerSize', 10, 'Color', 'blue');
        h_plot(5) = plot(pos_UE_victim(:, 1), pos_UE_victim(:, 2), '*', 'MarkerSize', 6, 'Color', 'red');
        title('TN-NTN Integration', 'Interpreter', 'latex', 'FontSize', 14)
        xlabel('$x$-Axis Position [m]', 'Interpreter', 'latex', 'FontSize', 11)
        ylabel('$y$-Axis Position [m]', 'Interpreter', 'latex', 'FontSize', 11)
        h_legend = legend(h_plot([3, 6, 1, 4, 5]), 'Desired TN BS (Downtilted)', 'Desired NTN BS (Uptilted)', 'Interfering TN BSs', 'Interfering NTN UEs', 'Victim TN UEs');
        % h_legend = legend(h_plot([3, 1, 4, 5, 2]), 'Desired TN BS (Downtilted) and TNT BS (Uptilted)', 'Interfering TN BSs', 'Interfering NTN UEs', 'Victim TN UEs', 'Other TN UEs');
        h_legend.Location = 'northwest';
        h_legend.Interpreter = 'latex';
        h_legend.FontSize = 12;
        set(gcf, 'Color', 'w')
        saveas(gcf, fullfile(results_dir, 'Scenario_Layout.png'));
        savefig(gcf, fullfile(results_dir, 'Scenario_Layout.fig'));
        hold off
        pause(0.1)
    end

    %% Phase 1 -- BS to TN UE
    link = 'BS2UE';
    fading_mode = 'Rayleigh'; % Choose among 'Rayleigh' or 'Rician'
    distance_BS2UE_desired = sqrt(sum((pos_BS_desired - pos_UE_victim) .^ 2, 2));
    distance_BS2UE_2D_desired = sqrt(sum((pos_BS_desired(1 : 2) - pos_UE_victim(:, 1 : 2)) .^ 2, 2));
    prLoS_BS2UE = ProbLoS_Vec(link, environment_mode, distance_BS2UE_2D_desired, [], [], [], h_UE_ground);
    [PL_LoS_BS2UE, PL_NLoS_BS2UE] = PathLoss_Vec(link, environment_mode, distance_BS2UE_2D_desired, distance_BS2UE_desired, h_BS, [], h_UE_ground, fc);
    PL_BS2UE_desired = prLoS_BS2UE .* PL_LoS_BS2UE + (1 - prLoS_BS2UE) .* PL_NLoS_BS2UE;
    theta_BS2UE_desired = pi - acos((h_BS - h_UE_ground) ./ distance_BS2UE_desired);
    switch fading_mode
        case 'Rayleigh'
            fading_BS2UE_desired = exprnd(1, [num_UE_victim, 1]);
        case 'Rician'
            K_BS2UE = 10; % Rician K-factor
            nu_BS2UE = sqrt(K_BS2UE / (1 + K_BS2UE));
            sigma_BS2UE = 1 / sqrt(2 * (1 + K_BS2UE));
            X = normrnd(nu_BS2UE * cos(pi / 3), sigma_BS2UE, [num_UE_victim, 1]); % pi / 3 is just a random phase
            Y = normrnd(nu_BS2UE * sin(pi / 3), sigma_BS2UE, [num_UE_victim, 1]);
            fading_BS2UE_desired = X .^ 2 + Y .^ 2;
    end
    g_desired = gain_ant(theta_BS2UE_desired, theta_downtilt_BS, theta_3dB_BS, SLA, g_UE);
    rain_att_desired = rain_attenuation(distance_BS2UE_desired, R_point_01, gamma_R, alpha_rain, fc);
    power_BS2UE_desired = p_BS * g_desired .* fading_BS2UE_desired ./ (PL_BS2UE_desired .* rain_att_desired);

    power_BS2UE_interferer = zeros(num_UE_victim, 6);
    for j = 1 : 6
        distance_BS2UE_interferer = sqrt(sum((pos_BS_interferer(j, :) - pos_UE_victim) .^ 2, 2));
        distance_BS2UE_2D_interferer = sqrt(sum((pos_BS_interferer(j, 1 : 2) - pos_UE_victim(:, 1 : 2)) .^ 2, 2));
        prLoS_BS2UE = ProbLoS_Vec(link, environment_mode, distance_BS2UE_2D_interferer, [], [], [], h_UE_ground);
        [PL_LoS_BS2UE, PL_NLoS_BS2UE] = PathLoss_Vec(link, environment_mode, distance_BS2UE_2D_interferer, distance_BS2UE_interferer, h_BS, [], h_UE_ground, fc);
        PL_BS2UE_interferer = prLoS_BS2UE .* PL_LoS_BS2UE + (1 - prLoS_BS2UE) .* PL_NLoS_BS2UE;
        theta_BS2UE_interferer = pi - acos((h_BS - h_UE_ground) ./ distance_BS2UE_interferer);
        switch fading_mode
            case 'Rayleigh'
                fading_BS2UE_interferer = exprnd(1, [num_UE_victim, 1]);
            case 'Rician'
                K_BS2UE = 10; % Rician K-factor
                nu_BS2UE = sqrt(K_BS2UE / (1 + K_BS2UE));
                sigma_BS2UE = 1 / sqrt(2 * (1 + K_BS2UE));
                X = normrnd(nu_BS2UE * cos(pi / 3), sigma_BS2UE, [num_UE_victim, 1]); % pi / 3 is just a random phase
                Y = normrnd(nu_BS2UE * sin(pi / 3), sigma_BS2UE, [num_UE_victim, 1]);
                fading_BS2UE_interferer = X .^ 2 + Y .^ 2;
        end
        g_BS2UE_interferer = gain_ant(theta_BS2UE_interferer, theta_downtilt_BS, theta_3dB_BS, SLA, g_UE);
        rain_att_interferer = rain_attenuation(distance_BS2UE_interferer, R_point_01, gamma_R, alpha_rain, fc);
        power_BS2UE_interferer(:, j) = p_BS * g_BS2UE_interferer .* fading_BS2UE_interferer ./ (PL_BS2UE_interferer .* rain_att_interferer);
    end
    power_BS2UE_interferer_all = sum(power_BS2UE_interferer, 2);

    %% Phase 2 -- NTN UE to TN UE
    link = 'UAV2UE';
    fading_mode = 'Rician'; % Choose among 'Rayleigh' or 'Rician'
    power_NTN2TN_interferer = zeros(num_UE_victim, num_UE_NTN);
    for j = 1 : num_UE_NTN
        distance_NTN2TN = sqrt(sum((pos_UE_NTN(j, :) - pos_UE_victim) .^ 2, 2));
        distance_NTN2TN_2D = sqrt(sum((pos_UE_NTN(j, 1 : 2) - pos_UE_victim(:, 1 : 2)) .^ 2, 2));
        prLoS_NTN2TN = ProbLoS_Vec(link, environment_mode, distance_NTN2TN_2D, distance_NTN2TN, h_BS, pos_UE_NTN(j, 3), h_UE_ground);%
        [PL_LoS_NTN2TN, PL_NLoS_NTN2TN] = PathLoss_Vec(link, environment_mode, distance_NTN2TN_2D, distance_NTN2TN, h_BS, pos_UE_NTN(j, 3), h_UE_ground, fc);
        PL_NTN2TN = prLoS_NTN2TN .* PL_LoS_NTN2TN + (1 - prLoS_NTN2TN) .* PL_NLoS_NTN2TN;
        theta_NTN2TN = pi - acos((pos_UE_NTN(j, 3) - h_UE_ground) ./ distance_NTN2TN);
        switch fading_mode
            case 'Rayleigh'
                fading_NTN2TN = exprnd(1, [num_UE_victim, 1]);
            case 'Rician'
                K_NTN2TN = 10; % Rician K-factor
                nu_NTN2TN = sqrt(K_NTN2TN / (1 + K_NTN2TN));
                sigma_NTN2TN = 1 / sqrt(2 * (1 + K_NTN2TN));
                X = normrnd(nu_NTN2TN * cos(pi / 3), sigma_NTN2TN, [num_UE_victim, 1]); % pi / 3 is just a random phase
                Y = normrnd(nu_NTN2TN * sin(pi / 3), sigma_NTN2TN, [num_UE_victim, 1]);
                fading_NTN2TN = X .^ 2 + Y .^ 2;
        end
        g_NTN2TN_interferer = gain_ant_NTN(theta_NTN2TN, theta_3dB_UE_NTN, SLA, g_UE);
        rain_att_NTN2TN_interferer = rain_attenuation(distance_NTN2TN, R_point_01, gamma_R, alpha_rain, fc);
        power_NTN2TN_interferer(:, j) = p_UE_NTN * g_NTN2TN_interferer .* fading_NTN2TN ./ (PL_NTN2TN .* rain_att_NTN2TN_interferer);
    end
    power_NTN2TN_interferer_all = sum(power_NTN2TN_interferer, 2);
    
    %{
    m1(i) = pow2db(mean(power_BS2UE_interferer_all));
    m2(i) = pow2db(mean(power_NTN2TN_interferer_all));
    figure(2)
    plot([pow2db(power_BS2UE_interferer_all), pow2db(power_NTN2TN_interferer_all)], 'LineWidth', 2)
    grid on
    if ISD_TN == ISD_NTN
        title(['ISD = ', num2str(ISD_TN), ' m for both TN gNBs and NTN UEs'], 'Interpreter', 'latex', 'FontSize', 12)
    else
        title(['ISD = ', num2str(ISD_TN), ' m for TN gNBs and ISD = ', num2str(ISD_NTN), ' m for NTN UEs'], 'Interpreter', 'latex', 'FontSize', 12)
    end
    xlabel('Victim UE Number', 'Interpreter', 'latex', 'FontSize', 12)
    ylabel('Interference Power', 'Interpreter', 'latex', 'FontSize', 12)
    hh = legend('From TN gNBs', 'From NTN UEs');
    hh.Interpreter = 'latex';
    hh.FontSize = 12;
    %}

    SINR_BS2UE = power_BS2UE_desired ./ (power_BS2UE_interferer_all + power_NTN2TN_interferer_all + pn_total);
    SINR_BS2UE_sorted = SINR_BS2UE(range_UE_victim_sorted_ind);
    tput_BS2UE_sorted = BW_total .* log2(1 + SINR_BS2UE_sorted) / 1e6; %[Mbps]

    interference_BS2UE_sorted_mat(:, i) = power_BS2UE_interferer_all(range_UE_victim_sorted_ind) + power_NTN2TN_interferer_all(range_UE_victim_sorted_ind);
    SINR_BS2UE_sorted_mat(:, i) = SINR_BS2UE_sorted;
    tput_BS2UE_sorted_mat(:, i) = tput_BS2UE_sorted;

    %% Phase 3 -- DFFR
    %{
    is_FR_UE = SINR_BS2UE >= db2pow(SINR_threshold); % Near-Cell UEs
    is_PR_UE = SINR_BS2UE < db2pow(SINR_threshold); % Cell-Edge UEs
    BW_FR = BW_FR_raw;
    BW_PR = BW_PR_raw + BW_bonus;
    beta_FR = BW_FR / BW_total;
    % p_BS_FR = p_BS * beta_FR;
    % p_BS_PR = p_BS * (1 - beta_FR);
    PR1_UEs = is_PR_UE(1 : num_UE_sector);
    PR2_UEs = is_PR_UE(num_UE_sector + 1 : 2 * num_UE_sector);
    PR3_UEs = is_PR_UE(2 * num_UE_sector + 1 : 3 * num_UE_sector);
    pn_FR = BW_FR * db2pow(-174 + NF); %[dBm]
    pn_PR = BW_PR * db2pow(-174 + NF); %[dBm]

    switch fading_mode
        case 'Rayleigh'
            fading_BS2UE_desired = exprnd(1, [num_UE_victim, 1]);
        case 'Rician'
            K_BS2UE = 10; % Rician K-factor
            nu_BS2UE = sqrt(K_BS2UE / (1 + K_BS2UE));
            sigma_BS2UE = 1 / sqrt(2 * (1 + K_BS2UE));
            X = normrnd(nu_BS2UE * cos(pi / 3), sigma_BS2UE, [num_UE_victim, 1]); % pi / 3 is just a random phase
            Y = normrnd(nu_BS2UE * sin(pi / 3), sigma_BS2UE, [num_UE_victim, 1]);
            fading_BS2UE_desired = X .^ 2 + Y .^ 2;
    end
    power_BS2UE_desired = p_BS * g_desired ./ PL_BS2UE_desired .* fading_BS2UE_desired;
    % power_BS2UE_FR_desired = p_BS_FR * g_desired ./ PL_BS2UE_desired .* fading_BS2UE_desired .* is_FR_UE;
    % power_BS2UE_PR_desired = p_BS_PR * g_desired ./ PL_BS2UE_desired .* fading_BS2UE_desired .* is_PR_UE;
    % power_BS2UE_desired = power_BS2UE_FR_desired + power_BS2UE_PR_desired;

    power_BS2UE_interferer = zeros(num_UE_victim, 6);
    for j = 1 : 6
        distance_BS2UE_interferer = sqrt(sum((pos_BS_interferer(j, :) - pos_UE_victim) .^ 2, 2));
        distance_BS2UE_2D_interferer = sqrt(sum((pos_BS_interferer(j, 1 : 2) - pos_UE_victim(:, 1 : 2)) .^ 2, 2));
        prLoS_BS2UE = ProbLoS_Vec(link, environment_mode, distance_BS2UE_2D_interferer, [], [], [], h_UE_ground);
        [PL_LoS_BS2UE, PL_NLoS_BS2UE] = PathLoss_Vec(link, environment_mode, distance_BS2UE_2D_interferer, distance_BS2UE_interferer, h_BS, [], h_UE_ground, fc);
        PL_BS2UE_interferer = prLoS_BS2UE .* PL_LoS_BS2UE + (1 - prLoS_BS2UE) .* PL_NLoS_BS2UE;
        theta_BS2UE_interferer = pi - acos((h_BS - h_UE_ground) ./ distance_BS2UE_interferer);
        switch fading_mode
            case 'Rayleigh'
                fading_BS2UE_interferer = exprnd(1, [num_UE_victim, 1]);
            case 'Rician'
                K_BS2UE = 10; % Rician K-factor
                nu_BS2UE = sqrt(K_BS2UE / (1 + K_BS2UE));
                sigma_BS2UE = 1 / sqrt(2 * (1 + K_BS2UE));
                X = normrnd(nu_BS2UE * cos(pi / 3), sigma_BS2UE, [num_UE_victim, 1]); % pi / 3 is just a random phase
                Y = normrnd(nu_BS2UE * sin(pi / 3), sigma_BS2UE, [num_UE_victim, 1]);
                fading_BS2UE_interferer = X .^ 2 + Y .^ 2;
        end
        g_BS2UE_interferer = gain_ant(theta_BS2UE_interferer, theta_downtilt_BS, theta_3dB_BS, SLA, g_UE);
        if j == 1
            g_BS2UE_interferer([PR1_UEs; PR2_UEs; false(num_UE_sector, 1)]) = db2pow(G_E_max - SLA); % Middle-right interfereing BS has minimal impact on the upper-right and lower-right victim UEs
        elseif j == 2
            g_BS2UE_interferer([PR1_UEs; false(2 * num_UE_sector, 1)]) = db2pow(G_E_max - SLA); % Upper-right interfereing BS has minimal impact on the upper-right victim UEs
        elseif j == 3
            g_BS2UE_interferer([PR1_UEs; false(num_UE_sector, 1); PR3_UEs]) = db2pow(G_E_max - SLA); % Upper-left interfereing BS has minimal impact on the upper-right and middle-left victim UEs
        elseif j == 4
            g_BS2UE_interferer([false(2 * num_UE_sector, 1); PR3_UEs]) = db2pow(G_E_max - SLA); % Middle-left interfereing BS has minimal impact on the middle-left victim UEs
        elseif j == 5
            g_BS2UE_interferer([false(num_UE_sector, 1); PR2_UEs; PR3_UEs]) = db2pow(G_E_max - SLA); % Lower-left interfereing BS has minimal impact on the lower-right and middle-left victim UEs
        elseif j == 6
            g_BS2UE_interferer([false(num_UE_sector, 1); PR2_UEs; false(num_UE_sector, 1)]) = db2pow(G_E_max - SLA); % Lower-right interfereing BS has minimal impact on the lower-right victim UEs
        end
        power_BS2UE_interferer(:, j) = p_BS * g_BS2UE_interferer ./ PL_BS2UE_interferer .* fading_BS2UE_interferer;
    end
    power_BS2UE_interferer_all = sum(power_BS2UE_interferer, 2);
    pn = pn_FR * is_FR_UE + pn_PR * is_PR_UE;
    SINR_BS2UE = power_BS2UE_desired ./ (power_BS2UE_interferer_all + pn);
    SINR_BS2UE_sorted = SINR_BS2UE(range_UE_victim_sorted_ind);

    BW = BW_FR * is_FR_UE + BW_PR * is_PR_UE;
    BW_sorted = BW(range_UE_victim_sorted_ind);
    tput_BS2UE_sorted = BW_sorted .* log2(1 + SINR_BS2UE_sorted) / 1e6; %[Mbps]

    interference_BS2UE_sorted_mat(:, i) = power_BS2UE_interferer_all(range_UE_victim_sorted_ind);
    SINR_BS2UE_sorted_mat(:, i) = SINR_BS2UE_sorted;
    tput_BS2UE_sorted_mat(:, i) = tput_BS2UE_sorted;
    %}
end
% mean([m1, m2])

%% Smoothing process
for j = 1 : num_points - 1
    temp_interference = [];
    temp_SINR = [];
    temp_tput = [];
    for i = 1 : realizations
        inds = find((range_target(j) < pos_UE_sorted_mat(:, i)) & (range_target(j + 1) >= pos_UE_sorted_mat(:, i)));
        temp_interference = [temp_interference; interference_BS2UE_sorted_mat(inds, i)]; %#ok<*AGROW>
        temp_SINR = [temp_SINR; SINR_BS2UE_sorted_mat(inds, i)];
        temp_tput = [temp_tput; tput_BS2UE_sorted_mat(inds, i)];
    end
    interference_target(j) = pow2db(mean(temp_interference)) + 30; %[dBm]
    interference_target_cell{j} = pow2db(temp_interference) + 30; %[dBm]
    SINR_target(j) = pow2db(mean(temp_SINR));
    SINR_target_cell{j} = pow2db(temp_SINR);
    tput_target(j) = mean(temp_tput);
    tput_target_cell{j} = temp_tput;
end

%% Plots
j_vec = [20, 40, 60];
valid_cdf_bins = find(~cellfun(@isempty, interference_target_cell) & ...
    ~cellfun(@isempty, SINR_target_cell) & ...
    ~cellfun(@isempty, tput_target_cell));
if isempty(valid_cdf_bins)
    error('No non-empty distance bins are available for CDF plotting.')
end
if any(~ismember(j_vec, valid_cdf_bins))
    num_cdf_bins = min(3, numel(valid_cdf_bins));
    j_vec = valid_cdf_bins(round(linspace(1, numel(valid_cdf_bins), num_cdf_bins)));
end
j_vec = j_vec(:).';
cdf_legend_entries = arrayfun(@(idx) ['TN UE Range = ', num2str(range_target(idx)), ' m'], ...
    j_vec, 'UniformOutput', false);

figure(602)
plot(range_target(1 : num_points - 1), interference_target, 'LineWidth', 2, 'LineStyle', ':')
title('Total Interference Power vs. UE Distance to BS', 'Interpreter', 'latex', 'FontSize', 14)
subtitle(['Scenario: 1 tier with 7 cells; Number of UEs per sector: ', num2str(num_UE_sector)], 'Interpreter', 'latex', 'FontSize', 12)
xlabel('UE Distance to BS', 'Interpreter', 'latex', 'FontSize', 11)
ylabel('Interference Power [dBm]', 'Interpreter', 'latex', 'FontSize', 11)
grid on

figure(605)
hold on
for j = j_vec
    h_cdf = cdfplot(interference_target_cell{j}(:));
    h_cdf.LineWidth = 2;
    h_cdf.LineStyle = '-';
    title('CDF of Interference', 'Interpreter', 'latex', 'FontSize', 14)
    subtitle({'1 TN tier with 7 TN cells and 1 NTN cell', ['Number of TN / NTN UEs per sector: ', num2str(num_UE_sector), ' / ', num2str(num_UE_NTN_sector)]}, 'Interpreter', 'latex', 'FontSize', 12)
    xlabel('$p$ [dBm]', 'Interpreter', 'latex', 'FontSize', 11)
    ylabel('Pr$[P_{\rm interference} \leq p]$', 'Interpreter', 'latex', 'FontSize', 11)
end
hh = legend(cdf_legend_entries);
hh.Interpreter = 'latex';
hh.FontSize = 12;
hh.Location = "northwest";
hold off

figure(606)
hold on
for j = j_vec
    h_cdf = cdfplot(SINR_target_cell{j}(:));
    h_cdf.LineWidth = 2;
    h_cdf.LineStyle = '-';
    title('CDF of SINR (Outage Probability)', 'Interpreter', 'latex', 'FontSize', 14)
    subtitle({'1 TN tier with 7 TN cells and 1 NTN cell', ['Number of TN / NTN UEs per sector: ', num2str(num_UE_sector), ' / ', num2str(num_UE_NTN_sector)]}, 'Interpreter', 'latex', 'FontSize', 12)
    xlabel('$\gamma$ [dB]', 'Interpreter', 'latex', 'FontSize', 11)
    ylabel('Outage Probability: Pr$[{\rm SINR} \leq \gamma]$', 'Interpreter', 'latex', 'FontSize', 11)
end
hh = legend(cdf_legend_entries);
hh.Interpreter = 'latex';
hh.FontSize = 12;
hh.Location = "northwest";
hold off

figure(607)
hold on
for j = j_vec
    h_cdf = cdfplot(tput_target_cell{j}(:));
    h_cdf.LineWidth = 2;
    h_cdf.LineStyle = '-';
    title('CDF of Throughput', 'Interpreter', 'latex', 'FontSize', 14)
    subtitle({'1 TN tier with 7 TN cells and 1 NTN cell', ['Number of TN / NTN UEs per sector: ', num2str(num_UE_sector), ' / ', num2str(num_UE_NTN_sector)]}, 'Interpreter', 'latex', 'FontSize', 12)
    xlabel('$r$ [Mbps]', 'Interpreter', 'latex', 'FontSize', 11)
    ylabel('Pr$[{\rm Throughput} \leq r]$', 'Interpreter', 'latex', 'FontSize', 11)
end
hh = legend(cdf_legend_entries);
hh.Interpreter = 'latex';
hh.FontSize = 12;
hh.Location = "southeast";
hold off

%% Save TN-NTN baseline outputs
summary_table = table(range_target(1 : num_points - 1), interference_target, SINR_target, tput_target, ...
    'VariableNames', {'UE_Distance_m', 'Interference_dBm', 'SINR_dB', 'Throughput_Mbps'});
writetable(summary_table, fullfile(results_dir, 'TN_NTN_summary.csv'));

save(fullfile(results_dir, 'TN_NTN_results.mat'), ...
    'fc', 'environment_mode', 'num_UE_sector', 'num_UE_NTN_sector', ...
    'ISD_TN', 'ISD_NTN', 'realizations', 'range_target', ...
    'interference_target', 'SINR_target', 'tput_target', ...
    'interference_target_cell', 'SINR_target_cell', 'tput_target_cell');

fig_ids = [602, 605, 606, 607];
fig_names = {'Interference_vs_Distance', ...
    'CDF_Interference', ...
    'CDF_SINR', ...
    'CDF_Throughput'};
for idx = 1 : numel(fig_ids)
    fig_handle = figure(fig_ids(idx));
    set(fig_handle, 'Color', 'w')
    saveas(fig_handle, fullfile(results_dir, [fig_names{idx}, '.png']));
    savefig(fig_handle, fullfile(results_dir, [fig_names{idx}, '.fig']));
end

disp(['Saved TN-NTN outputs to: ', results_dir])
