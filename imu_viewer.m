% ROB 311 - IMU Data Network Viewer (Newline-Delimited JSON)
% This version matches the actual Python sender format used in test_IMU.py
%
% Usage:
%     1. Download this script to your local machine, and ensure you have MATLAB installed
%     2. Set robot_ip variable below to your robot's IP address
%     3. Run the Python script, imu_realtime.py on the robot and enable network plotting first
%     4. Run this MATLAB script to visualize the IMU data in real-time

clear; clc; close all;

% ========== CONFIGURATION ==========
robot_ip = '67.194.46.111';  % CHANGE THIS TO YOUR ROBOT'S IP

PORT = 5555;
MAX_POINTS = 500;
UPDATE_RATE = 0.01; % 100Hz check rate

% ========== CONNECT TO ROBOT ==========
fprintf('Connecting to robot at %s:%d...\n', robot_ip, PORT);
try
    tcpClient = tcpclient(robot_ip, PORT, 'Timeout', 10);
    configureCallback(tcpClient, "off");
    fprintf('✓ Connected!\n');
catch ME
    error('Connection error: %s', ME.message);
end

% Set terminator for reading lines
configureTerminator(tcpClient, "LF");  % Line feed terminator

% ========== SETUP FIGURE ==========
fig = figure('Name', 'ROB 311 - IMU Test Real-Time Viewer', ...
             'NumberTitle', 'off', ...
             'Position', [100, 100, 1200, 800]);

% Create subplots with animated lines
ax1 = subplot(3, 1, 1);
line1 = animatedline(ax1, 'Color', 'r', 'LineWidth', 2, 'MaximumNumPoints', MAX_POINTS);
ylabel(ax1, 'Roll (θx) [deg]', 'FontSize', 10);
title(ax1, 'IMU Real-Time Data', 'FontSize', 12, 'FontWeight', 'bold');
grid(ax1, 'on');
ylim(ax1, [-90 90]);

ax2 = subplot(3, 1, 2);
line2 = animatedline(ax2, 'Color', 'g', 'LineWidth', 2, 'MaximumNumPoints', MAX_POINTS);
ylabel(ax2, 'Pitch (θy) [deg]', 'FontSize', 10);
grid(ax2, 'on');
ylim(ax2, [-90 90]);

ax3 = subplot(3, 1, 3);
line3 = animatedline(ax3, 'Color', 'b', 'LineWidth', 2, 'MaximumNumPoints', MAX_POINTS);
ylabel(ax3, 'Yaw (θz) [deg]', 'FontSize', 10);
xlabel(ax3, 'Time (s)', 'FontSize', 10);
grid(ax3, 'on');
ylim(ax3, [-90 90]);

fprintf('Viewer started. Waiting for data from robot...\n');
fprintf('(Make sure to enable network plotting when running test_IMU.py)\n');
fprintf('Close the figure window to exit.\n\n');

% ========== MAIN LOOP ==========
point_count = 0;
receiving = true;
last_update_time = tic;

try
    while receiving && isvalid(fig)
        % Check if a complete line is available
        if tcpClient.NumBytesAvailable > 0
            try
                % Read one line (JSON object)
                json_str = readline(tcpClient);
                
                % Parse JSON
                received = jsondecode(json_str);
                
                % Extract data and convert to degrees
                time_val = received.time;
                roll = rad2deg(received.theta_x_rad);
                pitch = rad2deg(received.theta_y_rad);
                yaw = rad2deg(received.theta_z_rad);
                
                % Add points to animated lines
                addpoints(line1, time_val, roll);
                addpoints(line2, time_val, pitch);
                addpoints(line3, time_val, yaw);
                
                point_count = point_count + 1;
                
                % Update title periodically
                if toc(last_update_time) > 0.5  % Every 0.5 seconds
                    title(ax1, sprintf('IMU Real-Time Data (%d points, %.2f Hz)', ...
                          point_count, point_count/time_val), ...
                          'FontSize', 12, 'FontWeight', 'bold');
                    last_update_time = tic;
                end
                
                % Update display
                drawnow limitrate;
                
            catch ME
                fprintf('Error parsing data: %s\n', ME.message);
                if exist('json_str', 'var')
                    fprintf('Received: %s\n', json_str);
                end
            end
        else
            % Small pause to prevent busy-waiting when no data
            pause(UPDATE_RATE);
        end
    end
catch ME
    fprintf('Error in main loop: %s\n', ME.message);
end

% ========== CLEANUP ==========
fprintf('\nClosing viewer...\n');
try
    clear tcpClient;
catch
end
try
    if isvalid(fig)
        close(fig);
    end
catch
end
fprintf('Connection closed\n');