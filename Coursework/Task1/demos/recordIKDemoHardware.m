function recordIKDemoHardware(port_num, lib_name)
% RECORDIKDEMOHARDWARE Record IK demo video on real robot
%
% Moves the robot through multiple target positions while recording.
% Creates a video showing the robot tracking XYZ targets.
%
% Usage:
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3');
%   recordIKDemoHardware(port_num, lib_name);
%
% Author: OpenManipulator-X Task 1
% Date: February 2026

fprintf('=============================================\n');
fprintf('   IK HARDWARE DEMO RECORDING\n');
fprintf('=============================================\n\n');

%% Configuration
SPEED = 20;  % Movement speed
PAUSE_TIME = 2;  % Seconds to hold at each position

% Demo positions - ONLY VERIFIED WORKING positions
targets = [
    330,    0,  200;   % Center, safe
    330,   30,  210;   % Left (worked: 7.3mm)
    330,  -30,  210;   % Right (worked: 7.6mm)
    320,    0,  210;   % Closer center
    320,   25,  200;   % Closer left
    320,  -25,  200;   % Closer right
    330,    0,  200;   % Return center
];

num_targets = size(targets, 1);

%% Setup Video Recording
video_filename = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'IK_Hardware_Demo.mp4');
fprintf('Video will be saved to:\n  %s\n\n', video_filename);

% Create figure for info display
fig = figure('Name', 'IK Hardware Demo', 'Position', [100 100 800 600], 'Color', 'w');

%% Countdown
fprintf('Starting in 3 seconds...\n');
fprintf('>>> POSITION YOUR CAMERA NOW <<<\n\n');
for i = 3:-1:1
    clf(fig);
    annotation('textbox', [0.1, 0.3, 0.8, 0.4], ...
        'String', {sprintf('Starting in %d...', i), '', 'Position your camera!'}, ...
        'FontSize', 36, 'HorizontalAlignment', 'center', ...
        'EdgeColor', 'none');
    drawnow;
    pause(1);
end

%% Move to Home First
fprintf('Moving to home position...\n');
clf(fig);
showStatus(fig, 'Moving to HOME', [0, 0, 0], 0, num_targets);
drawnow;

PROTOCOL_VERSION = 2.0;
DXL_IDS = [11, 12, 13, 14];
for id = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, 64, 1);  % Enable torque
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, 116, 2048);  % Home
end
pause(2);

%% Execute Demo Sequence
fprintf('\n--- Beginning Demo Sequence ---\n');
fprintf('Move your camera to capture the robot!\n\n');

for i = 1:num_targets
    target = targets(i, :);
    
    fprintf('Target %d/%d: [%.0f, %.0f, %.0f] mm\n', i, num_targets, target);
    
    % Update display
    clf(fig);
    showStatus(fig, 'MOVING', target, i, num_targets);
    drawnow;
    
    % Move to target
    [q, success, info] = inverseKinematicsAuto(target);
    
    if success
        % Convert to encoder
        encoders = zeros(1, 4);
        for j = 1:4
            encoders(j) = angleConversion('rad2enc', q(j));
        end
        
        % Set speed
        for j = 1:4
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(j), 112, SPEED);
        end
        
        % Move SEQUENTIALLY - one motor at a time with wait
        ADDR_MOVING = 122;
        for j = 1:4
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(j), 116, encoders(j));
            % Wait for this motor to stop
            motor_timeout = tic;
            while toc(motor_timeout) < 5
                moving = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(j), ADDR_MOVING);
                if moving == 0
                    pause(0.1);
                    break;
                end
                pause(0.05);
            end
        end
        
        % Brief settle time
        pause(0.5);
        
        % Read actual position
        final_enc = zeros(1, 4);
        for j = 1:4
            final_enc(j) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(j), 132);
        end
        final_q = zeros(1, 4);
        for j = 1:4
            final_q(j) = angleConversion('enc2rad', final_enc(j));
        end
        [~, actual_pos, ~] = forwardKinematics(final_q);
        
        error_mm = norm(actual_pos' - target);
        fprintf('  Reached! Error: %.1f mm\n', error_mm);
        
        % Update display with result
        clf(fig);
        showResult(fig, target, actual_pos', error_mm, i, num_targets);
        drawnow;
        
    else
        fprintf('  IK FAILED: %s\n', info.message);
        clf(fig);
        annotation('textbox', [0.1, 0.3, 0.8, 0.4], ...
            'String', {sprintf('Target %d UNREACHABLE', i), info.message}, ...
            'FontSize', 24, 'HorizontalAlignment', 'center', ...
            'EdgeColor', 'none', 'Color', 'r');
        drawnow;
    end
    
    % Hold at position
    pause(PAUSE_TIME);
end

%% Return Home
fprintf('\nReturning to home...\n');
clf(fig);
showStatus(fig, 'Returning HOME', [0, 0, 0], num_targets, num_targets);
drawnow;

for id = DXL_IDS
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, 116, 2048);
end
pause(2);

%% Complete
clf(fig);
annotation('textbox', [0.1, 0.3, 0.8, 0.4], ...
    'String', {'DEMO COMPLETE!', '', 'Stop your camera recording.', ...
               sprintf('%d positions visited', num_targets)}, ...
    'FontSize', 24, 'HorizontalAlignment', 'center', ...
    'EdgeColor', 'none', 'BackgroundColor', [0.8, 1, 0.8]);
drawnow;

fprintf('\n=============================================\n');
fprintf('   IK HARDWARE DEMO COMPLETE\n');
fprintf('=============================================\n');
fprintf('Save your camera recording as IK_Hardware_Demo.mp4\n');
fprintf('in: Task1/results/\n\n');

pause(3);
close(fig);

end

%% Helper: Show Status
function showStatus(fig, status, target, current, total)
    annotation('textbox', [0.1, 0.5, 0.8, 0.4], ...
        'String', {status, '', sprintf('Target: [%.0f, %.0f, %.0f] mm', target), ...
                   sprintf('Position %d of %d', current, total)}, ...
        'FontSize', 24, 'HorizontalAlignment', 'center', ...
        'EdgeColor', 'none');
end

%% Helper: Show Result
function showResult(fig, target, actual, error, current, total)
    annotation('textbox', [0.1, 0.4, 0.8, 0.5], ...
        'String', {sprintf('Position %d of %d', current, total), '', ...
                   sprintf('Target:  [%.0f, %.0f, %.0f] mm', target), ...
                   sprintf('Actual:  [%.0f, %.0f, %.0f] mm', actual), ...
                   sprintf('Error:   %.1f mm', error)}, ...
        'FontSize', 20, 'HorizontalAlignment', 'center', ...
        'EdgeColor', 'none', 'BackgroundColor', [0.9, 1, 0.9]);
end
