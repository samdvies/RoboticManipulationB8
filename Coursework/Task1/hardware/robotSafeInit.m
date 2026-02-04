function [port_num, lib_name, cleanup] = robotSafeInit(com_port)
% ROBOTSAFEINIT Initializes Dynamixel SDK with safe velocity and position limits
%
% Sets up the OpenManipulator-X with:
%   - Profile Velocity = 50 (≈11.5 RPM, safe slow speed)
%   - Position Limits = [1024, 3072] (±90° from home)
%   - Automatic emergency stop on Ctrl+C via onCleanup
%
% DYNAMIXEL ANGLE CONVENTION:
%   Encoder 1024 = 90° (Wizard) = -90° from home
%   Encoder 2048 = 180° (Wizard) = 0° (home)
%   Encoder 3072 = 270° (Wizard) = +90° from home
%
% Usage:
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3')
%
% Inputs:
%   com_port - Serial port name (e.g., 'COM3', 'COM4')
%
% Outputs:
%   port_num - Port handler for Dynamixel SDK
%   lib_name - Library name for calllib functions
%   cleanup  - onCleanup object (keep in scope to maintain auto-stop)
%
% IMPORTANT: Keep 'cleanup' variable in your workspace! If it goes out of
%            scope, the emergency stop will trigger automatically.
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Configuration Constants
    BAUDRATE = 1000000;
    PROTOCOL_VERSION = 2.0;
    
    % Motor IDs
    DXL_IDS = [11, 12, 13, 14, 15];  % Joints 1-4 + Gripper
    
    % Control Table Addresses (XM430-W350-T)
    ADDR_TORQUE_ENABLE      = 64;
    ADDR_PROFILE_VELOCITY   = 112;
    ADDR_MIN_POSITION_LIMIT = 52;
    ADDR_MAX_POSITION_LIMIT = 48;
    
    % Safety Settings
    SAFE_VELOCITY = 50;        % ≈11.5 RPM (0.229 RPM per unit)
    MIN_POSITION  = 1024;      % -90° from home (90° in Wizard)
    MAX_POSITION  = 3072;      % +90° from home (270° in Wizard)
    
    %% Load Dynamixel SDK Library
    lib_name = '';
    if strcmp(computer, 'PCWIN64')
        lib_name = 'dxl_x64_c';
    elseif strcmp(computer, 'GLNXA64')
        lib_name = 'libdxl_x64_c';
    elseif strcmp(computer, 'MACI64')
        lib_name = 'libdxl_mac_c';
    end
    
    if isempty(lib_name)
        error('Unsupported platform: %s', computer);
    end
    
    if ~libisloaded(lib_name)
        fprintf('Loading Dynamixel SDK library: %s\n', lib_name);
        [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
            'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
        
        if ~isempty(notfound)
            warning('Some functions not found: %s', strjoin(notfound, ', '));
        end
    else
        fprintf('Dynamixel SDK library already loaded\n');
    end
    
    %% Initialize Port
    fprintf('Opening port: %s at %d baud\n', com_port, BAUDRATE);
    
    port_num = calllib(lib_name, 'portHandler', com_port);
    calllib(lib_name, 'packetHandler');
    
    if ~calllib(lib_name, 'openPort', port_num)
        error('Failed to open port %s. Check connection and COM port number.', com_port);
    end
    
    if ~calllib(lib_name, 'setBaudRate', port_num, BAUDRATE)
        error('Failed to set baud rate to %d', BAUDRATE);
    end
    
    fprintf('Port opened successfully!\n');
    
    %% Setup Automatic Emergency Stop
    cleanup = onCleanup(@() cleanupFunction(port_num, lib_name));
    fprintf('Emergency stop configured (Ctrl+C will disable all motors)\n');
    
    %% Configure Each Motor
    fprintf('\n--- Configuring Motors with Safe Settings ---\n');
    fprintf('Profile Velocity: %d (≈%.1f RPM)\n', SAFE_VELOCITY, SAFE_VELOCITY * 0.229);
    fprintf('Position Limits: [%d, %d] (±90° from home)\n', MIN_POSITION, MAX_POSITION);
    fprintf('----------------------------------------------\n\n');
    
    for i = 1:length(DXL_IDS)
        id = DXL_IDS(i);
        
        % First, ensure torque is disabled (required to change limits)
        calllib(lib_name, 'write1ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);
        pause(0.05);
        
        % Set position limits
        calllib(lib_name, 'write4ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_MIN_POSITION_LIMIT, MIN_POSITION);
        calllib(lib_name, 'write4ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_MAX_POSITION_LIMIT, MAX_POSITION);
        
        % Set profile velocity (safe slow speed)
        calllib(lib_name, 'write4ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY, SAFE_VELOCITY);
        
        % Check for communication errors
        dxl_comm_result = calllib(lib_name, 'getLastTxRxResult', port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= 0
            warning('Motor ID %d: Communication error (code %d)', id, dxl_comm_result);
        else
            fprintf('Motor ID %d: Configured successfully\n', id);
        end
    end
    
    fprintf('\n=== Initialization Complete ===\n');
    fprintf('Motors are configured but torque is DISABLED.\n');
    fprintf('Use enableMotor() to enable individual motors.\n');
    fprintf('Press Ctrl+C at any time for emergency stop.\n\n');
end

%% Cleanup Function (called automatically on Ctrl+C or error)
function cleanupFunction(port_num, lib_name)
    fprintf('\n>>> Cleanup triggered - Disabling all motors...\n');
    
    DXL_IDS = [11, 12, 13, 14, 15];
    ADDR_TORQUE_ENABLE = 64;
    PROTOCOL_VERSION = 2.0;
    
    for id = DXL_IDS
        try
            calllib(lib_name, 'write1ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);
        catch
            % Ignore errors during cleanup
        end
    end
    
    % Close port
    try
        calllib(lib_name, 'closePort', port_num);
        fprintf('>>> Port closed\n');
    catch
        % Ignore
    end
    
    fprintf('>>> All motors disabled - Safe to handle robot\n');
end
