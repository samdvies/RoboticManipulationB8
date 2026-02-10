function progressiveMotorTest()
% PROGRESSIVEMOTORTEST Step-by-step debug for non-moving robot
%
% 1. Pings motors
% 2. Reads Error Status
% 3. Enables Torque explicitly
% 4. Wiggles Joint 4 (Wrist) slightly

    clc;
    fprintf('=== Progressive Motor Debug ===\n');
    setupPath();
    
    % Constants
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];
    ADDR_TORQUE_ENABLE = 64;
    ADDR_GOAL_POSITION = 116;
    ADDR_PRESENT_POSITION = 132;
    ADDR_HARDWARE_ERROR = 70;
    
    % 1. Init
    try
        [port_num, lib_name, ~] = robotSafeInit('COM4');
    catch
        fprintf('Init failed.\n'); return;
    end
    
    % 2. Check Errors & Torque
    fprintf('\nStep 2: Checking Motor Status...\n');
    for id = DXL_IDS
        err = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_HARDWARE_ERROR);
        torque = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE);
        pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
        
        fprintf('Motor %d: Pos=%d, Torque=%d, HW_Error=%d\n', id, pos, torque, err);
        
        if err ~= 0
            fprintf('  -> CRITICAL: Resetting motor %d...\n', id);
            % Reboot (not implemented here safely, just warn)
        end
    end
    
    % 3. Force Enable Torque
    fprintf('\nStep 3: Force Enabling Torque...\n');
    for id = DXL_IDS
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 1);
        pause(0.05);
    end
    
    % Verify
    for id = DXL_IDS
        torque = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE);
        if torque ~= 1, fprintf('  FAILED to enable Motor %d!\n', id); end
    end
    
    % 4. Wiggle Wrist (ID 14)
    fprintf('\nStep 4: Wiggling Wrist (ID 14)...\n');
    current_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRESENT_POSITION);
    target_pos = current_pos + 100; % Small move
    
    fprintf('  Moving ID 14 to %d (Current: %d)...\n', target_pos, current_pos);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_GOAL_POSITION, target_pos);
    
    pause(1);
    
    new_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRESENT_POSITION);
    fprintf('  Result ID 14 Pos: %d (Delta: %d)\n', new_pos, abs(new_pos - current_pos));
    
    if abs(new_pos - current_pos) < 10
        fprintf('\nFAIL: Motor did not move.\n');
    else
        fprintf('\nSUCCESS: Motor moved!\n');
    end
end
