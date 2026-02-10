function debugJointDirection()
% DEBUGJOINTDIRECTION Checks if +Angle moves the arm UP or DOWN
%
% Usage: debugJointDirection()
%
% 1. Moves to Home (All Zero / L-Shape)
% 2. Moves Shoulder (ID 12) by +30 degrees (Encoder +341)
% 3. Asks user what happened.

    clc;
    setupPath();
    [port_num, lib_name, cleanup_obj] = robotSafeInit('COM4');
    
    % 0. Enable Torque
    fprintf('Enabling Torque...\n');
    for id = 11:14
        write1ByteTxRx(port_num, 2.0, id, 64, 1); % ADDR_TORQUE_ENABLE = 64
        pause(0.05);
    end

    % 0. Enable Torque
    fprintf('Enabling Torque...\n');
    for id = 11:14
        write1ByteTxRx(port_num, 2.0, id, 64, 1); % ADDR_TORQUE_ENABLE = 64
        pause(0.05);
    end

    % 1. Home
    fprintf('Moving to Home (All 2048)...\n');
    write4ByteTxRx(port_num, 2.0, 11, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 12, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 13, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 14, 116, 2048);
    pause(2);
    
    % 2. Move Shoulder Positive
    fprintf('\nTEST: Moving Shoulder (ID 12) by +45 degrees (Encoder 2560)...\n');
    fprintf('WATCH CAREFULLY: Does the arm move UP or DOWN?\n');
    input('Press ENTER to move...', 's');
    
    write4ByteTxRx(port_num, 2.0, 12, 116, 2560); % 2048 + 512
    pause(1);
    
    fprintf('\nIf it moved DOWN, the axis is INVERTED.\n');
    fprintf('If it moved BACK/UP, the axis is CORRECT.\n');
end
