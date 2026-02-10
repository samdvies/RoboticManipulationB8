function verifyElbowDirection()
% VERIFYELBOWDIRECTION Tests if the software inversion logic correctly maps +Angle to UP for ELBOW
%
% Usage: verifyElbowDirection()
%
% 1. Moves to Home (All 2048)
% 2. Calculates encoder value for +30 degrees using angleConversion() for Joint 3
% 3. Moves Elbow (ID 13) to that calculated value
% 4. Asks user to confirm UP movement.

    clc;
    setupPath();
    [port_num, lib_name, cleanup_obj] = robotSafeInit('COM4');
    
    % 0. Enable Torque
    fprintf('Enabling Torque...\n');
    for id = 11:14
        write1ByteTxRx(port_num, 2.0, id, 64, 1);
        pause(0.05);
    end

    % 1. Home
    fprintf('Moving to Home (All 2048)...\n');
    write4ByteTxRx(port_num, 2.0, 11, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 12, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 13, 116, 2048);
    write4ByteTxRx(port_num, 2.0, 14, 116, 2048);
    pause(2);
    
    % 2. Calculate Target for +30 degrees for JOINT 3 (Elbow)
    target_angle_deg = 30;
    target_angle_rad = deg2rad(target_angle_deg);
    
    % Use the conversion function (Elbow is ID 13 / Joint 3)
    target_encoder = angleConversion('rad2enc', target_angle_rad, 3);
    
    fprintf('\nTESTING ELBOW FIX:\n');
    fprintf('Target Angle: +%.1f degrees (Should Bend UP/IN)\n', target_angle_deg);
    fprintf('Calculated Encoder Value: %d\n', target_encoder);
    
    input('Press ENTER to move...', 's');
    
    write4ByteTxRx(port_num, 2.0, 13, 116, target_encoder);
    pause(1);
    
    fprintf('\n------------------------------------------------\n');
    if target_encoder < 2048
        fprintf('Calculated encoder was LOWER (%d).\n', target_encoder);
    else
        fprintf('Calculated encoder was HIGHER (%d).\n', target_encoder);
    end
    fprintf('DID THE ELBOW MOVE UP (Flex) OR DOWN (Extend)?\n');
    fprintf('If it moved UP, the Inversion is CORRECT.\n');
    fprintf('If it moved DOWN (towards table), the Inversion is WRONG.\n');
    fprintf('------------------------------------------------\n');
end
