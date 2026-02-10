function verifyMotorDirection()
% VERIFYMOTORDIRECTION Tests if the software inversion logic correctly maps +Angle to UP
%
% Usage: verifyMotorDirection()
%
% 1. Moves to Home (All Zero / L-Shape)
% 2. Calculates encoder value for +30 degrees using angleConversion()
% 3. Moves Shoulder (ID 12) to that calculated value
% 4. Asks user to confirm UP movement.

    clc;
    setupPath();
    [port_num, lib_name, cleanup_obj] = robotSafeInit('COM4');
    
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
    
    % 2. Calculate Target for +30 degrees
    target_angle_deg = 30;
    target_angle_rad = deg2rad(target_angle_deg);
    
    % Use the conversion function (Shoulder is ID 12 / Joint 2)
    % This handles the inversion internally!
    target_encoder = angleConversion('rad2enc', target_angle_rad, 2);
    
    fprintf('\nTESTING FIX:\n');
    fprintf('Target Angle: +%.1f degrees (UP)\n', target_angle_deg);
    fprintf('Calculated Encoder Value: %d (Expect < 2048)\n', target_encoder);
    
    input('Press ENTER to move...', 's');
    
    write4ByteTxRx(port_num, 2.0, 12, 116, target_encoder);
    pause(1);
    
    fprintf('\n------------------------------------------------\n');
    if target_encoder < 2048
        fprintf('SUCCESS: The code calculated a LOWER encoder value (%d).\n', target_encoder);
        fprintf('Since +Encoder moves DOWN, a Lower Encoder should move UP.\n');
        fprintf('Please confirm visually.\n');
    else
        fprintf('FAILURE: The code calculated a HIGHER encoder value (%d).\n', target_encoder);
        fprintf('This will likely move DOWN.\n');
    end
    fprintf('------------------------------------------------\n');
end
