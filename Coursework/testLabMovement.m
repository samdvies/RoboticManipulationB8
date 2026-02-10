function cleanup_obj = testLabMovement()
% TESTLABMOVEMENT Quick check for lab session
%
% Usage: c = testLabMovement(); (Keep 'c' to keep torque ON)
%
% Moves to Home, then [300, 150, 0], then Gripper Open/Close.

    clc;
    fprintf('=== Lab Movement Test ===\n');
    
    % 1. Setup
    setupPath();
    try
        [port_num, lib_name, cleanup_obj] = robotSafeInit('COM4'); % Keep cleanup_obj to prevent auto-close
    catch
        fprintf('Error: Could not connect to robot. Check USB and power.\n');
        return;
    end

    % 2. Move Home (High Safe Position)
    fprintf('Moving to Home [200, 0, 150]...\n');
    moveToPosition(port_num, lib_name, [200, 0, 150], 'horizontal');
    pause(2);

    % 3. Test Reachability (User's Point)
    fprintf('Testing [300, 150, 0] (Should Auto-Elbow-Up)...\n');
    moveToPosition(port_num, lib_name, [300, 150, 0], 'auto');
    pause(2);

    % 4. Gripper Test
    fprintf('Testing Gripper...\n');
    gripperControl(port_num, lib_name, 'close');
    pause(0.5);
    gripperControl(port_num, lib_name, 'open');

    fprintf('=== Test Complete (Torque still ON) ===\n');
end
