function quit_requested = checkForQuit()
% CHECKFORQUIT Checks if user has pressed 'q' or 'Q' to request emergency stop
%
% Non-blocking keyboard check for use in motion control loops.
%
% Usage:
%   if checkForQuit()
%       emergencyStop(port_num, lib_name);
%       return;
%   end
%
% Output:
%   quit_requested - true if 'q' or 'Q' was pressed, false otherwise
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    quit_requested = false;
    
    % Check if a key has been pressed (non-blocking)
    if ~isempty(get(gcf, 'CurrentCharacter'))
        key = get(gcf, 'CurrentCharacter');
        if strcmpi(key, 'q')
            quit_requested = true;
            % Clear the key buffer
            set(gcf, 'CurrentCharacter', char(0));
        end
    end
end
