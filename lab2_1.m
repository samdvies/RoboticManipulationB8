% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID                      = 11;            % Dynamixel ID: 1
DXL2_ID                      = 13;            % Dynamixel ID: 1

BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 40;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MIN_POS, MIN_POS);
% ---------------------------------- %

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_OPERATING_MODE, 3);


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

 % Read present positions
dxl1_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_PRESENT_POSITION);
fprintf('[ID:%03d] Position (1) : %03d\n', DXL1_ID, typecast(uint32(dxl1_present_position), 'int32'));

dxl2_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_PRESENT_POSITION);
fprintf('[ID:%03d] Position (2) : %03d\n', DXL2_ID, typecast(uint32(dxl2_present_position), 'int32'));

%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_GOAL_POSITION, 2047);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_GOAL_POSITION, 1028);
pause(1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_GOAL_POSITION, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_GOAL_POSITION, 0);
pause(1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_GOAL_POSITION, 1028);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_GOAL_POSITION, 2047);
pause(1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_GOAL_POSITION, 3000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_GOAL_POSITION, 600);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

i = 0;


    j = 0;
    while (j<200)
        j = j+1;
        
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] Position: %03d\n', DXL_ID, typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end
i = 0;
rTotal = 0;
wTotal = 0; 
while (i<100)
    % --- Write Operation ---
    wTstart = tic; 
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, 2047);
    wTime = toc(wTstart); % wTime now holds the duration in seconds
    wTotal = wTotal+wTime;

    % --- Read Operation ---
    rTstart = tic;
    dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
    rTime = toc(rTstart); % rTime now holds the duration in seconds
    rTotal = rTotal+rTime;
    i=i+1;

end
cycle_count = 0;

while cycle_count < num_cycles
    for i = 1:length(goals)
        target_pos = goals(i);
        
        % Write Goal Position
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, target_pos);
        fprintf('Moving to Position: %d\n', target_pos);

        % --- Position Detection Loop ---
        while true
            % Read current position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
            
            % Handle potential signed value conversion
            cur_pos = typecast(uint32(dxl_present_position), 'int32');
            
            % Check if motor is within the threshold of the target
            if abs(target_pos - cur_pos) < DXL_MOVING_STATUS_THRESHOLD
                fprintf('Target Reached: %d\n', cur_pos);
                break; 
            end
            
            pause(0.01); % Small delay to prevent flooding the bus
        end
    end
    cycle_count = cycle_count + 1;
    fprintf('Cycle %d completed\n', cycle_count);
end

% --- Display Results ---
fprintf('Write Time(avg): %f s | Read Time(avg): %f s\n', wTotal/100, rTotal/100);
% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
