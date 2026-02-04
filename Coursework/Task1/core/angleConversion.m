function varargout = angleConversion(varargin)
% ANGLECONVERSION Converts between code angles (radians), degrees, and Dynamixel encoder values
%
% DYNAMIXEL XM430 CONVENTION:
%   Encoder 0    = 0° (Wizard)    = -180° from home
%   Encoder 1024 = 90° (Wizard)   = -90° from home
%   Encoder 2048 = 180° (Wizard)  = 0° (HOME)
%   Encoder 3072 = 270° (Wizard)  = +90° from home
%   Encoder 4095 = ~360° (Wizard) = +180° from home
%
% Operating Range (±90°):
%   Code: [-π/2, +π/2] radians
%   Encoder: [1024, 3072]
%
% USAGE MODES:
%
% 1. Convert radians to encoder:
%    encoder = angleConversion('rad2enc', radians)
%
% 2. Convert encoder to radians:
%    radians = angleConversion('enc2rad', encoder)
%
% 3. Convert degrees (from home) to encoder:
%    encoder = angleConversion('deg2enc', degrees)
%
% 4. Convert encoder to degrees (from home):
%    degrees = angleConversion('enc2deg', encoder)
%
% 5. Display conversion table:
%    angleConversion('table')
%
% Examples:
%    enc = angleConversion('rad2enc', pi/4)    % 45° → 2560
%    rad = angleConversion('enc2rad', 1024)    % → -π/2
%    angleConversion('table')                   % Print reference table
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    if nargin < 1
        printConversionTable();
        return;
    end
    
    mode = varargin{1};
    
    switch lower(mode)
        case 'rad2enc'
            % Radians (0 = home) to Encoder (2048 = home)
            rad = varargin{2};
            deg = rad2deg(rad);
            encoder = round((deg / 360) * 4096) + 2048;
            encoder = max(0, min(4095, encoder));  % Clamp to valid range
            varargout{1} = encoder;
            
        case 'enc2rad'
            % Encoder to Radians (0 = home)
            encoder = varargin{2};
            deg = ((encoder - 2048) / 4096) * 360;
            varargout{1} = deg2rad(deg);
            
        case 'deg2enc'
            % Degrees from home to Encoder
            deg = varargin{2};
            encoder = round((deg / 360) * 4096) + 2048;
            encoder = max(0, min(4095, encoder));
            varargout{1} = encoder;
            
        case 'enc2deg'
            % Encoder to Degrees from home
            encoder = varargin{2};
            deg = ((encoder - 2048) / 4096) * 360;
            varargout{1} = deg;
            
        case 'table'
            printConversionTable();
            
        otherwise
            error('Unknown mode: %s. Use ''rad2enc'', ''enc2rad'', ''deg2enc'', ''enc2deg'', or ''table''.', mode);
    end
end

function printConversionTable()
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║         DYNAMIXEL ANGLE CONVERSION REFERENCE                 ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Code (rad)  │  Code (deg)  │  Encoder  │  Wizard Display   ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║    -π/2      │    -90°      │   1024    │      90°          ║\n');
    fprintf('║    -π/4      │    -45°      │   1536    │     135°          ║\n');
    fprintf('║     0        │      0° ←HOME│   2048    │     180° ←HOME    ║\n');
    fprintf('║    +π/4      │    +45°      │   2560    │     225°          ║\n');
    fprintf('║    +π/2      │    +90°      │   3072    │     270°          ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  OPERATING RANGE: Encoder [1024, 3072] = ±90° from home     ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Conversion: encoder = (deg/360)*4096 + 2048                ║\n');
    fprintf('║              deg = (encoder - 2048)/4096 * 360              ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');
end
