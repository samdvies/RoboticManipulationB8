try
    [port_num, lib_name, cleanup] = robotSafeInit('COM3');
    m = libfunctions(lib_name, '-full');
    
    fprintf('Search for SyncWrite:\n');
    for i = 1:length(m)
        if contains(lower(m{i}), 'sync')
            disp(m{i});
        end
    end
    clear cleanup;
catch ME
    disp(ME.message);
end
