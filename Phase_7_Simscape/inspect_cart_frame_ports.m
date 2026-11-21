
%% PHASE 7.7A: INSPECT CART PHYSICAL FRAME PORTS

clear;
clc;

model = 'sm_xy_cart_test';

if ~bdIsLoaded(model)
    load_system(model);
end

blocks = {
    'World'
    'X_Prismatic'
    'Y_Prismatic'
    'Intermediate_Carriage'
    'Main_Cart'
};

fprintf('\n============================================\n');
fprintf(' CART PHYSICAL FRAME PORT INSPECTION\n');
fprintf('============================================\n');

for k = 1:numel(blocks)

    blockPath = [model '/' blocks{k}];

    fprintf('\n--------------------------------------------\n');
    fprintf('BLOCK: %s\n',blocks{k});
    fprintf('--------------------------------------------\n');

    ports = get_param(blockPath,'PortHandles');

    portTypes = {'LConn','RConn'};

    for p = 1:numel(portTypes)

        portType = portTypes{p};

        if ~isfield(ports,portType)
            continue;
        end

        handles = ports.(portType);

        fprintf('\n%s ports: %d\n',portType,numel(handles));

        for j = 1:numel(handles)

            h = handles(j);

            fprintf('\n  Port %d\n',j);
            fprintf('  Handle: %.0f\n',h);

            try
                connectivity = get_param(h,'Line');
                fprintf('  Connected line handle: %.0f\n',connectivity);
            catch
                fprintf('  Line information unavailable.\n');
            end

        end
    end

    try
        connectivity = get_param(blockPath,'PortConnectivity');

        fprintf('\nPort connectivity information:\n');

        for j = 1:numel(connectivity)

            fprintf('  Port %d\n',j);

            if isfield(connectivity(j),'Type')
                fprintf('    Type: %s\n',connectivity(j).Type);
            end

            if isfield(connectivity(j),'Position')
                fprintf('    Position: ');
                disp(connectivity(j).Position);
            end

            if isfield(connectivity(j),'SrcBlock')
                fprintf('    Source block handles: ');
                disp(connectivity(j).SrcBlock);
            end

            if isfield(connectivity(j),'DstBlock')
                fprintf('    Destination block handles: ');
                disp(connectivity(j).DstBlock);
            end

        end

    catch ME
        fprintf('PortConnectivity inspection: %s\n',ME.message);
    end

end

fprintf('\n============================================\n');
fprintf(' FRAME PORT INSPECTION COMPLETED\n');
fprintf('============================================\n');
