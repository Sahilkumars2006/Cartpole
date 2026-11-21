
%% PHASE 7.7: INSPECT EXISTING X-Y CART MODEL
%
% Identifies the existing cart blocks and frame connections.
% Does not modify the model.

clear;
clc;

model = 'sm_xy_cart_test';
modelFile = 'sm_xy_cart_test.slx';

fprintf('\n============================================\n');
fprintf(' X-Y CART MODEL TOPOLOGY INSPECTION\n');
fprintf('============================================\n');

%% Load model

if ~bdIsLoaded(model)

    if isfile(modelFile)
        load_system(modelFile);
    else
        error('Model file not found: %s',modelFile);
    end

end

%% List blocks

fprintf('\n--- BLOCKS IN MODEL ---\n');

blocks = find_system(model, ...
    'SearchDepth',Inf, ...
    'Type','Block');

for k = 1:numel(blocks)

    block = blocks{k};

    if strcmp(block,model)
        continue;
    end

    try
        blockType = get_param(block,'BlockType');
    catch
        blockType = 'Unknown';
    end

    try
        maskType = get_param(block,'MaskType');
    catch
        maskType = '';
    end

    fprintf('\nBlock: %s\n',block);
    fprintf('  BlockType: %s\n',blockType);

    if ~isempty(maskType)
        fprintf('  MaskType: %s\n',maskType);
    end

end

%% List physical frame and signal connections

fprintf('\n============================================\n');
fprintf(' BLOCK CONNECTIONS\n');
fprintf('============================================\n');

lines = find_system(model, ...
    'FindAll','on', ...
    'Type','line');

for k = 1:numel(lines)

    try

        src = get_param(lines(k),'SrcBlockHandle');
        dst = get_param(lines(k),'DstBlockHandle');

        srcPort = get_param(lines(k),'SrcPortHandle');
        dstPorts = get_param(lines(k),'DstPortHandle');

        if isempty(src) || src == -1
            continue;
        end

        srcName = getfullname(src);

        for j = 1:numel(dst)

            if isempty(dst(j)) || dst(j) == -1
                continue;
            end

            dstName = getfullname(dst(j));

            fprintf('\n%s  -->  %s\n',srcName,dstName);

            if ~isempty(srcPort) && srcPort ~= -1
                try
                    fprintf('  Source port: %s\n', ...
                        get_param(srcPort,'PortNumber'));
                catch
                end
            end

            if ~isempty(dstPorts) && numel(dstPorts) >= j
                try
                    fprintf('  Destination port: %s\n', ...
                        get_param(dstPorts(j),'PortNumber'));
                catch
                end
            end

        end

    catch
        % Ignore lines that do not expose ordinary block handles.
    end

end

%% Show model configuration

fprintf('\n============================================\n');
fprintf(' MODEL INFORMATION\n');
fprintf('============================================\n');

fprintf('Model name: %s\n',model);
fprintf('Model file: %s\n',get_param(model,'FileName'));

fprintf('\nTopology inspection completed.\n');
