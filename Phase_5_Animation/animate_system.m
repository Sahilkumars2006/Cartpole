
%% PHASE 5: 3D ANIMATION
% Inverted Spherical Pendulum on an X-Y Cart
% Uses Cartesian constrained simulation results.

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' 3D INVERTED SPHERICAL PENDULUM ANIMATION\n');
fprintf('============================================\n');

%% 1. Locate project and results

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

resultsFile = fullfile(projectDir, ...
    'Results', 'MATLAB', ...
    'cartesian_simulation_results.mat');

if ~isfile(resultsFile)
    error('Cartesian simulation results not found:\n%s', ...
        resultsFile);
end

%% 2. Load simulation data

loadedData = load(resultsFile, 'results');
results = loadedData.results;

time = results.time;

cart_position = results.cart_position;
bob_position = results.bob_position;

p = results.parameters;

numberOfFrames = length(time);

fprintf('\nSimulation data loaded successfully.\n');
fprintf('Number of frames: %d\n', numberOfFrames);
fprintf('Simulation duration: %.2f seconds\n', time(end));

%% 3. Animation settings

% Cart dimensions
cartLength = 0.50;
cartWidth = 0.35;

% Bob radius
bobRadius = 0.08;

% Animation speed
frameSkip = 2;
playbackSpeed = 1.0;

% Use frameSkip = 1 for every simulation frame.
% playbackSpeed = 1 means approximately real-time playback.

%% 4. Calculate plot limits

allX = [cart_position(:,1); bob_position(:,1)];
allY = [cart_position(:,2); bob_position(:,2)];
allZ = [zeros(numberOfFrames,1); bob_position(:,3)];

margin = 0.35;

xLimits = [min(allX)-margin-cartLength, ...
           max(allX)+margin+cartLength];

yLimits = [min(allY)-margin-cartWidth, ...
           max(allY)+margin+cartWidth];

zLimits = [min(allZ)-margin, ...
           max(allZ)+margin];

%% 5. Create animation figure

fig = figure( ...
    'Name', 'Inverted Spherical Pendulum - 3D Animation', ...
    'Color', 'w', ...
    'Position', [100 100 1100 750]);

ax = axes(fig);

hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

xlabel(ax, 'X Position (m)');
ylabel(ax, 'Y Position (m)');
zlabel(ax, 'Z Position (m)');

title(ax, '3D Motion of Inverted Spherical Pendulum');

xlim(ax, xLimits);
ylim(ax, yLimits);
zlim(ax, zLimits);

view(ax, 35, 25);

%% 6. Draw ground reference plane

[Xground, Yground] = meshgrid( ...
    linspace(xLimits(1), xLimits(2), 2), ...
    linspace(yLimits(1), yLimits(2), 2));

Zground = zeros(size(Xground));

surf(ax, Xground, Yground, Zground, ...
    'FaceColor', [0.90 0.90 0.90], ...
    'FaceAlpha', 0.20, ...
    'EdgeColor', 'none');

%% 7. Draw cart

x = cart_position(1,1);
y = cart_position(1,2);

halfL = cartLength/2;
halfW = cartWidth/2;

cartVertices = [ ...
    x-halfL, y-halfW, 0;
    x+halfL, y-halfW, 0;
    x+halfL, y+halfW, 0;
    x-halfL, y+halfW, 0];

cartHandle = patch(ax, ...
    'Vertices', cartVertices, ...
    'Faces', [1 2 3 4], ...
    'FaceColor', [0.2 0.45 0.85], ...
    'FaceAlpha', 0.85, ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.5);

%% 8. Draw pendulum rod

pivot = [x, y, 0];

bob = bob_position(1,:);

rodHandle = plot3(ax, ...
    [pivot(1), bob(1)], ...
    [pivot(2), bob(2)], ...
    [pivot(3), bob(3)], ...
    'k-', ...
    'LineWidth', 4);

%% 9. Draw bob

[sphereX, sphereY, sphereZ] = sphere(20);

bobHandle = surf(ax, ...
    bobRadius*sphereX + bob(1), ...
    bobRadius*sphereY + bob(2), ...
    bobRadius*sphereZ + bob(3), ...
    'FaceColor', [0.9 0.15 0.1], ...
    'EdgeColor', 'none');

%% 10. Draw trajectory

trajectoryHandle = animatedline(ax, ...
    'Color', [0.9 0.2 0.1], ...
    'LineWidth', 1.5);

%% 11. Draw pivot marker

pivotHandle = plot3(ax, ...
    pivot(1), pivot(2), pivot(3), ...
    'ko', ...
    'MarkerFaceColor', 'k', ...
    'MarkerSize', 6);

%% 12. Lighting and appearance

camlight(ax, 'headlight');
lighting(ax, 'gouraud');

legend(ax, ...
    [cartHandle, rodHandle, bobHandle, trajectoryHandle], ...
    {'X-Y Cart', 'Pendulum Rod', 'Bob', 'Bob Trajectory'}, ...
    'Location', 'northeastoutside');

%% 13. Animation loop

fprintf('\nStarting 3D animation...\n');

for k = 1:frameSkip:numberOfFrames

    % Current cart position
    x = cart_position(k,1);
    y = cart_position(k,2);

    % Current bob position
    bob = bob_position(k,:);

    % Current pivot position
    pivot = [x, y, 0];

    %% Update cart vertices

    cartVertices = [ ...
        x-halfL, y-halfW, 0;
        x+halfL, y-halfW, 0;
        x+halfL, y+halfW, 0;
        x-halfL, y+halfW, 0];

    set(cartHandle, 'Vertices', cartVertices);

    %% Update rod

    set(rodHandle, ...
        'XData', [pivot(1), bob(1)], ...
        'YData', [pivot(2), bob(2)], ...
        'ZData', [pivot(3), bob(3)]);

    %% Update bob

    set(bobHandle, ...
        'XData', bobRadius*sphereX + bob(1), ...
        'YData', bobRadius*sphereY + bob(2), ...
        'ZData', bobRadius*sphereZ + bob(3));

    %% Update pivot

    set(pivotHandle, ...
        'XData', pivot(1), ...
        'YData', pivot(2), ...
        'ZData', pivot(3));

    %% Update trajectory

    addpoints(trajectoryHandle, ...
        bob(1), bob(2), bob(3));

    %% Update title

    title(ax, sprintf( ...
        'Inverted Spherical Pendulum | Time: %.2f s', ...
        time(k)));

    %% Refresh figure

    drawnow;

    %% Playback timing

    if k < numberOfFrames
        nextFrame = min(k+frameSkip, numberOfFrames);

        dt = time(nextFrame)-time(k);

        pause(dt/playbackSpeed);
    end

end

fprintf('\nAnimation completed successfully.\n');

fprintf('Total simulation time: %.2f seconds\n', time(end));

fprintf('\n============================================\n');
fprintf('       ANIMATION FINISHED\n');
fprintf('============================================\n');
