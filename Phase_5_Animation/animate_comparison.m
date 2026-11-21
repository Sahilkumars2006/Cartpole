
%% PHASE 5: CONTROLLED VS UNCONTROLLED COMPARISON
% 3D animation of an inverted spherical pendulum
%
% LEFT:   Uncontrolled Cartesian simulation
% RIGHT:  LQR-controlled Cartesian simulation
%
% BOTTOM LEFT:  Pendulum inclination comparison
% BOTTOM RIGHT: LQR control forces
%
% OUTPUT:
% Inverted_Spherical_Pendulum_LQR_Comparison.mp4

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' CONTROLLED VS UNCONTROLLED ANIMATION\n');
fprintf('============================================\n');

%% 1. LOCATE PROJECT

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

resultsDir = fullfile(projectDir,'Results','MATLAB');

uncontrolledFile = fullfile(resultsDir, ...
    'cartesian_simulation_results.mat');

controlledFile = fullfile(resultsDir, ...
    'controlled_simulation_results.mat');

%% 2. CHECK RESULT FILES

if ~isfile(uncontrolledFile)
    error('Uncontrolled simulation results not found: %s', ...
        uncontrolledFile);
end

if ~isfile(controlledFile)
    error('Controlled simulation results not found: %s', ...
        controlledFile);
end

%% 3. LOAD SIMULATION RESULTS

data1 = load(uncontrolledFile,'results');
data2 = load(controlledFile,'results');

uncontrolled = data1.results;
controlled = data2.results;

fprintf('\nBoth simulation files loaded successfully.\n');

% Check required fields
requiredU = {'time','cart_position','bob_position','parameters'};
requiredC = {'time','cart_position','bob_position', ...
             'control_force','parameters'};

for k = 1:numel(requiredU)
    if ~isfield(uncontrolled,requiredU{k})
        error('Missing field in uncontrolled results: %s', ...
            requiredU{k});
    end
end

for k = 1:numel(requiredC)
    if ~isfield(controlled,requiredC{k})
        error('Missing field in controlled results: %s', ...
            requiredC{k});
    end
end

%% 4. PREPARE COMMON TIME VECTOR

time = controlled.time(:);

if numel(time) < 2 || any(diff(time) <= 0)
    error('Controlled simulation time must be strictly increasing.');
end

if time(end) > uncontrolled.time(end) + 1e-8
    error(['Uncontrolled simulation is shorter than controlled ' ...
           'simulation.']);
end

if time(1) < uncontrolled.time(1) - 1e-8
    error('Simulation time ranges do not overlap.');
end

%% 5. INTERPOLATE UNCONTROLLED DATA

cartU = interp1( ...
    uncontrolled.time, ...
    uncontrolled.cart_position, ...
    time,'linear');

bobU = interp1( ...
    uncontrolled.time, ...
    uncontrolled.bob_position, ...
    time,'linear');

% Controlled data already uses the common time vector
cartC = controlled.cart_position;
bobC = controlled.bob_position;

forceC = controlled.control_force;

numberOfFrames = numel(time);

%% 6. CALCULATE PENDULUM INCLINATION

L = controlled.parameters.L;

relativeU = bobU - ...
    [cartU,zeros(numberOfFrames,1)];

relativeC = bobC - ...
    [cartC,zeros(numberOfFrames,1)];

thetaU = acos(max(-1,min(1,relativeU(:,3)/L)));

thetaC = acos(max(-1,min(1,relativeC(:,3)/L)));

thetaU_deg = rad2deg(thetaU);
thetaC_deg = rad2deg(thetaC);

%% 7. ANIMATION SETTINGS

p = controlled.parameters;

cartLength = 0.50;
cartWidth = 0.35;
bobRadius = 0.08;

frameSkip = 2;
playbackSpeed = 1.0;

if frameSkip < 1 || frameSkip ~= round(frameSkip)
    error('frameSkip must be a positive integer.');
end

if playbackSpeed <= 0
    error('playbackSpeed must be positive.');
end

% Select frames for animation and video
frameIndices = 1:frameSkip:numberOfFrames;

% Always include final frame
if frameIndices(end) ~= numberOfFrames
    frameIndices = [frameIndices,numberOfFrames];
end

totalVideoFrames = numel(frameIndices);

%% 8. CALCULATE COMMON 3D PLOT LIMITS

allX = [cartU(:,1);bobU(:,1);cartC(:,1);bobC(:,1)];
allY = [cartU(:,2);bobU(:,2);cartC(:,2);bobC(:,2)];
allZ = [bobU(:,3);bobC(:,3)];

margin = 0.35;

xLimits = [min(allX)-cartLength-margin, ...
           max(allX)+cartLength+margin];

yLimits = [min(allY)-cartWidth-margin, ...
           max(allY)+cartWidth+margin];

zLimits = [min(-0.15,min(allZ)-margin), ...
           max(1.15,max(allZ)+margin)];

%% 9. CREATE FIGURE

fig = figure( ...
    'Name','Inverted Spherical Pendulum Comparison', ...
    'Color','w', ...
    'Units','pixels', ...
    'Position',[50 50 1450 900], ...
    'NumberTitle','off', ...
    'Resize','off');

layout = tiledlayout(fig,2,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

sgtitle(layout, ...
    'Inverted Spherical Pendulum: LQR Stabilization', ...
    'FontSize',16, ...
    'FontWeight','bold');

%% 10. CREATE UNCONTROLLED 3D AXES

axU = nexttile(layout,1);

hold(axU,'on');
grid(axU,'on');
box(axU,'on');
axis(axU,'equal');

xlabel(axU,'X (m)');
ylabel(axU,'Y (m)');
zlabel(axU,'Z (m)');

xlim(axU,xLimits);
ylim(axU,yLimits);
zlim(axU,zLimits);

view(axU,35,25);

title(axU,'UNCONTROLLED');

%% 11. CREATE CONTROLLED 3D AXES

axC = nexttile(layout,2);

hold(axC,'on');
grid(axC,'on');
box(axC,'on');
axis(axC,'equal');

xlabel(axC,'X (m)');
ylabel(axC,'Y (m)');
zlabel(axC,'Z (m)');

xlim(axC,xLimits);
ylim(axC,yLimits);
zlim(axC,zLimits);

view(axC,35,25);

title(axC,'LQR CONTROLLED');

%% 12. DRAW GROUND PLANES

[Xg,Yg] = meshgrid( ...
    linspace(xLimits(1),xLimits(2),2), ...
    linspace(yLimits(1),yLimits(2),2));

Zg = zeros(size(Xg));

surf(axU,Xg,Yg,Zg, ...
    'FaceColor',[0.85 0.85 0.85], ...
    'FaceAlpha',0.20, ...
    'EdgeColor','none');

surf(axC,Xg,Yg,Zg, ...
    'FaceColor',[0.85 0.85 0.85], ...
    'FaceAlpha',0.20, ...
    'EdgeColor','none');

%% 13. PREPARE CART GEOMETRY

halfL = cartLength/2;
halfW = cartWidth/2;

%% 14. INITIALIZE UNCONTROLLED CART

x = cartU(1,1);
y = cartU(1,2);

vertices = [ ...
    x-halfL,y-halfW,0;
    x+halfL,y-halfW,0;
    x+halfL,y+halfW,0;
    x-halfL,y+halfW,0];

cartHandleU = patch(axU, ...
    'Vertices',vertices, ...
    'Faces',[1 2 3 4], ...
    'FaceColor',[0.85 0.35 0.15], ...
    'EdgeColor','k', ...
    'LineWidth',1.5);

%% 15. INITIALIZE CONTROLLED CART

x = cartC(1,1);
y = cartC(1,2);

vertices = [ ...
    x-halfL,y-halfW,0;
    x+halfL,y-halfW,0;
    x+halfL,y+halfW,0;
    x-halfL,y+halfW,0];

cartHandleC = patch(axC, ...
    'Vertices',vertices, ...
    'Faces',[1 2 3 4], ...
    'FaceColor',[0.15 0.55 0.85], ...
    'EdgeColor','k', ...
    'LineWidth',1.5);

%% 16. CREATE BOB SPHERE

[sphereX,sphereY,sphereZ] = sphere(20);

%% 17. INITIALIZE UNCONTROLLED ROD AND BOB

pivotU = [cartU(1,:),0];
bob = bobU(1,:);

rodU = plot3(axU, ...
    [pivotU(1),bob(1)], ...
    [pivotU(2),bob(2)], ...
    [0,bob(3)], ...
    'k-', ...
    'LineWidth',4);

bobHandleU = surf(axU, ...
    bobRadius*sphereX+bob(1), ...
    bobRadius*sphereY+bob(2), ...
    bobRadius*sphereZ+bob(3), ...
    'FaceColor',[0.9 0.15 0.1], ...
    'EdgeColor','none');

trajectoryU = animatedline(axU, ...
    'Color',[0.9 0.15 0.1], ...
    'LineWidth',1.5);

%% 18. INITIALIZE CONTROLLED ROD AND BOB

pivotC = [cartC(1,:),0];
bob = bobC(1,:);

rodC = plot3(axC, ...
    [pivotC(1),bob(1)], ...
    [pivotC(2),bob(2)], ...
    [0,bob(3)], ...
    'k-', ...
    'LineWidth',4);

bobHandleC = surf(axC, ...
    bobRadius*sphereX+bob(1), ...
    bobRadius*sphereY+bob(2), ...
    bobRadius*sphereZ+bob(3), ...
    'FaceColor',[0.1 0.75 0.25], ...
    'EdgeColor','none');

trajectoryC = animatedline(axC, ...
    'Color',[0.1 0.65 0.2], ...
    'LineWidth',1.5);

%% 19. ADD LEGENDS

legend(axU, ...
    [cartHandleU,rodU,bobHandleU], ...
    {'Cart','Rod','Bob'}, ...
    'Location','best');

legend(axC, ...
    [cartHandleC,rodC,bobHandleC], ...
    {'Cart','Rod','Bob'}, ...
    'Location','best');

%% 20. INITIALIZE DUAL-AXIS INCLINATION PLOT

axAngle = nexttile(layout,3);

hold(axAngle,'on');
grid(axAngle,'on');
box(axAngle,'on');

% Left axis: uncontrolled inclination
yyaxis(axAngle,'left');

angleLineU = plot(axAngle,nan,nan, ...
    'Color',[0.85 0.25 0.15], ...
    'LineWidth',1.8);

ylabel(axAngle,'Uncontrolled inclination (degrees)');

ylim(axAngle,[0 180]);

% Right axis: controlled inclination
yyaxis(axAngle,'right');

angleLineC = plot(axAngle,nan,nan, ...
    'Color',[0.1 0.6 0.2], ...
    'LineWidth',1.8);

ylabel(axAngle,'Controlled inclination (degrees)');

controlledMax = max(thetaC_deg);

controlledAxisMax = max(1,ceil(controlledMax+1));

ylim(axAngle,[0 controlledAxisMax]);

xlabel(axAngle,'Time (s)');

title(axAngle,'Pendulum Inclination Comparison');

legend(axAngle, ...
    [angleLineU,angleLineC], ...
    {'Uncontrolled','LQR Controlled'}, ...
    'Location','best');

xlim(axAngle,[time(1),time(end)]);

%% 21. INITIALIZE CONTROL-FORCE PLOT

axForce = nexttile(layout,4);

hold(axForce,'on');
grid(axForce,'on');
box(axForce,'on');

forceLineX = plot(axForce,nan,nan, ...
    'LineWidth',1.5);

forceLineY = plot(axForce,nan,nan, ...
    'LineWidth',1.5);

xlabel(axForce,'Time (s)');
ylabel(axForce,'Force (N)');

title(axForce,'LQR Control Forces');

legend(axForce,{'F_x','F_y'}, ...
    'Location','best');

xlim(axForce,[time(1),time(end)]);

forceLimit = max(abs(forceC(:)));

forceAxisLimit = max(5,forceLimit+2);

ylim(axForce,[-forceAxisLimit,forceAxisLimit]);


%% 22. CONFIGURE VIDEO RECORDING

% Try MPEG-4 first. If unavailable, use Motion JPEG AVI.

videoFile = fullfile(scriptDir, ...
    'Inverted_Spherical_Pendulum_LQR_Comparison.mp4');

try

    videoObj = VideoWriter(videoFile,'MPEG-4');

    videoFormat = 'MP4';

    fprintf('\nMPEG-4 profile is available.\n');

catch

    fprintf('\nMPEG-4 profile is unavailable.\n');
    fprintf('Switching to Motion JPEG AVI...\n');

    videoFile = fullfile(scriptDir, ...
        'Inverted_Spherical_Pendulum_LQR_Comparison.avi');

    videoObj = VideoWriter(videoFile,'Motion JPEG AVI');

    videoFormat = 'AVI';

end

% Calculate frame rate
videoDuration = time(end)-time(1);

videoFrameRate = (totalVideoFrames-1)/videoDuration;

if videoFrameRate <= 0
    error('Unable to calculate a valid video frame rate.');
end

videoObj.FrameRate = videoFrameRate;
videoObj.Quality = 95;

open(videoObj);

fprintf('\nVideo recording initialized.\n');
fprintf('Video format: %s\n',videoFormat);
fprintf('Frame rate: %.2f FPS\n',videoFrameRate);
fprintf('Total frames: %d\n',totalVideoFrames);

%% 23. ANIMATION LOOP

fprintf('\nStarting comparative animation...\n');

for frameNumber = 1:totalVideoFrames

    % Current simulation frame
    k = frameIndices(frameNumber);

    %% A. UPDATE UNCONTROLLED SYSTEM

    x = cartU(k,1);
    y = cartU(k,2);

    pivot = [x,y,0];
    bob = bobU(k,:);

    vertices = [ ...
        x-halfL,y-halfW,0;
        x+halfL,y-halfW,0;
        x+halfL,y+halfW,0;
        x-halfL,y+halfW,0];

    set(cartHandleU,'Vertices',vertices);

    set(rodU, ...
        'XData',[pivot(1),bob(1)], ...
        'YData',[pivot(2),bob(2)], ...
        'ZData',[0,bob(3)]);

    set(bobHandleU, ...
        'XData',bobRadius*sphereX+bob(1), ...
        'YData',bobRadius*sphereY+bob(2), ...
        'ZData',bobRadius*sphereZ+bob(3));

    addpoints(trajectoryU,bob(1),bob(2),bob(3));

    %% B. UPDATE CONTROLLED SYSTEM

    x = cartC(k,1);
    y = cartC(k,2);

    pivot = [x,y,0];
    bob = bobC(k,:);

    vertices = [ ...
        x-halfL,y-halfW,0;
        x+halfL,y-halfW,0;
        x+halfL,y+halfW,0;
        x-halfL,y+halfW,0];

    set(cartHandleC,'Vertices',vertices);

    set(rodC, ...
        'XData',[pivot(1),bob(1)], ...
        'YData',[pivot(2),bob(2)], ...
        'ZData',[0,bob(3)]);

    set(bobHandleC, ...
        'XData',bobRadius*sphereX+bob(1), ...
        'YData',bobRadius*sphereY+bob(2), ...
        'ZData',bobRadius*sphereZ+bob(3));

    addpoints(trajectoryC,bob(1),bob(2),bob(3));

    %% C. UPDATE UNCONTROLLED ANGLE - LEFT AXIS

    yyaxis(axAngle,'left');

    set(angleLineU, ...
        'XData',time(1:k), ...
        'YData',thetaU_deg(1:k));

    %% D. UPDATE CONTROLLED ANGLE - RIGHT AXIS

    yyaxis(axAngle,'right');

    set(angleLineC, ...
        'XData',time(1:k), ...
        'YData',thetaC_deg(1:k));

    %% E. UPDATE CONTROL FORCE PLOT

    set(forceLineX, ...
        'XData',time(1:k), ...
        'YData',forceC(1:k,1));

    set(forceLineY, ...
        'XData',time(1:k), ...
        'YData',forceC(1:k,2));

    %% F. UPDATE TITLES

    title(axU,sprintf( ...
        'UNCONTROLLED | Inclination: %.2f deg', ...
        thetaU_deg(k)));

    title(axC,sprintf( ...
        'LQR CONTROLLED | Inclination: %.4f deg', ...
        thetaC_deg(k)));

    sgtitle(layout,sprintf( ...
        'Inverted Spherical Pendulum Comparison | Time: %.2f s', ...
        time(k)));

    %% G. REFRESH FIGURE

    drawnow;


    %% H. CAPTURE AND WRITE VIDEO FRAME

    % Capture current figure
    videoFrame = getframe(fig);

    frameData = videoFrame.cdata;

    % Required video dimensions
    targetHeight = 900;
    targetWidth = 1450;

    % Create a fixed-size RGB frame
    fixedFrame = zeros(targetHeight,targetWidth,3,'uint8');

    % Get captured frame dimensions
    [sourceHeight,sourceWidth,~] = size(frameData);

    % Calculate centered crop/padding dimensions
    copyHeight = min(sourceHeight,targetHeight);
    copyWidth = min(sourceWidth,targetWidth);

    sourceY = floor((sourceHeight-copyHeight)/2)+1;
    sourceX = floor((sourceWidth-copyWidth)/2)+1;

    targetY = floor((targetHeight-copyHeight)/2)+1;
    targetX = floor((targetWidth-copyWidth)/2)+1;

    % Copy captured image into fixed-size frame
    fixedFrame( ...
        targetY:targetY+copyHeight-1, ...
        targetX:targetX+copyWidth-1,:) = ...
        frameData( ...
        sourceY:sourceY+copyHeight-1, ...
        sourceX:sourceX+copyWidth-1,:);

    % Write fixed-size frame
    writeVideo(videoObj,fixedFrame);


    %% I. ANIMATION PLAYBACK TIMING

    if frameNumber < totalVideoFrames

        nextFrame = frameIndices(frameNumber+1);

        dt = time(nextFrame)-time(k);

        pause(dt/playbackSpeed);

    end

    %% J. DISPLAY PROGRESS

    if mod(frameNumber,50) == 0 || ...
            frameNumber == totalVideoFrames

        fprintf('Animation progress: %.1f%%\n', ...
            100*frameNumber/totalVideoFrames);

    end

end

%% 24. CLOSE AND FINALIZE MP4 VIDEO

close(videoObj);

fprintf('\nVideo finalized successfully.\n');
%% 25. FINAL SIMULATION SUMMARY

fprintf('\n============================================\n');
fprintf(' COMPARATIVE ANIMATION COMPLETED\n');
fprintf('============================================\n');

fprintf('Simulation duration: %.2f seconds\n', ...
    videoDuration);

fprintf('Total video frames: %d\n', ...
    totalVideoFrames);

fprintf('Video frame rate: %.2f FPS\n', ...
    videoFrameRate);

fprintf('\n--- FINAL INCLINATIONS ---\n');

fprintf('Uncontrolled: %.4f degrees\n', ...
    thetaU_deg(end));

fprintf('LQR controlled: %.6f degrees\n', ...
    thetaC_deg(end));

fprintf('\n--- CONTROL FORCES ---\n');

fprintf('Maximum |Fx|: %.4f N\n', ...
    max(abs(forceC(:,1))));

fprintf('Maximum |Fy|: %.4f N\n', ...
    max(abs(forceC(:,2))));

fprintf('\n--- VIDEO FILE ---\n');

fprintf('Format: %s\n',videoFormat);
fprintf('Location:\n%s\n',videoFile);

if isfile(videoFile)

    videoInfo = dir(videoFile);

    fprintf('Video size: %.2f MB\n', ...
        videoInfo.bytes/(1024^2));

else

    warning('Video file was not found after recording.');

end

fprintf('\n============================================\n');
fprintf('       PHASE 5 ANIMATION FINISHED\n');
fprintf('============================================\n');
