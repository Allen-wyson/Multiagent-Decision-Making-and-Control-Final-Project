%% create_racetrack.m
% This script creates and visualizes a 50×3 grid racetrack.

% Parameters
numLanes    = 3;    % number of lanes (rows)
trackLength = 50;   % track length (columns)

% Create the track matrix (all zeros for now)
% Rows correspond to lanes 1..3, columns to positions 1..50
track = zeros(numLanes, trackLength);

% Define start and finish positions
startPos  = 1;              % starting column
finishPos = trackLength;    % finishing column

% Visualization
figure('Name','Grid Racetrack','NumberTitle','off');
hold on;
axis equal;
xlim([0 trackLength]);
ylim([0 numLanes]);
xlabel('Position');
ylabel('Lane');
title('Grid Racetrack');

% Draw grid cells
for lane = 1:numLanes
    for pos = 1:trackLength
        % draw each cell as a 1×1 rectangle
        rectangle('Position',[pos-1, lane-1, 1, 1], ...
                  'EdgeColor','k', 'LineWidth', 0.5);
    end
end

% Mark start line (green) and finish line (red)
yStart = [0, numLanes];
plot([startPos-1, startPos-1], yStart, 'g-', 'LineWidth', 2);
plot([finishPos, finishPos],     yStart, 'r-', 'LineWidth', 2);

% Annotate lanes
for lane = 1:numLanes
    text(-1, lane-0.5, sprintf('Lane %d', lane), ...
         'HorizontalAlignment','right');
end

% Adjust axes
set(gca, 'YDir','reverse', ...
         'XTick', 0:5:trackLength, 'YTick', 0:1:numLanes);
hold off;
