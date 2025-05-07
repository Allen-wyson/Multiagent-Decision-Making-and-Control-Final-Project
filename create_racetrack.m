function ax = create_racetrack(numLanes, trackLength)
%CREATE_RACETRACK Draw a numLanes-by-trackLength grid racetrack.
%   ax = CREATE_RACETRACK(numLanes, trackLength) opens a figure,
%   draws a grid of size numLanes×trackLength, marks start (green)
%   and finish (red) lines, labels the axes, and returns the axes handle.
%
%   Example:
%     ax = create_racetrack(3, 50);
%     hold(ax,'on');
%     % now you can plot cars or paths on ax

  %--- Set defaults if not provided ---
  if nargin < 1, numLanes    = 3;  end
  if nargin < 2, trackLength = 50; end

  %--- Create figure and configure axes ---
  figure('Name','Grid Racetrack','NumberTitle','off');
  ax = gca;
  hold(ax,'on');
  axis(ax,'equal');
  xlim(ax,[0, trackLength]);
  ylim(ax,[0, numLanes]);
  xlabel(ax,'Position');
  ylabel(ax,'Lane');
  title(ax,'Grid Racetrack');

  %--- Draw grid cells ---
  for row = 1:numLanes
    for col = 1:trackLength
      rectangle(ax, 'Position',[col-1, row-1, 1, 1], ...
                'EdgeColor','k', 'LineWidth',0.5);
    end
  end

  %--- Mark start (green) and finish (red) lines ---
  yl = [0, numLanes];
  plot(ax, [0, 0],       yl, 'g-', 'LineWidth',2);            % start at col=1 → x=0
  plot(ax, [trackLength, trackLength], yl, 'r-', 'LineWidth',2);

  %--- Annotate lanes on the left ---
  for row = 1:numLanes
    text(ax, -0.5, row-0.5, sprintf('Lane %d',row), ...
         'HorizontalAlignment','right');
  end

  %--- Configure tick marks and orientation ---
  set(ax, ...
      'YDir','reverse', ...               % so Lane 1 is at top
      'XTick',0:5:trackLength, ...
      'YTick',0:1:numLanes);
  hold(ax,'off');
end
