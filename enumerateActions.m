function actions = enumerateActions(lane0, pos0, maxD, numLanes, trackLength)
% enumerateActions lists all possible “snake” moves from (lane0,pos0)
%   lane0, pos0       : starting lane and position
%   maxD              : maximum sub-steps per action (e.g. 3)
%   numLanes          : total number of lanes (e.g. 3)
%   trackLength       : total track length (e.g. 50)
% Example:
%   actions = enumerateActions(2,1,3,3,50);

    %% input validation
    narginchk(3,5);   % require at least 3 inputs, at most 5
    if nargin < 4, numLanes    = 3;  end
    if nargin < 5, trackLength = 50; end

    % lane0 must be integer in [1, numLanes]
    assert(ismember(lane0,1:numLanes), ...
           'lane0 must be integer between 1 and %d', numLanes);
    % pos0 must be integer in [1, trackLength-1]
    assert(pos0 >= 1 && pos0 < trackLength && pos0==floor(pos0), ...
           'pos0 must be integer in [1, %d)', trackLength);
    % maxD must be positive integer
    assert(maxD >= 1 && maxD == floor(maxD), ...
           'maxD must be a positive integer');

    %% enumerate snake‐style actions
    actions = {};  % cell array to hold each action path
    dfs(lane0, pos0, maxD, []);  % start recursion

    function dfs(curLane, curPos, stepsLeft, path)
        if ~isempty(path)
            actions{end+1} = path;  %#ok<AGROW>
        end
        if stepsLeft == 0
            return;
        end
        for dl = -1:1
            nextLane = curLane + dl;
            nextPos  = curPos + 1;
            % ensure next step stays on track
            if nextLane>=1 && nextLane<=numLanes && nextPos<=trackLength
                newPath = [path; nextLane, nextPos];
                dfs(nextLane, nextPos, stepsLeft-1, newPath);
            end
        end
    end
end
