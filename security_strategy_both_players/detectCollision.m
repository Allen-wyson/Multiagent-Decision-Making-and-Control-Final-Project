function collide = detectCollision(a1,a2)
%% This function allows to detect if the two cars would crash if player 1 choses action a1 and player 2 choses action a2
% collide : bool, True if actions lead to a crash
% a1 : list of subactions played by player 1
% a2 : list of subactions played by player 2

    len_action_1 = size(a1,1);
    len_action_2 = size(a2,1);
    max_len = max(len_action_1, len_action_2);
    collide = false;

    % Extend the shorter trajectory with zeros
    if len_action_1 < max_len
        a1(end+1:max_len, :) = 0;
    end
    if len_action_2 < max_len
        a2(end+1:max_len, :) = 0;
    end

    % Check if cars are on the same tile during action
    for k = 1:max_len
        if isequal(a1(k,:), a2(k,:))
            collide = true;
            return;
        end
    end

    % Check for cross-over during action
    for k = 1:max_len-1
        if isequal(a1(k,:), a2(k+1,:)) && isequal(a1(k+1,:), a2(k,:))
            collide = true;
            return;
        end
    end


end

