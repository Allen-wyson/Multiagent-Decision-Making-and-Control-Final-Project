function [plan1, plan2] = securityStrategyPlanner(actions1, actions2, trackLength)
    n1 = length(actions1);
    n2 = length(actions2);
    C = zeros(n1, n2);

    for i = 1:n1
        for j = 1:n2
            a1 = actions1{i}; 
            a2 = actions2{j};
            car1_next_state = struct('lane', a1(end,1), 'pos', a1(end,2));
            car2_next_state = struct('lane', a2(end,1), 'pos', a2(end,2));
            C(i,j) = costFunction(a1, a2, car1_next_state, car2_next_state, trackLength);
        end
    end
    % Player 1 plays security strategy
    worst_cases_1 = max(C, [], 2, 'omitnan');
    min_val = min(worst_cases_1, [], 'omitnan');
    candidates_i = find(worst_cases_1 == min_val);

    % Randomly select one of the best actions
    best_i = candidates_i(randi(length(candidates_i)));
    plan1 = actions1{best_i};

    % Player 2 plays security strategy
    worst_cases_2 = min(C, [], 1, 'omitnan');
    max_val = max(worst_cases_2, [], 'omitnan');
    candidates_j = find(worst_cases_2 == max_val);

    % Randomly select one of the best counteractions
    best_j = candidates_j(randi(length(candidates_j)));
    plan2 = actions2{best_j};
end
