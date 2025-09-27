function [goalIsolated, not_on_my_ranges] = goal_isolated_ros1(n, thetaGoal, distToGoal, scanData)
index = round((scanData.AngleMax - thetaGoal) * n /  (scanData.AngleMax - scanData.AngleMin));
if index >0 && index<= n
    not_on_my_ranges = 0;
    if scanData.Ranges(index) < distToGoal
        goalIsolated = 1;
    else
        goalIsolated = 0;
    end
else
    not_on_my_ranges = 1;
    goalIsolated = 1;
end

end