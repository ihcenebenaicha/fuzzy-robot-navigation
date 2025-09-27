function vid = find_vid_v2(scan_data, robot, goals, goal_index, L_or_r, vid_width_min)
vid.width = 0;
vid.dist_to_g = inf;


angles_scan = scan_data.AngleMin:scan_data.AngleIncrement:scan_data.AngleMax;
[n,m] = size(scan_data.Ranges);
if strcmp(L_or_r,"r")
    for i = round(n/2):-1:2
        k = 1;
        for  j = i-1:-1:1
            if scan_data.Ranges(i-k) > scan_data.Ranges(i)+0.4 && vid.width < vid_width_min 
                vid.edge = i;
                a = [robot.x+scan_data.Ranges(i) * cos(robot.theta+angles_scan(i)) robot.y+scan_data.Ranges(i) * sin(robot.theta+angles_scan(i))];
                b = [robot.x+scan_data.Ranges(i) * cos(robot.theta+angles_scan(i-k)) robot.y+scan_data.Ranges(i) * sin(robot.theta+angles_scan(i-k))];
                vid.width = norm(a-b);
                k = k+1;
            elseif vid.width > vid_width_min
                mid_vid = a + (b-a)/2;
                dist_r_to_mid_vid = norm(mid_vid-[goals(goal_index,1) goals(goal_index,2)]);
                dist_mid_vid_to_g = norm(mid_vid-[robot.x robot.y]);
                vid.dist_to_g = dist_r_to_mid_vid + dist_mid_vid_to_g;
                return;
            else
                k = 1;
                break;
            end
        end
    end
end

if strcmp(L_or_r,"l")
    for i = round(n/2)+1:n-1
        k = 1;
        for  j = i+1:n
            if  scan_data.Ranges(i+k) > scan_data.Ranges(i)+0.4 && vid.width < vid_width_min 
                vid.edge = i;
                a = [robot.x+scan_data.Ranges(i) * cos(robot.theta+angles_scan(i)) robot.y+scan_data.Ranges(i) * sin(robot.theta+angles_scan(i))];
                b = [robot.x+scan_data.Ranges(i) * cos(robot.theta+angles_scan(i+k)) robot.y+scan_data.Ranges(i) * sin(robot.theta+angles_scan(i+k))];
                vid.width = norm(a-b);
                k = k+1;
            elseif vid.width > vid_width_min
                mid_vid = a + (b-a)/2;
                dist_r_to_mid_vid = norm(mid_vid-[goals(goal_index,1) goals(goal_index,2)]);
                dist_mid_vid_to_g = norm(mid_vid-[robot.x robot.y]);
                vid.dist_to_g = dist_r_to_mid_vid + dist_mid_vid_to_g;
                return;
            else
                k = 1;
                break;
            end
        end
    end
end



end