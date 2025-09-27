function [no_vide] = no_vide_ros1(scan_data, L_or_r)
[n,m] = size(scan_data.Ranges);
j = 0;
if strcmp(L_or_r,"r")
    for i = 1:round(n/2)
        if abs(scan_data.Ranges(i) - scan_data.Ranges(i+1)) < 0.5%(tan(scan_data.angle_increment)*0.5)
            j = j+1;
        else 
            break;
        end
    end
    if j >= n/2-0.05*n
        no_vide = true;
    else
        no_vide = false;
    end
end

if strcmp(L_or_r,"l")
    for i = round(n/2)+1:n
        if abs(scan_data.Ranges(i-1) - scan_data.Ranges(i)) < 0.5%(tan(scan_data.angle_increment)*0.5)
            j = j+1;
        else 
            break;
        end
    end
    if j >= n/2-0.1*n
        no_vide = true;
    else
        no_vide = false;
    end
end
end