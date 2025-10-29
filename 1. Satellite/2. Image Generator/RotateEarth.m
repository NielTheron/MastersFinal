%==========================================================================
% Niel Theron
% 03-09-2025
%==========================================================================
function RotateEarth(ax, angle_deg)

    surfaces = findobj(ax, 'Type', 'surface');
    for i = 1:length(surfaces)
        rotate(surfaces(i), [0 0 1], angle_deg, [0 0 0]);
    end
end
