function ball_collision_visual(x, alpha, r, r_s)
%BALL_SLOPE_VISUAL Summary of this function goes here
%   Detailed explanation goes here
    xmin = -10;
    xmax = 10;
    ymin = -10;
    ymax = 10;
    qx = x(1);
    qy = x(2);
    
    figure(17);
    clf;
    hold on;
    viscircles([qx qy], r);
    viscircles([0 0], r_s);
    axis equal;
    xlim([xmin xmax]);
    ylim([ymin ymax]);
    
    
end

