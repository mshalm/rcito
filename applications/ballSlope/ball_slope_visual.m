function ball_slope_visual(x, alpha, r)
    xmin = -10;
    xmax = 10;
    ymin = -10;
    ymax = 10;
    qx = x(1);
    qy = x(2);
    
    figure(17);
    clf;
    hold on;
    plot([xmin xmax], tan(alpha) * [xmin xmax],'LineWidth',2);
    viscircles([qx qy], 1);
    axis equal;
    xlim([xmin xmax]);
    ylim([ymin ymax]);
    
    
end

