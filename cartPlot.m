function cartPlot(states, m0, m1, m2, l1, l2, recAnimation, movePlot, windowSize)
    arguments
        states
        m0 double
        m1 double
        m2 double
        l1 double
        l2 double
        recAnimation logical = false
        movePlot logical = false
        windowSize double = 10
    end
    x = states(:,1);
    theta1 = states(:,2);
    theta2 = states(:,3);

    cartHeight = .125*m0;
    cartWidth = .25*m0;

    link1Width = .1*m1;
    link2Width = .1*m2;

    cartX = x - cartWidth/2;
    link1X = x + l1*sin(theta1);
    link1Y = cartHeight/2 + l1*cos(theta1);
    link2X = link1X + l2*sin(theta2);
    link2Y = link1Y + l2*cos(theta2);
    
    if recAnimation
        if ispref('ECE726', 'VideoNumber')
            num = getpref('ECE726', 'VideoNumber') + 1;
        else
            num = 1;
        end
        cam = VideoWriter(fullfile(pwd, ['CartVideo_' num2str(num)]));
        cam.FrameRate = 20;
        open(cam);
    end

    f = figure;
    hold on
    
    ax = f.Children;
    if ~movePlot
        ax.XLim = [min(x) - 1 max(x) + 1];
    end
    axis equal
    for i = 1:height(x)
        if movePlot
            ax.XLim = [x(i) - windowSize/2 x(i) + windowSize/2];
        end
        
        if i == 1
            cart = rectangle('Position', [cartX(1) 0 cartWidth cartHeight]);
            link1 = plot([x(1) link1X(1)], [cartHeight/2 link1Y(1)], 'LineWidth', link1Width);
            link2 = plot([link1X(1) link2X(1)], [link1Y(1) link2Y(1)], 'LineWidth', link2Width);
            pause(1);
        else
            pos = cartX(i);
            cart.Position(1) = pos;
            link1.XData = [x(i) link1X(i)];
            link1.YData = [cartHeight/2 link1Y(i)];
            link2.XData = [link1X(i) link2X(i)];
            link2.YData = [link1Y(i) link2Y(i)];
            if recAnimation
                writeVideo(cam, getframe(gcf));
            end
        end

        % pause(.05);
        drawnow;
    end

    if recAnimation
        close(cam);
        setpref('ECE726', 'VideoNumber', num)
    end
end
