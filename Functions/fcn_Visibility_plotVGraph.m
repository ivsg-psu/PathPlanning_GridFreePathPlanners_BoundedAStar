function fcn_Visibility_plotVGraph(vgraph, all_pts,  styleString)
Npoints = size(vgraph,1);
for ith_fromIndex = 1:Npoints
    pointsToPlot = [];
    for jth_toIndex = 1:size(vgraph,1)
        if vgraph(ith_fromIndex,jth_toIndex) == 1
            % plot([all_pts(i,1),all_pts(j,1)],[all_pts(i,2),all_pts(j,2)],'-g')
            % pause(0.01);
            pointsToPlot = [pointsToPlot; [all_pts(ith_fromIndex,1:2); all_pts(jth_toIndex,1:2); nan(1,2)]]; %#ok<AGROW>

        end
    end
    plot(pointsToPlot(:,1),pointsToPlot(:,2),styleString)
    drawnow;

    if 1==0 % To save movie
        % Capture the current frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert to indexed image

        % Write the frame to the GIF
        if ith_fromIndex == 1
            % Create a new GIF file for the first frame
            imwrite(imind, cm, filename, 'gif', 'LoopCount', loopCount, 'DelayTime', delayTime);
        else
            % Append subsequent frames
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
        end
    end
end
end % Ends the function