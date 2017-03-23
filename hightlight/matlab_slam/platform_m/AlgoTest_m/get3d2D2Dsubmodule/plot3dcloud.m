%% plot the 3d pointscloud 
% input: 
%        points: 3d points; (N * 3)
%        colortype: color type of pointscloud, 1 for random color, 2 for black color, 3 for purple;
%        x_lim: plot range of x coordinate
%        y_lim: plot range of y coordinate
%        z_lim: plot range of z coordinate
%        figurePositionSize: the fixed figure position and size
function plot3dcloud(points, colortype, x_lim, y_lim, z_lim, figurePositionSize)
     
    % get the number of points
        numPoints = size(points, 1);
    % generate random color (type: uint8)
        if colortype == 1   
            color = uint8(255 * rand(numPoints, 3));
        elseif colortype == 2
            color = uint8(255 * zeros(numPoints, 3));  % black points
        elseif colortype == 3
            color = uint8(repmat([128, 0, 64], numPoints, 1)); % purple points
        end
        
    % Create the point cloud
         ptCloud = pointCloud(points, 'Color', color);
         

    % Visualize the point cloud (along the z coordinate)
         pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'up', ...
                'MarkerSize', 100);
         xlim([x_lim(1) - 1,x_lim(2) + 1]);
         ylim([y_lim(1) - 1,y_lim(2) + 1]);
         zlim([z_lim(1) - 1,z_lim(2) + 1]);
         xlabel(' x-axis ');
         ylabel(' y-axis ');
         zlabel(' z-axis ');
         title('3D points')
    
    % fix figure position
         set(gcf,'position', figurePositionSize)
end