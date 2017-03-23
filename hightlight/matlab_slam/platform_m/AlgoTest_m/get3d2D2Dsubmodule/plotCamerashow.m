%% plot the camera trajectory 
% input: 
%        allR: all Rotation Matrixs
%        allt: all translation vectors 
%        x_lim: plot range of x coordinate
%        y_lim: plot range of y coordinate
%        z_lim: plot range of z coordinate
%        figureID: figure number
function plotCamerashow(allR, allt, x_lim, y_lim, z_lim, figureID)
     

    % get the number of R components
      numR = size(allR, 2);
        
    % define the camera size 
      cameraSize = 0.3;
    
    % plot camera trajectory
      figure(figureID) 
      
      for Frameind = 1 : numR
         plotCamera('Location', allt(Frameind,:), 'Orientation', allR{Frameind},  'Size', cameraSize, ...
                   'Color', 'g', 'Label', num2str(Frameind), 'Opacity', 0); 
         
         xlim([x_lim(1)+3, x_lim(2)-3]);
         ylim([y_lim(1)+3, y_lim(2)-3]);
         zlim([z_lim(1)+3, z_lim(2)-3]);
%          xlim([x_lim(1) + 3,x_lim(2) - 3]);
%          ylim([y_lim(1) + 3,y_lim(2) - 3]);
%          zlim([z_lim(1) + 3,z_lim(2) - 1]);
         xlabel(' x-axis ');
         ylabel(' y-axis ');
         zlabel(' z-axis ');   
         hold on
      end
     % fix figure position
%       set(gcf,'position',[40 300 600 500])

      
end