%% Generata Sythetic data.
   clc; close all; clear;
     
%% Set parameters  
   % choose two nearby frame (set the first frame number: 1~30) 
   frame1 = 1;
   % field of camera view
   Theta = 90;
   % 3D/2D points pair number
   Number = 2000;
   % define the range of points
   x_lim = [-10, 10];
   y_lim = [-10, 10];
   z_lim = [-10, 10];
%   tic
%% Generate  data 
   % Rtype: 0, have rotatio and translation;    1, fix R (no rotation);    2, fix t (no transalation)
   Rtype = 0;
   % Use function Get3D2D2DParis() to generate synthetic data
   [points3D, points2D, flag2D, viewpointsnum, AllR, Allt, K] = Get3D2D2DPair(Theta, Number, x_lim, y_lim, z_lim, Rtype);  
%   toc
%% Plot the track of the data in the view port (W * H pixels)    
   tic
   startId =1;
   endId =startId+1;
   pxs1 = points2D{startId};
   flags1 = flag2D{startId};
   pxs2 = points2D{endId};
   flags2 = flag2D{endId};
   figure(1);
   hold on;
   
   for i=1:size(pxs1, 1)
        if flags1(i) ==1 && flags2(i) ==1
            line([pxs1(i,1),pxs2(i,1)],[pxs1(i,2),pxs2(i,2)])
            plot(pxs1(i,1),pxs1(i,2),'go')
            plot(pxs2(i,1), pxs2(i,2), 'r*');
        end
   end
   xlim([1 1024]);
   ylim([1 768]);
%    axis equal
%        
%    toc
   
 %% plot the figure of 3D points   
%     tic
  % Figure1: original 3D points and camera trajectory   
    figureID = 2;
  % color type of pointscloud, 1 for random color, 2 for black color, 3 for purple
    colortype = 2;
    figurepositionsize = [40 500 500 400];
    plotCamerashow(AllR, Allt, x_lim, y_lim, z_lim, figureID);
    plot3dcloud(points3D, colortype, x_lim, y_lim, z_lim, figurepositionsize);
    view(0,0);
    title('Original 3D points')
%    toc
     
         