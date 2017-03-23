%  show the camera trajectory and the track of FrameId and previous FrameId .

function ShowData(points3D, points2D, flag2D, AllR, Allt, DataRange, visible, frameID)     
 
% Plot the track of the data in the view port (W * H pixels)    
   if visible == 0
       disp('Show option is disabled!');
       return;
   elseif visible == 1
          if(frameID <= 1 )
             disp('invalid frameId, please check!');
             return;
          end
       endId = frameID;
       startId = endId - 1;
       pxs1 = points2D{startId};
       flags1 = flag2D{startId};
       pxs2 = points2D{endId};
       flags2 = flag2D{endId};
       figure(20);
       hold on;

       for i=1:size(pxs1, 1)
            if flags1(i) == 1 && flags2(i) == 1
                line([pxs1(i,1), pxs2(i,1)], [pxs1(i,2), pxs2(i,2)])
                plot(pxs1(i,1), pxs1(i,2), 'go')
                plot(pxs2(i,1), pxs2(i,2), 'r*');
            end
       end
       title(['Frame(', num2str(startId), ')--Frame(', num2str(endId),')']);
    %    axis ij
       xlim([0 1024]);
       ylim([0 768]);

    %%  -----------for test: from here--------------------
%        posi1 = find(flags1 == 1);
%        posi2 = find(flags2 == 1);
%        posiintersec = intersect(posi1,posi2);
%        P2D1 = points2D{startId}(posiintersec,:);
%        P2D2 = points2D{endId}(posiintersec,:); 
%        save MP2.mat P2D1 P2D2 posi1 posi2
    %   -----------for test: end here--------------------- 

     %% plot the figure of 3D points   

      % Figure1: original 3D points and camera trajectory   
        figureID = 21;
      % color type of pointscloud, 1 for random color, 2 for black color, 3 for purple
        colortype = 2;
        figurepositionsize = [40 200 600 500];
      % get the pose in camera coordinate  
         for indPro = 1 : size(AllR, 2)
             homoTrans = [AllR{indPro} Allt(indPro, :)'; 0 0 0 1]^-1;
             cameraR{indPro} = homoTrans(1 : 3, 1 : 3)';
             camerat(indPro,:) = homoTrans(1 : 3, 4)';
         end
        x_lim = DataRange(1, :);
        y_lim = DataRange(2, :);
        z_lim = DataRange(3, :); 
        plotCamerashow(cameraR, camerat, x_lim, y_lim, z_lim, figureID);
        plot3dcloud(points3D, colortype, x_lim, y_lim, z_lim, figurepositionsize);
%         view(0,0);
        title('Original 3D points')

   end
end     
         