%% generate 3D points in the range 
% input: 
%        x_lim: range of x coordinate
%        y_lim: range of y coordinate
%        z_lim: range of z coordinate
%        randseed: 0 for random choose; 1 for fixed seed
% output: 
%        points3d: 3d points(number * 3)

function points3d = get3dpoints(x_lim,y_lim,z_lim,number,randseed)   
    
       fixseed1 = 7;
       fixseed2 = 32;
       fixseed3 = 167;
     % generate the coordinate in three directions       
       if randseed == 1   
           rand('seed', fixseed1); 
       end
       x_coor = x_lim(1) + (x_lim(2) - x_lim(1)) .* rand(number, 1);
       
       if randseed == 1   
           rand('seed', fixseed2); 
       end
       y_coor = y_lim(1) + (y_lim(2) - y_lim(1)) .* rand(number, 1);
       
       if randseed == 1   
           rand('seed', fixseed3); 
       end
       z_coor = z_lim(1) + (z_lim(2) - z_lim(1)) .* rand(number, 1);     
     
     % genetate final 3D points
       points3d = [x_coor, y_coor, z_coor];
end