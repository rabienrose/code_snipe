%% Generate  camera translation vectors
% input: 
%        diameter: diameter of circumference
%        angle: the rotation angle each time 
%        totalAngle: total rotation angle  
%        length: distance from the 1st frame to the last frame 
%        stepsize: every step size along the x-axis and y-axis
%        type: 0, Rotate and translate around a circle
%              1, have rotatio and translation;  
%              2, fix t (no transalation)
% output: 
%        allt: all translation vectors of each move 

function allt = getTransVectors(diam, angle, totalAngle, length, stepsize, type)   
    switch type 
        case 0  
             % calculate the total moving times
               Times = totalAngle / angle;
             % initialized all coordinates
               allt =  zeros(Times + 1, 3);
             % original translation vector
               allt(1, :) = [diam/2, 0, 0];
             % calculate the coordinate of in Cartesian  coordinates
               accumAngle = 0;  % initial accumulate angle is 0
               for rotate = 1 : Times
                    accumAngle = accumAngle + angle;   % current accumulate angle
                    % tranform digital value to degree
                    accumAngle_degree = accumAngle * pi /180;
                    x_rotate = diam/2 * cos(accumAngle_degree); % x coordinate
                    z_rotate = diam/2 * sin(accumAngle_degree); % y coordinate
                  % save current data
                    allt(rotate + 1, :) = [x_rotate, 0, z_rotate];
               end
        case 1  % fix R
             % calculate the total moving times
               Times = length / stepsize;
             % initialized all coordinates
               allt =  zeros(Times, 3);
             % original translation vector
%                allt(1, :) = [-diam/2, 0, 0];
             % calculate the coordinate of in Cartesian  coordinates
               accumlength = -diam / 2;  % initial accumulate length is 0
               for rotate = 1 : Times
                    accumlength = accumlength + stepsize;   % current accumulate steplength
                    % save current data
                    allt(rotate, :) = [accumlength, 0, 0];
               end
        case 2  % fix t
               times = totalAngle / angle;
               t = [diam/2, 0, 0];
               allt = repmat(t, times, 1);
        otherwise
               warning('Unexpected type.')
    end
end