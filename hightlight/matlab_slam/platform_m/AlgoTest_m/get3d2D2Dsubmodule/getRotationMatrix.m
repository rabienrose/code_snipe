%% Generate camera Rotation Matrices 
% here we use simple "Axis-angle method" to get simulated R
% and use x-coordinate as Axis
% input: 
%        Axis:  rotation axis
%        angle: rotation angle each time 
%        totalRotationAngle: total rotation angle         
% output: 
%        allR: all Rotation Matrixes of each move (Including the original R: eye(3))

function allR = getRotationMatrix(Axis, rotangle, totalRotationAngle)  
     
     % pass the parameters
       x = Axis(1);
       y = Axis(2);
       z = Axis(3);
       
     % calculate the total moving times
       Times = totalRotationAngle / rotangle;
     
     % initialized all Rotation Matrix
       for ind = 1 : (totalRotationAngle/rotangle + 1)
             allR{ind} = zeros(3);
       end
     
     % original Rotation Matrix
       allR{1} = eye(3);

     % calculate above the cell Matrixes M1, M2, and M3
       M1 = eye(3);
       M2 = [0, -z, y; z, 0, -x; -y, x, 0];
       M3 = [-y*y-z*z, x*y, x*z; x*y, -x*x-z*z, y*y; x*z, y*z, -x*x-y*y];
     
     % calculate the Rotation Matrixes of 
       accumAngle = 0;  % initial accumulate angle is 0
       for rotate = 1 : Times
            accumAngle = accumAngle + rotangle;   % current accumulate angle
            % tranform digital value to degree
            accumAngle_degree = accumAngle * pi /180;
            % compute current Rotation Matrix 
            R = M1 + sin(accumAngle_degree) * M2 + (1 - cos(accumAngle_degree)) * M3; 
            % save current data
            allR{rotate + 1} = R;
       end

end
