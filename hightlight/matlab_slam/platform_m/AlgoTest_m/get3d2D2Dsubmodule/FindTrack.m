% clear;
%% find continue track Id 
%  Here flag should be a row vector (contains numbers of 0/1)

%   flag = round(rand(1,30));
function  Track = FindTrack(flag)
   % check the continues positions
     positions = find(flag == 1);
     
   % computer the subsets
     rowId = 1;
     TrackSet{rowId, 1} = []; 
     for ScanId = 2 : size(positions, 2)
         % check if they are continue
         if (positions(1, ScanId) - positions(1, ScanId - 1) == 1)
             TrackSet{rowId, 1} = union(TrackSet{rowId, 1}, positions(1, ScanId));
             TrackSet{rowId, 1} = union(TrackSet{rowId, 1}, positions(1, ScanId - 1));
         else
             % create a new subset
             rowId = rowId + 1;
             TrackSet{rowId, 1} = []; 
         end
     end
    
   % delete the empty subcells
     id = cellfun('length', TrackSet);
     TrackSet(id == 0) = [];
   % reconstruct trackset as a vector 
     Track = [];
     if isempty(TrackSet) == 0
         for setId  = 1 : size(TrackSet, 1)
             Track = [Track, TrackSet{setId, 1}];
         end
     end
% end