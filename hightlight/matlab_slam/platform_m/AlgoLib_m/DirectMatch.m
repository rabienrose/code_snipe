% Direct match module which include these methods below:
% 'Exhaustive' compute the pairwise distance between feature vectors in 
% features1 and features2.
%'Approximate' use an efficient approximate nearest neighbor search. 
% Use this method for large feature sets[1].
% [1]Muja, M., and D. G. Lowe. "Fast Approximate Nearest Neighbors with 
% Automatic Algorithm Configuration." 
% TODO: AND OTHER MATCH METHOD...
% Author: Zhongjun Dai
% Date: 2016-6-27

function [indexPairs, matchMetric] = DirectMatch(features1, features2,...
                                                   method, unique)
    if nargout == 1    
        indexPairs = matchFeatures(features1, features2,...
            'Method', method, 'Unique', unique);
    elseif nargout == 2
        [indexPairs, matchMetric] = matchFeatures(features1, features2,...
            'Method', method, 'Unique', unique);
    end  
end