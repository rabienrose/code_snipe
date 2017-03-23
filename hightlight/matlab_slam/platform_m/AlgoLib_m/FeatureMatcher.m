% Feature matching interface to platform
% Author: Zhongjun Dai
% Date: 2016-6-30

function [indexPairs] = FeatureMatcher(descriptor1, descriptor2, varargin)
    if strcmp('DIRECTMATCH', varargin{1})
        indexPairs = DirectMatch(descriptor1, descriptor2, 'Exhaustive', true);
    elseif strcmp('OPTICFLOW', varargin{1})
        indexPairs = OpticFlow();
    end     
end