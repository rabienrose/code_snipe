% Search matching pairs by epipolar, before triangulation un-matched 
% keys, matching pairs should be obtained by this.

function [isSucc] = matchByEpiCore_m(frameId, offset)
    global params;
    global coreDataMatrix;
    
    if frameId + offset < 1 || frameId > params.imagePaths.Count
%         disp(['Absolute frame ', num2str(frameId), ': invalid inner input in matchByEpiCore, please check!']);
        isSucc = false;
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner + offset < 1 || currIdInner > size(coreDataMatrix.frImgDirMatrix, 1)
        disp(['Inner frame ', num2str(frameId), ': invalid inner input in matchByEpiCore, please check!']);
        isSucc = false;
        return;
    end
    
    prevPose = coreDataMatrix.frPoseMatrix(:, :, currIdInner + offset);
    if isempty(prevPose)
        disp('empty previous pose!');
        isSucc = false;
        return;
    end
    
    currPose = coreDataMatrix.frPoseMatrix(:, :, currIdInner);
    if isempty(currPose)
        disp('empty current pose!');
        isSucc = false;
        return;
    end
    
    %% query keys those has not map points.
    key = struct('Location', [],...
                 'Scale', [],...
                 'Metric', [],...
                 'Misc', [],...
                 'Orientation', [],...
                 'Octave', []);
    
    currZeroIdx = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner) == 0;
    currKeys = coreDataMatrix.kpPosiMatrix(:, currZeroIdx, currIdInner);
    currDesc = coreDataMatrix.kpDescMatrix(:, currZeroIdx, currIdInner);
    cvtCurrKeys = key;
    cvtCurrKeys.Location    = single(currKeys');
    cvtCurrKeys.Scale       = single(ones(size(cvtCurrKeys.Location, 1), 1));
    cvtCurrKeys.Metric      = single(zeros(size(cvtCurrKeys.Location, 1), 1));
    cvtCurrKeys.Orientation = single(zeros(size(cvtCurrKeys.Location, 1), 1));
    cvtCurrKeys.Misc        = cast(zeros(size(cvtCurrKeys.Location, 1), 1), 'uint32');
    cvtCurrKeys.Octave      = cast(coreDataMatrix.kpOctaveMatrix(currZeroIdx, currIdInner), 'uint32');
             
    prevZeroIdx = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner + offset) == 0;
    prevKeys = coreDataMatrix.kpPosiMatrix(:, prevZeroIdx, currIdInner + offset);
    prevDesc = coreDataMatrix.kpDescMatrix(:, prevZeroIdx, currIdInner + offset);
    cvtPrevKeys = key;
    cvtPrevKeys.Location    = single(prevKeys');
    cvtPrevKeys.Scale       = single(ones(size(cvtPrevKeys.Location, 1), 1));
    cvtPrevKeys.Metric      = single(zeros(size(cvtPrevKeys.Location, 1), 1));
    cvtPrevKeys.Orientation = single(zeros(size(cvtPrevKeys.Location, 1), 1));
    cvtPrevKeys.Misc        = cast(zeros(size(cvtPrevKeys.Location, 1), 1), 'uint32');
    cvtPrevKeys.Octave      = cast(coreDataMatrix.kpOctaveMatrix(prevZeroIdx, currIdInner + offset), 'uint32');         
               
    %% try to search matching by epipolar.
    [matchedPairs, currMatchedFlag, prevMatchedFlag] = ...
                                        SearchByEpipolar_m(cvtCurrKeys, currDesc', currPose,...
                                                         cvtPrevKeys, prevDesc', prevPose, ...
                                                         params.cameraParam, params.scaleFactorSet);
    
    if isempty(matchedPairs)
%         disp(['Absolute frame ', num2str(frameId), ': no epipolar matches between the two frames!']);
        isSucc = false;
        return;
    end
    isSucc = true;
    
    %% save matched result to CoreData.
    currIdx1 = find(currZeroIdx == 1);
    currIdx2 = currIdx1(matchedPairs(:, 1));
    prevIdx1 = find(prevZeroIdx == 1);
    prevIdx2 = prevIdx1(matchedPairs(:, 2));
    
    matches(:,1) = prevIdx2;
    matches(:,2) = currIdx2;
    fill2DMatches_m(currIdInner + offset, currIdInner, matches);
end


