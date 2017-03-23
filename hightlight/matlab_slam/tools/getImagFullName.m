function [fullName] = getImagFullName(nameLen, fileRoot, id)
    initChar = [{'0'};
                {'00'};
                {'000'};
                {'0000'};
                {'00000'};
                {'000000'}];
    
    intStrN = num2str(id);
    if nameLen ==-1
        fullName = [fileRoot intStrN '.jpg'];
    else
        intStr = initChar{nameLen};
        intStr(end-size(intStrN,2)+1:end) = intStrN;
        fullName = [fileRoot intStr '.jpg'];
    end
end