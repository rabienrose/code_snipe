function [fullAddr] = getFullAddress(fileName)
    global params;
    fullAddr = [params.imageDir fileName];
end