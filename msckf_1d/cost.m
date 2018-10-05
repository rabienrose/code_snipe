function [f] = cost(x)
    global ps us cof;
    sum=0;
    for i=1:length(ps)
        sum= sum+(cof/(x-ps(i))-us(i))^2;
    end
    f=sum;
end