len=100000;
record=[];
nn=0;
for jj = 10000:10000: len
    nn=nn+1;
    tic
    a=[];
    for i=1:jj
        a =[a {'chamo'}];
    end
    toc
    record(nn,1)=toc;
    
    tic
    a={[]};
    for i=1:jj
        a(end+1) ={'chamo'};
    end
    record(nn,2)=toc;

    tic
    a(jj) = {[]};
    for i=1:jj
        a(i) ={'chamo'};
    end
    a = a(1:jj-10);
    record(nn,3)=toc;
end

%%
figure(1)
hold on
plot(10000:10000:len, log(record(:,1)),'r')
plot(10000:10000:len, log(record(:,2)),'g')
plot(10000:10000:len, log(record(:,3)),'b')