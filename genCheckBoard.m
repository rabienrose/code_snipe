clear();
img(1:200*8,1:200*9)=255;
for i=1:8
    if mod(i,2) == 0
        for j=2:2:8
            img((i-1)*200+1:i*200, (j-1)*200+1:j*200) =0;
        end
    else
        for j=1:2:9
            img((i-1)*200+1:i*200, (j-1)*200+1:j*200) =0;
        end
    end
end
imsave(img,'checkboard.bmp');
