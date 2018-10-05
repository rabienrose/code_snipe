%% read data
frameid='100295';
root='/Volumes/chamo/working/catkin_mac/src/loc/core/algorithm_vehicle/examples/LocRealTimeRosAlgo/output/';
img =imread([root 'img' frameid '.jpg']);
img = rgb2gray(img);
fid = fopen([root 'output' frameid '.txt']);
tline = fgetl(fid);
kp_count=0;
mp_count=0;
mp_desc_count=0;
match_count=0;
kp_posi=[];
kp_desc=[];
mp_posi=[];
mp_desc_offset=[];
mp_desc=[];
match_posi=[];
while ischar(tline)
    if strcmp(tline, 'kps')
        read_type=0;
        tline = fgetl(fid);
        while ~strcmp(tline, 'mps')
            kp_count=kp_count+1;
            kp_posi(kp_count,:)=strread(tline);
            tline = fgetl(fid);
            kp_desc(kp_count,:)=strread(tline);
            tline = fgetl(fid);
        end
        tline = fgetl(fid);
        while ~strcmp(tline, 'pose')
            mp_count=mp_count+1;
            mp_posi(mp_count,:)=strread(tline);
            tline = fgetl(fid);
            desc_num = strread(tline);
            mp_desc_count=mp_desc_count+1;
            mp_desc_offset(mp_count)=mp_desc_count;
            for i=1:desc_num
                tline = fgetl(fid);
                mp_desc(mp_desc_count,:)=strread(tline);
                mp_desc_count=mp_desc_count+1;
            end
            tline = fgetl(fid);
        end
        tline = fgetl(fid);
        pose = strread(tline);
        pose(17)=[];
        pose = reshape(pose, [4 4]);
        pose=pose';
        tline = fgetl(fid);%matches
        tline = fgetl(fid);
        while ischar(tline)
            match_count=match_count+1;
            match_posi(match_count,:)=strread(tline);
            tline = fgetl(fid);
        end
    else
        'error!!'
        break;
    end
end
mp_desc(:,65)=[];
kp_desc(:,65)=[];
fclose(fid);
%% show kps
close all;
fx = 1102.9*3/4;
fy = 1103.5*3/4;
cx = 662*3/4;
cy = 391*3/4;
k=[fx 0  cx 0
   0  fy cy 0
   0  0  1  0];

figure(1)
imshow(img);
hold on;

plot(kp_posi(:,1),kp_posi(:,2),'ro','MarkerSize', 5);

for i=1:size(mp_posi,1)
    uv = k*pose*[mp_posi(i,:) 1]';
    if uv(3,1)<=0
        continue;
    end
    u = uv(1,1)/uv(3,1);
    v = uv(2,1)/uv(3,1);
    plot(u,v,'y.','MarkerSize', 20);
end

for i=1:size(match_posi,1)
    mp=match_posi(i,1:3);
    kp=match_posi(i,4:5);
    uv = k*pose*[mp 1]';
    u = uv(1,1)/uv(3,1);
    v = uv(2,1)/uv(3,1);
    plot([u kp(1,1)], [v kp(1,2)], 'b','LineWidth',2);
end

figure(2)
grid on
plot3(mp_posi(:,1),mp_posi(:,2),mp_posi(:,3), 'b.','MarkerSize', 10);
hold on
axis equal
pose_inv = inv(pose);
posi_cam= pose_inv(1:3,4);
rot_cam = pose_inv(1:3,1:3)';
cam = plotCamera('Location',posi_cam,'Orientation',rot_cam,'Opacity',0);