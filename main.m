clc;clear;close all;

%% 从图像处理结果的txt中导入各个图像出现的特征点及其序号

% img_location = 'E:\CheckboardLocalization-067110c2eea95957c57943350b78819d8b247793\CheckboardLocalization\Registration\';
% idx = 1:1:11;
% idx_bias = 0;

% img_location = '.\registration_image\group1\';
% idx = 2:1:26;
% idx_bias = 1;

img_location = '.\registration_image\group2\';
idx = 221:1:235; % 241;
idx_bias = 220;

% points是个cell数组，里面存放了各个图像内提取出的特征点及其序号
points = {};

for i = idx
    fid = fopen(strcat(img_location,num2str(i,'%.2d'),'.txt'),'r');
    markers = [];
    while ~feof(fid)
        str = fgetl(fid);   % 读取一行, str是字符串
        split_list = strsplit(str,{':',' '});
        if strcmp(split_list{1},'matrix_coordinate')
            markers = [markers; [str2double(split_list{2}),str2double(split_list{4}),str2double(split_list{5})]];
        end
    end
    fclose(fid);
    points{i-idx_bias,1} = markers;
end

%% 特征点序号可视化

% img_idx = 5;
% img = imread(strcat(img_location,num2str(img_idx,'%.2d'),'.bmp'));
% img = insertText(img, points{img_idx-idx_bias}(:,2:3), points{img_idx-idx_bias}(:,1),'FontSize',8);
% img = insertMarker(img, points{img_idx-idx_bias}(:,2:3), 'o', 'Color', 'red', 'Size', 5);
% imshow(img);

%% 计算各个序号特征点的世界座标系坐标(依据测量结果)
% D = 10; % unit: mm 笔的直径
% L = 24; % unit: mm 贴纸边长
% N = 12; % 图案边长有多少格
% Np = N-1; % 图案边长有多少点
% 
% % 计算中间量
% d = L/N; % unit: mm 一个格子有多长
% Round = pi*D; % unit: mm 笔的横截面周长
% Angle_full = 2*pi*L/Round; % unit: rad 整张贴纸的边长(12格)对应于多少弧度
% Angle_grid = Angle_full/12; % unit: rad 贴纸边长的一格对应于多少弧度
% 
% points_world = zeros(Np,Np,4);
% points_world(:,:,1) = reshape(1:1:Np*Np,[Np,Np]);
% points_world(:,:,2) = repmat((0:1:(Np-1))'*Angle_grid ,1,Np);
% points_world(:,:,2) = 0.5*D*cos(points_world(:,:,2));
% points_world(:,:,3) = repmat((0:1:(Np-1))'*Angle_grid ,1,Np);
% points_world(:,:,3) = 0.5*D*sin(points_world(:,:,3));
% points_world(:,:,4) = repmat([0:1:(Np-1)]*d, Np,1);
% points_world = reshape(points_world,[Np*Np,1,4]);
% points_world = permute(points_world,[1,3,2]);
% 
% % 世界坐标可视化
% plot3(points_world(:,2),points_world(:,3),points_world(:,4),'o');
% text(points_world(:,2),points_world(:,3),points_world(:,4),num2str(points_world(:,1)));

%% 加载标定参数

load('.\CameraParams.mat');

%% 准备pointTracks

full_set = {};
for i = 1:1:length(points)
    for j = 1:1:size(points{i},1)
        if isempty(full_set)
            full_set = [full_set; points{i}(j,1),{i}];
        else
            flag = 0;
            for k = 1:1:size(full_set,1)
                if full_set{k,1} == points{i}(j,1)
                    full_set{k,2} = [full_set{k,2} i];
                    flag = 1;
                    break;
                end
            end
            if flag == 0
                full_set = [full_set; points{i}(j,1),{i}];
            end
        end
    end
end

pointTracks = [];
pointIdx = [];
for i = 1:1:size(full_set,1)
    vid = full_set{i,2};
    pt = [];
    for j = 1:1:length(vid)
        pt = [pt; points{vid(j),1}(points{vid(j),1}(:,1)==full_set{i,1},2:3)];
    end
    pointTracks = [pointTracks pointTrack(vid,pt)];
    pointIdx = [pointIdx;full_set{i,1}];
end

%%
prevPointsLabels = points{1}(:,1);
prevPoints = points{1}(:,2:3);
vSet = viewSet;
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints), 'Location', ...
    zeros(1, 3, 'like', prevPoints));

for i = 2:numel(points)
% for i = 2:9
    
    % Detect, extract and match features.
    currPointsLabels = points{i}(:,1);
    currPoints = points{i}(:,2:3);
    
    % Select matched points.
    [~,iPrev,iCurr] = intersect(prevPointsLabels,currPointsLabels);
    matchedPoints1 = prevPoints(iPrev,:);
    matchedPoints2 = currPoints(iCurr,:);
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', [iPrev iCurr]);
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevPointsLabels   = currPointsLabels; 
    prevPoints   = currPoints;  
end

%% Display Camera Poses
% Display the refined camera poses and 3-D world points.
% 这一部分代码直接抄的StructureFromMotionFromMultipleViewsExample.m

camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');