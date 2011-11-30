%%%%%%%%%%
% CMSC660 Fall'11 Final Project: Affine Structure from Motion(SfM)
% doSfM.m
% Driver script to do affine SfM
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%


IMDIR = 'supp/images/'; % location of all images
OUTDIR = 'supp/'; % where to save mat files
TAU = 1000;
imFiles  = getImageSet(IMDIR); % gets cell array of frames

%% Step 1: get initial keypoints
fprintf('getting intial keypoints from %s\n', imFiles{1});
[keyXs, keyYs]= getKeypoints(imread(imFiles{1}), TAU);    

% sfigure;
% imagesc(imread(imFiles{1})); colormap('gray'); hold on;
% plot(keyYs, keyXs, 'y.');
% title(['first frame overlayed with keypoints']);

%% Step 2: track features
% for each keypoint at frame f, I(x,y,f), we want to compute expected translation
% in the next frame I(x', y', f+1)
numPoints =  numel(keyXs);
% if ~exist('tracked_points.mat');
    trackedXs = zeros(length(imFiles), numPoints);
    trackedYs = zeros(length(imFiles), numPoints);
    trackedXs(1, :) = keyXs; trackedYs(1, :) = keyYs;
    for i=2:length(imFiles)
        [trackedXs(i,:) trackedYs(i,:)] = predictTranslationAll(trackedXs(i-1, :), trackedYs(i-1, :),...
                                                          imread(imFiles{i-1}), imread(imFiles{i}));
    end
    save('tracked_points.mat', 'trackedXs', 'trackedYs');
% else
%     load('tracked_points.mat');
% end

% remove nans i.e. points that went out of frame
outFrame = find(isnan(trackedXs(end, :)));
trackedXs(:, outFrame) = [];
trackedYs(:, outFrame) = [];
numPoints = numPoints - numel(outFrame);
% to draw path from one frame to another:

% X1toX2 = [keyXs trackedXs(2,:)'];
% Y1toY2 = [keyYs trackedYs(2,:)'];
%line(Y1toY2', X1toX2', 'color', 'b', 'LineStyle', '-.');

sfigure; imagesc(imread(imFiles{1})); colormap('gray'); hold on;
pt1 =  [trackedYs(1,:)' trackedXs(1,:)' ones(numPoints, 1)];
pt2 =  [trackedYs(2,:)' trackedXs(2,:)' ones(numPoints, 1)];
arrow(pt1, pt2, 'Length', 4, 'Width', 1, 'EdgeColor', 'y', ...
      'FaceColor', 'b');
