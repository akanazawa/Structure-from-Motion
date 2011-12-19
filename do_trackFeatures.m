function [trackedXs, trackedYs] = do_trackFeatures(config_file)
%%%%%%%%%%
% do_trackFeatures.m
% Top level file to track freatures from the key points obtained in
% do_getKeypoints.m
% OUTPUT - trackedXs, trackedYs: the tracked points. Saves the
% result in the file specified by config.m
%
% DESCRIPTION 
% for each keypoint at frame f, I(x,y,f), we want to compute
% expected translation in the next frame I(x', y', f+1)
%
% Angjoo Kanazawa 12/16/'11
%%%%%%%%%%

%% Evaluate the global configuration file and load parameters
eval(config_file);

% load the data computed in do_getKeypoints.m
load(keypoints_f, 'keyXs', 'keyYs');

% gets cell array of image file names (frames)
imFiles  = getImageSet(IMAGE_DIR); 
F = length(imFiles);
P =  numel(keyXs);
if ~exist(tracked_pts_f);
    trackedXs = zeros(F, P);
    trackedYs = zeros(F, P);
    trackedXs(1, :) = keyXs; trackedYs(1, :) = keyYs;
    for i=2:F
        [trackedXs(i,:) trackedYs(i,:)] = predictTranslationAll(trackedXs(i-1, :), trackedYs(i-1, :),...
                                                          imread(imFiles{i-1}), imread(imFiles{i}));
    end
    % remove nans i.e. points that went out of frame
    outFrame = find(isnan(trackedXs(end, :)));
    trackedXs(:, outFrame) = [];
    trackedYs(:, outFrame) = [];
    P = P - numel(outFrame);
    save(tracked_pts_f, 'trackedXs', 'trackedYs', 'P');
else
    load(tracked_pts_f);
end


%% Draw the path of random 30 tracked points over all frames
if VERBOSE
    % frame 1, 15, 30, and 45
    sfigure; subplot(141); imagesc(imread(imFiles{1}));colormap('gray');
    axis off;  title('frame 1');
    subplot(142); imagesc(imread(imFiles{15})); colormap('gray');
    axis off;  title('frame 15');
    subplot(143); imagesc(imread(imFiles{30})); colormap('gray');
    axis off;  title('frame 30');
    subplot(144); imagesc(imread(imFiles{45})); colormap('gray');
    axis off;  title('frame 45');
    suptitle('frame 1, 15, 30, and 45 of the hotel sequence');

    pts = randperm(P);
    pts = pts(1:30);

    sfigure; imagesc(imread(imFiles{1})); colormap('gray'); hold on;
    plot(trackedXs(1, pts), trackedYs(1,pts),'bo');
    for f=2:F
        plot(trackedXs(f, pts), trackedYs(f,pts), 'b.');
    end
    title('the path of random 30 points tracked over all frames');
    % for f = 1:F-1
    %      u = trackedXs(f+1, pts)- trackedXs(f, pts);
    %      v = trackedYs(f+1, pts)-trackedYs(f, pts);
    %      uv= [u;v];
    %      quiver(trackedYs(f, pts),trackedXs(f, pts),uv(2,:), uv(1,:));
    % end
end

