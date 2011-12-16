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

P =  numel(keyXs);
if ~exist(tracked_pts_f);
    trackedXs = zeros(length(imFiles), P);
    trackedYs = zeros(length(imFiles), P);
    trackedXs(1, :) = keyXs; trackedYs(1, :) = keyYs;
    for i=2:length(imFiles)
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
    pts = randperm(P);
    pts = pts(1:30);

    sfigure; imagesc(imread(imFiles{1})); colormap('gray'); hold on;
    plot(trackedYs(1, pts), trackedXs(1,pts),'y.');
    plot(trackedYs(2, pts), trackedXs(2,pts),'b.');
    for f = 1:F-1
        u = trackedXs(f+1, pts)- trackedXs(f, pts);
        v = trackedYs(f+1, pts)-trackedYs(f, pts);
        uv= [u;v];
        quiver(trackedYs(f, pts),trackedXs(f, pts),uv(2,:), uv(1,:));
    end
end

