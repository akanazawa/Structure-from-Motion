function do_SfM(config_file)
%%%%%%%%%%
% CMSC660 Fall'11 Final Project: Affine Structure from Motion(SfM)
% doSfM.m
% Driver script to do affine SfM
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%

%% Evaluate the global configuration file and load parameters
eval(config_file);

imFiles  = getImageSet(IMAGE_DIR); % gets cell array of frames
F = length(imFiles); % number of frames

%% Step 1: get initial keypoints
[keyXs, keyYs] = do_getKeypoints(config_file); 

%% Step 2: track features

[trackedXs, trackedYs] = do_trackFeatures(config_file);


%% Step 3: Affine Structure for Motion via Factorization

[M S] = do_factorization(config_file);

