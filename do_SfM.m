function do_SfM(config_file)
%%%%%%%%%%
% CMSC660 Fall'11 Final Project: Affine Structure from Motion(SfM)
% doSfM.m
% Driver script to do affine SfM, to run, do:
% do_SfM('config'); where 'config' refers to the config.m in this directory
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%

%% Step 1: get initial keypoints

[keyXs, keyYs] = do_getKeypoints(config_file); 

%% Step 2: track features

[trackedXs, trackedYs] = do_trackFeatures(config_file);

%% Step 3: Affine Structure for Motion via Factorization

[M S] = do_factorization(config_file);

