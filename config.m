%%%%%%%%%%%%%%%%%%%%
% The global configuration file 
% Holds all settings used in all parts of the affine Structure from Motion project,
% enabling the exact reproduction of the experiment at some future
% date.
%
% Use this file to set various options
% 
% Angjoo Kanazawa

%%%%%
% DIRECTORIES
%%%%%

% Directory holding the experiment 
RUN_DIR = [ 'Structure-from-Motion/' ];

% Directory holding all the source images
IMAGE_DIR = [ 'supp/images' ];

% Data directory - holds all intermediate .mat files
DATA_DIR = [ 'data/' ];   


%----------
%% EXPERIMENT SETTINGS
%----------
VERBOSE = 0;

%% Featuren Extraction method currently 'harris' or 'sift'
Feature.method = 'harris'; %'sift';

if strcmp(Feature.method, 'harris')
    Feature.alpha = 0.05; % corner response size
    Feature.radius = 5; % window size
    Feature.sigma = 3; % size of the gaussian filter
    Feature.tau = 1000; % threshold for non-maxima supression
else
    
end

%% FILE NAMES
% name to save the tracked files
tracked_pts_f = [DATA_DIR, Feature.method, '_tracked_points.mat'];
% name to save the initial key points
keypoints_f = [DATA_DIR, Feature.method, '_keypoints.mat'];
