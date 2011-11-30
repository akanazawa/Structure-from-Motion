function [newXs newYs] = predictTranslationAll(startXs, startYs,im0,im1);
%%%%%%%%%%
% implementation of the KLT tracker introduced in: 
% Carlo Tomasi and Takeo Kanade. Detection and Tracking of Point Features. Carnegie Mellon University Technical Report CMU-CS-91-132, April 1991.
% predictTranslationAll.m
% script to get new X, Y, locations in im1 for all startXs and
% startYs in im0
%
% Computes the gradients here, calls predictTranslation.m to get
% predict eahc keypoint independently
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%

if ~isa(im0, 'double')  im0 = double(im0); end
if ~isa(im1, 'double')  im1 = double(im1); end

% Using the brightness constancy assumption, we expect the pixel
% intensity of location x, y at frame f is same as the pixel
% instensiy of location x'=x+u, y'=y+v at frame f+1
% i.e. I(x,y,f) = I(x', y', f+1), where u and v are displacement of
% pixels in the next frame.

% With an additional constraint that this must be true within w by w window, this amounts to solving the LSP:
% -I_t(ps) = grad I(ps)[u; v] => Ax = b
% where A = grad I(ps), b = -I_t(ps), x = [u;v]
% - ps are all points in the w by w window
% - I_t is the temporal gradient: I(x'y', f+1) - I(x,y,f)

%% Step 1 compute the gradient of im0 

[Ix Iy] = derivative5(im0, 'x', 'y'); 
numPoints = length(startXs);
newXs = zeros(numPoints, 1); newYs =  zeros(numPoints, 1);
for i=1:numPoints
    fprintf('.');
    if ~isnan(startXs(i)) || ~isnan(startYs(i))
        [newX newY] = predictTranslation(startXs(i), startYs(i), Ix, Iy, ...
                                     im0, im1);
    else
        newX = nan; newY= nan;
    end
    newXs(i) = newX; 
    newYs(i) = newY;
    end
end

