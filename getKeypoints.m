function [keyXs, keyYs] = getKeypoints(im, tau)
%%%%%%%%%%
% getKeypoints.m
% script to get the key points from im using Harris corner detector
% INPUT - im: image to get the keypoint
%         tau: threshold to do non-maxima supression
%
% parts of script referenced from: http://www.csse.uwa.edu.au/~pk/research/matlabfns/
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%
im = double(im);
alpha = 0.05;
radius = 5; % 5 x 5 window
sigma = 3; 

% compute image derivatives in x and y using Peter Kovesi's
% accurate derivative
[Ix Iy] = derivative5(im, 'x', 'y');
% old way
% dx = [ -1 0 1 ; -1 0 1 ; -1 0 1]; 
% dy = dx';
% Ix = imfilter(im, dx, 'same');
% Iy = imfilter(im, dy, 'same');

% compute components of H
Ix2 = Ix.^2;
Iy2 = Iy.^2;
IxIy = Ix.*Iy ; 

% smooth H using gaussian filter
filt = fspecial('gaussian', 6*sigma, sigma);
Ix2sm = imfilter(Ix2, filt, 'same');
Iy2sm = imfilter(Ix2, filt, 'same');
IxIysm = imfilter(IxIy, filt, 'same');

% display plot
% sfigure; subplot(2,2,1); imagesc(im); colormap('gray'); title('original');
% subplot(2,2,2); imagesc(Ix); colormap('gray');title('der in x');
% subplot(2,2,3); imagesc([Ix2 IxIy; IxIy Iy2]);colormap('gray'); title('hessian');
% subplot(2,2,4); imagesc([Ix2sm IxIysm; IxIysm Iy2sm]);colormap('gray'); title('smoothed hessian');

% compute the corner response matrix = det(H) - a*trace(H)^2
M = Ix2sm.*Iy2sm - IxIy.^2 - alpha*(Ix2sm + Iy2sm).^2;

% perform non-maxima supression over radius window size

% Make mask to exclude points within radius of the image boundary. 
bordermask = zeros(size(im));
bordermask(radius+1:end-radius, radius+1:end-radius) = 1;
% dilate image
M_sup = ordfilt2(M, radius^2, ones(radius));
% find points that's still there in dilated image & stronger than tau
corner = (M==M_sup) & M>tau & bordermask;

% plot
% sfigure; subplot(221); imagesc(M); title('corner response M');
% subplot(222); imagesc(M_sup); title('max dilated');
% colormap('gray');
% subplot(223); imagesc(M> tau); title(['corner response > thresh']);
% colormap('gray');
% subplot(224); imagesc(corner); title('corner response max suppressed');
% colormap('gray');

[keyXs, keyYs] = find(corner); % get the r, c index of key points

