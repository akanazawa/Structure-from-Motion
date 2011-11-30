function [newX newY] = predictTranslation(startX, startY,Ix, Iy,im0,im1);
%%%%%%%%%%
% implementation of the KLT tracker introduced in: 
% Carlo Tomasi and Takeo Kanade. Detection and Tracking of Point Features. Carnegie Mellon University Technical Report CMU-CS-91-132, April 1991.
% predictTranslation.m
% For a single X Y location, use Ix and Iy, im0 and im1 to compute
% the new location X', Y' iteratively using Newton-Raphson style iteration
%
% Angjoo Kanazawa 11/23/'11
%%%%%%%%%%

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

WINDOW = 15; 
% ignore points that are outside or close to the border (within 3)
radius = 3;
bordermask = zeros(size(im0));
bordermask(radius+1:end-radius, radius+1:end-radius) = 1;

% all points in the window x window 
% make A: [sum_w Ix*Ix sum_w Ix*Iy; sum_w Ix*Iy sum_w Iy*Iy]

% get the indices of the window grid we want to look at
[x_w, y_w] = meshgrid(startX-WINDOW:startX+WINDOW, startY-WINDOW:startY+WINDOW);
Img1_w = interp2(im0, x_w, y_w);    
Img2_w = interp2(im1, x_w, y_w);    
Ix_w = interp2(Ix, x_w, y_w);
Ix_w = Ix_w(~isnan(Ix_w));
Iy_w = interp2(Iy, x_w, y_w);
Iy_w = Iy_w(~isnan(Iy_w));

Ixy_w = interp2(Iy.*Ix, x_w, y_w);
Ixy_w = Ixy_w(~isnan(Ixy_w));

A = [sum(Ix_w.^2) sum(Ixy_w); sum(Ixy_w) sum(Iy_w.^2)];

%% get It = I(x', y', t+1) - (x,y,t+1)
% iteratively so the first one is, (x0', y0') = (x,y)

diff = norm(Img2_w- Img1_w);
dx = 100;
uv = [0;0]; %(x',y') starts at (x0,y0) 
itr = 0;
maxItr = 30;
while dx > 0.01 & itr < maxItr
    Img2_w_new = interp2(im1, x_w+uv(1), y_w+uv(2));
    It = Img2_w_new - Img1_w;    
    % remove if point/window (X+uv) moves out of frame
    if length(find(isnan(It)))>0
        uv = [NaN; NaN];
        break;
    end
    % calculate b = - [sum_w IxIt sum_w IyIt]
    b = - [sum(Ix_w.*It(:)); sum(Iy_w.*It(:))];
    % estimate (u,v)
    uv_new = A\b;
    % update displacement
    uv = uv+uv_new;
    itr = itr + 1;
    diffNew =norm(Img2_w - Img2_w_new);
    if diff < diffNew
        break;
    end
    dx = abs(diff - diffNew);
    diff = diffNew;
end
if isnan(uv)
    newX = nan; newY=nan;
else 
    %maybe not needed but check if this computed uv sets key point
    %out of frame
    x_int = interp2(im1, startX+uv(1), startY+uv(2));
    if isnan(x_int)
        newX = nan; newY=nan;
        fprintf('\nout of frame!\n');
    else
        newX = startX+uv(1); newY = startY+uv(2);
    end
end
