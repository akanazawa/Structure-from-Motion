function [M S] = do_factorization(Xs, Ys)
%%%%%%%%%%
% do_factorization.m
% Using tracked points, implement affine structure from motion
% procedure described in
% "Shape and Motion from Image Streams under Orthography: a
% Factorization Method" 1992 by Tomasi and Kanade.
%
% INPUT - Xs, Ys: tracked 2D points from sequences in
% format F x P, where F is the number of frames and P is the number
% of points tracked
% OUTPUT - M: 2*F by 3 Motion matrix (Camera movements)
%        - S: 3 by P Shape matrix (3D world coordinates)
%
% ALGORITHM 
%   1. represent the input as a 2F x P measurement matrix W 
%   2. Compute SVD of W = USV'
%   3. Define M' = U_3(S_3)^(1/2), S' = (S_3)^(1/2)V'_3 (U_3 means the
%   first 3 x 3 block, where M' and S' are liner transformations of
%   the actual M and S
%   4. Compute Q by imposing the metric constraints i.e. let L = QQ' 
%   and solve Gl = c for l, use cholseky to recover Q
%   5. Compute M and S using M', S', and Q
%  
% Angjoo Kanazawa 12/14/'11
%%%%%%%%%%

% for the moment use the trustworthy points
load 'supp/tracked_points';

[F P] = size(Xs); 

%%% 0. to eliminate translation, center all points. i.e. subtract
%% mean of each row
Xs = bsxfun(@minus, Xs, mean(Xs, 2));
Ys = bsxfun(@minus, Ys, mean(Ys, 2));

%%% 1. compute W
W = [Xs; Ys];
% W = zeros(2*F, P);
% for f = 1:F
%     W(2*f-1:2*f, :) = [Xs(f, :); Ys(f, :)];
% end

%%% 2. SVD of W
[U D V] = svd(W);

%%% 3. make M', S' 
Mhat = U(:, 1:3)*sqrt(D(1:3, 1:3)); 
Shat = sqrt(D(1:3, 1:3))*V(:, 1:3)';

%%% 4. Compute Q, impose the metric constraints
Is = Mhat(1:F, :);
Js = Mhat(F+1:end, :);


gfun = @(a, b)[ a(1)*b(1), a(1)*b(2)+a(2)*b(1), a(1)*b(3)+a(3)*b(1), ...
              a(2)*b(2), a(2)*b(3)+a(3)*b(2), a(3)*b(3)] ;
G = zeros(3*F, 6);
for f = 1:3*F
    if f <= F
        G(f, :) = gfun(Is(f,:), Is(f,:));
    elseif f <= 2*F
        %        fprintf('do j(%d) ', mod(f, F+1)+1);
        G(f, :) = gfun(Js(mod(f, F+1)+1, :), Js(mod(f, F+1)+1, :));
    else
        %        fprintf('\tdo i,j(%d)', mod(f, 2*F));
        G(f, :) = gfun(Is(mod(f, 2*F),:), Js(mod(f, 2*F),:));
    end
end

c = [ones(2*F, 1); zeros(F, 1)];

% solve Gl = c (do it by svd later)
l = G\c;

% could be a programatic way, but hey we "see" 3D or 2D
L = [l(1) l(2) l(3);...
     l(2) l(4) l(5);...
     l(3) l(5) l(6)] ;

Q = chol(L); % finally!

%fprintf('check %g\n', all(all(L = Q'*Q)));

%%% 5. get M and S
M = Mhat*Q;
S = inv(Q)*Shat;

sfigure;
plot3(Shat(1, :), Shat(2,:), Shat(3,:),'k.'); hold on;
plot3(S(1, :), S(2,:), S(3,:),'b.');
plot3(0,0,0,'gs');
grid on;
title(['3D points from tracked points: before and after eliminating ' ...
       'affine ambiguity upto orthography']);
legend('before enforcing metric constraints',['after enforcing metric ' ...
                    'constraints', 'origin']);

