function lines = lines(pts, n, wts)
% Transform points and their normal vectors to line parameters
nLines = size(pts,1);
if nargin == 2, wts = ones(nLines,1); end;
lines = [n(:,1:2), -dot(pts,n,2), wts]';