function [pts,mag,grad] = getVP2EdgePixels(obj, frame, Bkg_mask, Mag, Grad)
% Get edges that are on moving objects
%
% What can be tuned:
% * canny threshold
% * bkg threshold

%Mag_mask = edge(frame,'canny');
Mag_mask = obj.edgeDetector.step(frame);

VP1mask = abs(cos(Grad).*obj.gradientMask(:,:,1) + sin(Grad).*obj.gradientMask(:,:,2)) < 0.95;

% Da se vyhodit, kdyz jsou pekna videa (zepredu, vysoko nad vozovkou)
VP2mask = zeros(size(VP1mask));
if(obj.vp{1}(2) > -99 && obj.vp{1}(2) < obj.frameSize(2) + 99 )
    lb = round(max(1,round(obj.vp{1}(2) - 100)));
    ub = round(min(obj.frameSize(2), obj.vp{1}(2) + 100));
    VP2mask(lb:ub,:) = 1;
end

VP3mask = abs(cos(Grad)) < 0.6;

Q = (Bkg_mask & Mag_mask & VP3mask & (VP1mask | VP2mask));

Q1 = conv2(single(Q),ones(3),'same');
Q = Q & (Q1 > 2);

eidx = find(Q == 1 & (Mag > 1));
[r,c] = ind2sub(size(Q), eidx);
pts = [c,r];
mag = Mag(eidx);
grad = Grad(eidx);

[~,ord] = sort(mag,'descend');
n = min(length(mag),100); n = ord(1:n);
pts = pts(n,:);
mag = mag(n,:);
grad = grad(n,:);
