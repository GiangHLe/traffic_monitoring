function p = normalizePoints(p, o, Z, backward)

assert(ismatrix(p));
if nargin == 3, backward = false; end;
[nPts,d] = size(p);

if d == 2, p = [p, ones(nPts,1)]; end;

regular = p(:,3) > 1e-5;
p(regular,:) = bsxfun(@rdivide, p(regular,:), p(regular,3));

if ~backward
    p(regular,1:2) = bsxfun(@minus, p(regular,1:2), o) ./ Z;
else
    p(regular,1:2) = bsxfun(@plus, p(regular,1:2).*Z, o);
end

