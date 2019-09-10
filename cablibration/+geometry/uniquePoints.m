function p = uniquePoints(p1, p2, minDist)

if nargin < 3, minDist = 1; end;

if isempty(p1)
    p = p2;
else   
    D = pdist2(p2, p1,'euclidean');
    valid = min(D,[],2) > minDist;
    p = p2(valid,:);
end