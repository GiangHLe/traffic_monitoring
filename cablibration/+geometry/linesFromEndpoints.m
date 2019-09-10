function l = linesFromEndpoints(p1, p2)

assert(all(size(p1) == size(p2)));

if isempty(p1)
    l = [];
else
    n = size(p1,1);
    d = p2 - p1;
    nv = [d(:,2), -d(:,1) zeros(n,1)];
    l = geometry.lines(p1, nv);
end