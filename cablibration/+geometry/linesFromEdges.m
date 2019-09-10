function l = linesFromEdges(pts, mag, grad)

n = size(pts,1);
normal = [cos(grad), sin(grad) zeros(n,1)];
l = geometry.lines(pts, normal, mag);