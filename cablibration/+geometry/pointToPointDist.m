function m = pointToPointDist(p1, p2)
d = p1-p2;
m = sqrt(dot(d,d,2));