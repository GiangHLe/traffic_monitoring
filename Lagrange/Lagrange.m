function y=lagrange(x,pointx,pointy)

    n=size(pointx,2);
    L=ones(n,size(x,2));
    
    for i=1:n
        for j=1:n
            if (i~=j)
            L(i,:)=L(i,:).*(x-pointx(j))/(pointx(i)-pointx(j));
            end
        end
    end
    y= zeros(1,size(x,2));
    for i=1:n
        y=y+pointy(i)*L(i,:);
        fprintf('%i\n', y)
    end
    
end






