function im = show(obj, im)

if isempty(im)
    im = zeros(obj.frameSize([2,1]), 'uint8');
end
%%{
h_left = cross(obj.horizon, [1,0,0]);
h_right = cross(obj.horizon, [1,0,-size(im,2)]);
hx = [h_left(1)/h_left(3), h_right(1)/h_right(3)];
hy = [h_left(2)/h_left(3), h_right(2)/h_right(3)];
if all(isfinite([hx,hy]))
    im = insertShape(im, 'Line', [hx(1),hy(1),hx(2),hy(2)],'Linewidth',4,'Color','yellow');
end
%}
im = insertShape(im, 'circle', [obj.pp,5], 'LineWidth',1,'color','green');

im = insertShape(im, 'circle', [obj.vp{1}(1:2), 5], 'LineWidth', 3, 'Color', 'red');
%%{
im = insertShape(im, 'circle', [obj.vp{2}(1:2), 5], 'LineWidth', 3, 'Color', 'green');
if isfinite(obj.vp{3}(1:2))
    im = insertShape(im, 'circle', [obj.vp{3}(1:2), 5], 'LineWidth', 3, 'Color', 'blue');
end
%}
[x,y] = meshgrid(linspace(1,size(im,2),10),linspace(1,size(im,1),10));
lines = cell(numel(x),3);
for i = 1:numel(x)
    for j = 1:3
        d = 50 * normr([obj.vp{j}(1)-x(i), obj.vp{j}(2)-y(i)]);
        lines{i,j} = [x(i), y(i) x(i)+d(1), y(i)+d(2)];
    end
end
lines = vertcat(lines{:});
clrs = vertcat(repmat([255,0,0],numel(x),1),repmat([0,255,0],numel(x),1),repmat([0,0,255],numel(x),1));
im = insertShape(im, 'Line', lines, 'Color', clrs, 'LineWidth', 2);

im = insertText(im, [10,10], sprintf('f = %.2f', obj.focal),'FontSize', 18, 'BoxColor', 'white', 'BoxOpacity', 1, 'TextColor', 'black');
