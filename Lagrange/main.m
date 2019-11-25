accept = true;
while accept
    promt = 'Please enter list of x for interpolation Lagrange polynomial:';
    x_in = input(promt);
    promt = 'Please enter list of y for interpolation Lagrange polynomial:';
    y_in = input(promt);
    x_length = size(x_in,2);
    y_length = size(y_in,2);
    if x_length~=y_length
        fprintf('x and y must have same length, please enter again!!!\n');
    else
        accept = false
    end
end
promt = 'Please enter x you want to query:';
x = input(promt);
y = Lagrange(x,x_in,y_in);
fprintf('Your result is: %i')
disp(y)

low_x_show = min(x) - 30;
high_x_show = max(x) + 30;
x_range = [low_x_show:1:high_x_show]
y_range = Lagrange(x_range,x_in,y_in)
% subplot(x,y,'x')
plot(x_range,y_range, x,y,'x')
set(get(gca, 'Title'), 'String', 'Graph of function');
set(get(gca, 'XLabel'), 'String', 'x value');
set(get(gca, 'YLabel'), 'String', 'y_value');