filename = 'turtlebot1_discrete_filter_odom.csv';
Odom = readtable(filename);
x = Odom.x_position;
y = Odom.y_position;
filename = 'turtlebot1_comp_data.csv';
Comp = readtable(filename);
z = Comp.prob;

hold all
scatter3(x,y,z,[],z)
scatter3(x,y,zeros([length(z),1]))
for ind = 1:length(x)
   line([x(ind),x(ind)],[y(ind),y(ind)],[0,z(ind)])
end

axis([-1 11 -5 1 0 1])
view(-10,30)