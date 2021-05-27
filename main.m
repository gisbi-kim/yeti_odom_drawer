log = readmatrix("accuracy_num.csv");

dx = log(:, 9);
dy = log(:, 10);
dh = log(:, 11); % h for heading 

curr_x = 0.0;
curr_y = 0.0;
curr_h = 0.0;

curr_SE3 = [ cos(curr_h) -sin(curr_h) 0 curr_x;
             sin(curr_h)  cos(curr_h) 0 curr_y;
             0            0           1 0; 
             0            0           0 1];

traj_SE3 = {curr_SE3};
traj_x = [curr_x]; 
traj_y = [curr_y];
for ii=1:length(dx)
    delta_SE3 = [cos(dh(ii)) -sin(dh(ii)) 0 dx(ii);
                 sin(dh(ii))  cos(dh(ii)) 0 dy(ii);
                 0            0           1 0; 
                 0            0           0 1];
    curr_SE3 = curr_SE3 * delta_SE3;

    traj_SE3{end+1} = curr_SE3;
    traj_x(end+1) = curr_SE3(1,4);
    traj_y(end+1) = curr_SE3(2,4);
end

figure(1); clf;
traj_xyz = [traj_x', traj_y', zeros(size(traj_x'))];
traj_pc = pointCloud(traj_xyz);
traj_pc.Intensity = linspace(1, length(traj_xyz), length(traj_xyz))';

pcshow(traj_pc, 'MarkerSize', 400);
view(-270, -90);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

