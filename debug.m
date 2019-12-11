close all;
%clear all;
a_ref = out.get('a_ref');
t0 = out.get('t0');
car_a = out.get('car_acc');
car_v = out.get('car_vel');
car_pos = out.get('car_pos');
uav_a = out.get('uav_acc');
uav_v = out.get('uav_vel');
uav_pos = out.get('uav_pos');

Radius = out.get('R');
accel_cent = out.get('accel_cent');

%% accel ref
figure();
plot(a_ref.time, a_ref.data(:,1));
hold on
plot(a_ref.time, a_ref.data(:,2));
plot(a_ref.time, car_a.data(:,1));
plot(a_ref.time, car_a.data(:,2));
legend('ref a_x','ref a_y', 'car a_x', 'car a_y');
ylim([-0.05, 0.05])
% figure();
% plot(a.time, a.data(:,3));
% legend('a_z');

%% accel
figure();
plot(uav_a.time, uav_a.data(:,1));
hold on
plot(uav_a.time, uav_a.data(:,2));
plot(car_a.time, car_a.data(:,1));
plot(car_a.time, car_a.data(:,2));
ylim([-0.05, 0.05])
legend('uav a_x','uav a_y', 'car a_x', 'car a_y');

%% vel
figure();
plot(uav_v.time, uav_v.data(:,1));
hold on
plot(uav_v.time, uav_v.data(:,2));
plot(car_v.time, car_v.data(:,1));
plot(car_v.time, car_v.data(:,2));
ylim([-2, 2])
legend('uav v_x','uav v_y', 'car v_x', 'car v_y');

%% pos
figure();
plot(car_pos(:,1), car_pos(:,2));
hold on
plot(uav_pos(:,1), uav_pos(:,2));
ylim([-30, 30])
xlim([-30, 30])
grid;
pbaspect([1 1 1])

%% R, accel_cent
figure()
plot(Radius.time, Radius.data);
ylim([0 10])
grid

figure()
plot(accel_cent.time, accel_cent.data);

%%
% figure();
% plot(t0);
% 
% tmax = 5;
% traj_res = 3;
% ts = 0.025;
% xyz = [0.5*cos(linspace(0,pi/2,traj_res)); ...
%     0.5*sin(linspace(0,pi/2,traj_res)); 10*ones(1,traj_res)];
% 
% [s1, sd1, sdd1] = qc_trajgen(xyz, zeros(3,1), 0.5, ts);
% 
% 
% plot(xyz(1,:), xyz(2,:), 'x')
% hold on
% fnplt(s1)
% fnplt(sd1)
% fnplt(sdd1)
