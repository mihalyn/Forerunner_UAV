close all;
a = out.get('a');
t0 = out.get('t0');

figure();
plot(a.time, a.data(:,1));
legend('a_x');

figure();
plot(a.time, a.data(:,2));
legend('a_y');

% figure();
% plot(a.time, a.data(:,3));
% legend('a_z');

% figure();
% plot(t0);