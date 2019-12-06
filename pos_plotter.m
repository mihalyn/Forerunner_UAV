% Position plotter after the simulation is finished
UGV_pos = out.get('car_pos');
UAV_pos = out.get('uav_pos');

figure();
plot(UGV_pos(:,1), UGV_pos(:,2));
hold on
plot(UAV_pos(:,1), UAV_pos(:,2));
legend('UGV position', 'UAV position');
title('Position of the vehicles');

% Plot the difference in X and Y position
figure();
plot(time, UGV_pos(1:end-1,1)-UAV_pos(1:end-1, 1));
hold on,
plot(time, UGV_pos(1:end-1,2)-UAV_pos(1:end-1, 2));
legend('X difference', 'Y difference');
title('Difference of x-y position');
