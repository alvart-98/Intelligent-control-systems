function robot_animation(t, curr, des)
    figure(1);
    cla('reset');
    axis([-3 3 -3 3]);
    axis equal;
    axis manual;
    hold on;
    grid on;
    plot(des.x(1,:),des.x(2,:),'k');
    
    robot_arm_x = [0 0 0];
    robot_arm_y = [0 0 0];
    robot_arm = plot(robot_arm_x,robot_arm_y,'b');
    
    traj = animatedline('Color','r');

    for iter=1:length(t)
        set(robot_arm,'XData',[0 curr.x_eb(1,iter) curr.x(1,iter)],'YData',[0 curr.x_eb(2,iter) curr.x(2,iter)]);
        addpoints(traj,curr.x(1,iter),curr.x(2,iter));
        drawnow;
        pause(0.03); %pause execution for to make animation visible    
    end
    hold off
end