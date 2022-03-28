function [x_RMSE, th_RMSE] = analyze_performance(t, curr, des, output)

    if nargin < 4
        output = true;
    end
    x_err = des.x - curr.x;
    th_err = mod((des.th - curr.th) + pi, 2*pi) - pi; % error mapped to [-pi, pi]
    x_RMSE = mean(x_err(1,:).^2+x_err(2,:).^2)^.5;
    th_RMSE = mean(th_err(1,:).^2+th_err(2,:).^2)^.5;
    
    if output
        fprintf('RMSE x : %d\n',x_RMSE);
        fprintf('RMSE th: %d\n',th_RMSE);
    
        figure(2);
        subplot(3,1,1);
        plot(t,[des.x(1,:)' curr.x(1,:)']);
        ylabel('x1');
        legend('des','curr');
        title('Cartesian Space');
        subplot(3,1,2)
        plot(t,[des.x(2,:)' curr.x(2,:)']);
        ylabel('x2');
        legend('des','curr');
        subplot(3,1,3)
        plot(t,x_err');
        ylabel('x error');
        legend('x1','x2');
        xlabel('t (sec)');

        figure(3);
        subplot(3,1,1);
        plot(t,[des.th(1,:)' curr.th(1,:)']);
        hold on
        plot(t,[des.th(1,:)' wrapToPi(curr.th(1,:))']);
        ylabel('th1');
        legend('des','curr');
        title('Joint Space');
        subplot(3,1,2)
        plot(t,[des.th(2,:)' curr.th(2,:)']);
        hold on
        plot(t,[des.th(2,:)' wrapToPi(curr.th(2,:))']);
        ylabel('th2');
        legend('des','curr');
        subplot(3,1,3)
        plot(t,th_err');
        ylabel('th error');
        legend('th1','th2');
        xlabel('t (sec)');
    end
end