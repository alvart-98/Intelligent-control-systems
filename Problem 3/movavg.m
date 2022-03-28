function y = movavg(x, wdw)

    if size(x, 1) > 1
        x = [repmat(x(1, :), wdw, 1);x;repmat(x(end,:), wdw, 1)];
    end
    
    if size(x, 2) > 1
        x = [repmat(x(:, 1), 1, wdw),x,repmat(x(:,end), 1, wdw)];
    end

    y = conv2(x, ones(wdw, wdw)/wdw^sum(size(x)>1), 'same');
    
    if size(x, 1) > 1
        y = y(wdw+1:end-wdw,:);
    end
    if size(x, 2) > 1
        y = y(:,wdw+1:end-wdw);
    end
        
    