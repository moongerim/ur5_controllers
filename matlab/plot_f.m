function a = plot_f(data, name, dt, s, len)
    
    fig_1 = figure('Name', name);
    subplot(3,2,1);
    grid on;
    hold on;
    plot(data(s:len,1));
    % plot(lin_vell_limit_array(s:len,1));
    set(gca,'XTick',0:100:100*len)
    set(gca,'XTickLabel',0:dt*100:len*100*dt)
    title("q 1")

    subplot(3,2,2);
    grid on;
    hold on;
    plot(data(s:len,2));
    % plot(lin_vell_limit_array(s:len,2));
    set(gca,'XTick',0:100:100*len);
    set(gca,'XTickLabel',0:dt*100:len*100*dt);
    title("q 2 ")

    subplot(3,2,3);
    grid on;
    hold on;
    plot(data(s:len,3));
    % plot(lin_vell_limit_array(s:len,3));
    set(gca,'XTick',0:100:100*len);
    set(gca,'XTickLabel',0:dt*100:len*100*dt);
    title("q 3")

    subplot(3,2,4);
    grid on;
    hold on;
    plot(data(s:len,4));
    set(gca,'XTick',0:100:100*len);
    set(gca,'XTickLabel',0:dt*100:len*100*dt);
    title("q 4")

    subplot(3,2,5);
    grid on;
    hold on;
    plot(data(s:len,5));
    set(gca,'XTick',0:100:100*len);
    set(gca,'XTickLabel',0:dt*100:len*100*dt);
    title("q 5")

    subplot(3,2,6);
    grid on;
    hold on;
    l1 = plot(data(s:len,6));
    set(gca,'XTick',0:100:100*len);
    set(gca,'XTickLabel',0:dt*100:len*100*dt);
    title("q 6")
    saveas(fig_1,name)
end