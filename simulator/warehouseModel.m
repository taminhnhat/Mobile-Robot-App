function warehouseModel()
    close all;
    clear all;
    set(gcf, 'Position', [1000, 50, 900, 900])
    axis equal;
    axis([0 28 0 40]);
    xlabel('x(m)'); ylabel('y(m)');
    hold on;

    for i = 1:10
        temp = 3 + 3.6 * (i - 1);
        rectangle('Position', [3 temp 9 0.6], 'FaceColor', [.87 .72 .53]);
    end

    for i = 1:10
        temp = 3 + 3.6 * (i - 1);
        rectangle('Position', [16 temp 9 0.6], 'FaceColor', [.87 .72 .53]);
    end

    rectangle('Position', [13 0 2 40], 'FaceColor', [.7 .7 .7], 'EdgeColor', [.7 .7 .7]);

    for i = 1:10
        rectangle('Position', [0 1 + 3.6 * i 28 1], 'FaceColor', [.7 .7 .7], 'EdgeColor', [.7 .7 .7]);
    end

end
