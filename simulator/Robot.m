classdef Robot < handle

    properties (Constant)
        width = 0.5;
        length = 0.6;
        edgeColor = [0 0 0];
    end

    properties (Access = private)
        id {} = "";
        faceColor = [0 0 0];
        pos
        grap
    end

    methods (Access = public)

        function robot = Robot(id, color, pos)
            robot.id = id;
            disp(robot.id);
            robot.faceColor(1) = color(1) / 255;
            robot.faceColor(2) = color(2) / 255;
            robot.faceColor(3) = color(3) / 255;
            disp(robot.faceColor);
            robot.pos.x = pos.x;
            robot.pos.y = pos.y;
            disp(robot.pos);
            disp(robot.width);
            disp(robot.length);
        end

        function p = show(rb)
            p = rectangle('Position', [rb.pos.x - rb.width / 2 rb.pos.y - rb.length / 2 rb.width rb.length], 'FaceColor', rb.faceColor, 'EdgeColor', rb.edgeColor);
            hold on;
        end

        function p = getPos(rb)
            p = rb.pos;
        end

        function p = run(rb, pos)
            rb.pos.x = pos.x;
            rb.pos.y = pos.y;
            p = rectangle('Position', [rb.pos.x - rb.width / 2 rb.pos.y - rb.length / 2 rb.width rb.length], 'FaceColor', rb.faceColor, 'EdgeColor', rb.edgeColor);
            hold on;
        end

        function des = simulate(rb, endPoint)
            t_samp = 0.001;
            velocity = endPoint.v;
            nx = (endPoint.x - rb.pos.x) / (velocity * t_samp);
            ny = (endPoint.y - rb.pos.y) / (velocity * t_samp);
            n = abs(nx);

            if (abs(ny) > abs(nx))
                n = abs(ny);
            end

            n = round(n);
            p = rb.show();

            if (n == 0 || n == 1)
                delete(p);
                des = rb.run(endPoint);
                hold on;
                result.x = endPoint.x;
                result.y = endPoint.y;
            elseif (n > 1)
                dx = (endPoint.x - rb.pos.x) / n;
                dy = (endPoint.y - rb.pos.y) / n;
                path.x(1) = rb.pos.x + dx;
                path.y(1) = rb.pos.y + dy;
                pause(t_samp);

                for i = 2:n
                    path.x(i) = path.x(i - 1) + dx;
                    path.y(i) = path.y(i - 1) + dy;
                    delete(p);
                    tmp.x = path.x(i);
                    tmp.y = path.y(i);
                    p = rb.run(tmp);
                    hold on;
                    pause(t_samp);
                end

                delete(p);
                rb.pos.x = path.x(n);
                rb.pos.y = path.y(n);
                des = rb.pos;
            else
                disp("error");
            end

        end

        function i = info(obj)
            disp(obj.id);
            disp(obj.faceColor);
        end

    end

end

% p1.x = 6; p1.y = 10;
% p2.x = 8; p2.y = 10;
% p3.x = 10; p3.y = 10;
% R01 = Robot([235 160 28], p1);
% R02 = Robot([47 120 115], p2);
% R03 = Robot([11 48 74], p3);
% R01.show();
% R02.show();
% R03.show();
