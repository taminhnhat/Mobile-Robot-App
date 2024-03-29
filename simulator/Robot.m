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
            robot.pos.p = pos.p;
            disp(robot.pos);
            disp(robot.width);
            disp(robot.length);
        end

        function p = show(rb)
            delete(rb.grap);
            rb.grap = rectangle('Position', [rb.pos.x - rb.width / 2 rb.pos.y - rb.length / 2 rb.width rb.length], 'FaceColor', rb.faceColor, 'EdgeColor', rb.edgeColor);
            hold on;
            p = rb.grap;
        end

        function p = getPos(rb)
            p = rb.pos;
        end

        function p = run(rb, des)
            rb.pos.x = des.x;
            rb.pos.y = des.y;
            delete(rb.grap)
            rb.grap = rectangle('Position', [rb.pos.x - rb.width / 2 rb.pos.y - rb.length / 2 rb.width rb.length], 'FaceColor', rb.faceColor, 'EdgeColor', rb.edgeColor);
            hold on;
            p = rb.pos;
        end

        function p = rotate(rb, des)
            rb.pos.a = des.a;
            delete(rb.grap)
            rb.grap = rectangle('Position', [rb.pos.x - rb.width / 2 rb.pos.y - rb.length / 2 rb.width rb.length], 'FaceColor', rb.faceColor, 'EdgeColor', rb.edgeColor);
            hold on;
            p = rb.pos;
        end

        % function des = simulate(rb, endPoint)
        %     t_samp = 0.01;
        %     nx = (endPoint.x - rb.pos.x) / (endPoint.v * t_samp);
        %     ny = (endPoint.y - rb.pos.y) / (endPoint.v * t_samp);
        %     n = abs(nx);

        %     if (abs(ny) > abs(nx))
        %         n = abs(ny);
        %     end

        %     n = round(n);

        %     if (n == 0)
        %         des = rb.show();
        %     elseif (n > 0)
        %         dx = (endPoint.x - rb.pos.x) / n;
        %         dy = (endPoint.y - rb.pos.y) / n;
        %         t = timer('StartDelay', t_samp, 'Period', t_samp, 'TasksToExecute', 2, 'ExecutionMode', 'fixedRate');
        %         t.TimerFcn = @t_callback_fcn;
        %         i = 1;
        %         start(t);

        %         function t_callback_fcn()
        %             if (i == n)delete(t); end
        %             tmp.x = rb.;
        %             tmp.y = path.y(i);
        %             des = rb.run(tmp);
        %             i += 1;
        %         end

        %     else
        %         disp('error');
        %     end

        % end

        function des = simulate(rb, endPoint)
            t_samp = 0.01;
            velocity = endPoint.v;
            nx = (endPoint.x - rb.pos.x) / (velocity * t_samp);
            ny = (endPoint.y - rb.pos.y) / (velocity * t_samp);
            n = abs(nx);

            if (abs(ny) > abs(nx))
                n = abs(ny);
            end

            n = round(n);

            if (n == 0 || n == 1)
                des = rb.run(endPoint);
                hold on;
                rb.pos.x = endPoint.x;
                rb.pos.y = endPoint.y;
            elseif (n > 1)
                dx = (endPoint.x - rb.pos.x) / n;
                dy = (endPoint.y - rb.pos.y) / n;

                for i = 1:n
                    pause(t_samp);
                    tmp.x = rb.pos.x + dx;
                    tmp.y = rb.pos.y + dy;
                    des = rb.run(tmp);
                end

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
