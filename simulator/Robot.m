classdef Robot

    properties (Constant)
        rWidth = 0.5;
        rLength = 0.6;
        edgeColor = [0 0 0];
    end

    properties (Access = private)
        faceColor {mustBeInteger} = [0 0 0];
        pos
    end

    methods (Access = public)

        function robot = Robot(robot)
            disp(Robot.rWidth);
            disp(robot.faceColor);
            % faceColor(1) = faceClr(1) / 255;
            % faceColor(2) = faceClr(2) / 255;
            % faceColor(3) = faceClr(3) / 255;
            % this.pos.x = pos.x;
            % this.pos.y = pos.y;
        end

        function p = show(pos)
            p = rectangle('Position', [pos.x - rWidth / 2 pos.y - rLength / 2 rWidth rLength], 'FaceColor', this.faceColor, this.edgeColor, [0 0 0]);
            hold on;
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
