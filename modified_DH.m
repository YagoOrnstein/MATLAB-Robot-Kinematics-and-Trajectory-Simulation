function [MDH] = modified_DH()
    d = [60, 0, 0, 130, 0, 90]';
    a = [0, 0, 150, 70, 0, 0]';
    alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2]';
    MDH = [d, a, alpha];
end
