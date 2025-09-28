function R_zyz = euler_angle(th_z1, th_y, th_z2)
    %% This function calculates the Euler angle rotation matrix for given Z-Y-Z angles

    % r_11: Element in the first row, first column of the rotation matrix
    % Calculated as cos(th_z1) * cos(th_y) * cos(th_z2) - sin(th_z1) * sin(th_z2)
    r_11 = cos(th_z1) .* cos(th_y) .* cos(th_z2) - sin(th_z1) .* sin(th_z2);

    % r_21: Element in the second row, first column of the rotation matrix
    % Calculated as sin(th_z1) * cos(th_y) * cos(th_z2) + cos(th_z1) * sin(th_z2)
    r_21 = sin(th_z1) .* cos(th_y) .* cos(th_z2) + cos(th_z1) .* sin(th_z2);

    % r_31: Element in the third row, first column of the rotation matrix
    % Calculated as -sin(th_y) * cos(th_z2)
    r_31 = -sin(th_y) .* cos(th_z2);

    % r_12: Element in the first row, second column of the rotation matrix
    % Calculated as -cos(th_z1) * cos(th_y) * sin(th_z2) - sin(th_z1) * cos(th_z2)
    r_12 = -cos(th_z1) .* cos(th_y) .* sin(th_z2) - sin(th_z1) .* cos(th_z2);

    % r_22: Element in the second row, second column of the rotation matrix
    % Calculated as -sin(th_z1) * cos(th_y) * sin(th_z2) + cos(th_z1) * cos(th_z2)
    r_22 = -sin(th_z1) .* cos(th_y) .* sin(th_z2) + cos(th_z1) .* cos(th_z2);

    % r_32: Element in the third row, second column of the rotation matrix
    % Calculated as sin(th_y) * sin(th_z2)
    r_32 = sin(th_y) .* sin(th_z2);

    % r_13: Element in the first row, third column of the rotation matrix
    % Calculated as cos(th_z1) * sin(th_y)
    r_13 = cos(th_z1) .* sin(th_y);

    % r_23: Element in the second row, third column of the rotation matrix
    % Calculated as sin(th_z1) * sin(th_y)
    r_23 = sin(th_z1) .* sin(th_y);

    % r_33: Element in the third row, third column of the rotation matrix
    % Calculated as cos(th_y)
    r_33 = cos(th_y);

    % Combine all the elements into the rotation matrix R_zyz
    R_zyz = [r_11, r_12, r_13;
             r_21, r_22, r_23;
             r_31, r_32, r_33];
end
