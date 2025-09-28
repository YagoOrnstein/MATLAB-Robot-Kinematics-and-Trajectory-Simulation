function [q] = custom_inverse_kinematics(Ti)
    %% Modified D-H Model
    d1 = 60;
    d2 = 0;
    d3 = 0;
    d4 = 130;
    d5 = 0;
    d6 = 90;
    
    a0 = 0;
    a1 = 0;
    a2 = 150;
    a3 = 70;
    a4 = 0;
    a5 = 0;
    
    alpha1 = 0;
    alpha2 = -pi/2;
    alpha3 = 0;
    alpha4 = -pi/2;
    alpha5 = pi/2;
    alpha6 = -pi/2;
    nx = Ti(1,1);
    ny = Ti(2,1);
    nz = Ti(3,1);
    ox = Ti(1,2);
    oy = Ti(2,2);
    oz = Ti(3,2);
    ax = Ti(1,3);
    ay = Ti(2,3);
    az = Ti(3,3);
    px = Ti(1,4);
    py = Ti(2,4);
    pz = Ti(3,4);
    tol = 1e-8; % Set the tolerence

    %% for theta 1
    th1_1 = atan2(py-90*ay,px-90*ax);
    th11 = th1_1 - pi;
    if abs(th11) < tol
        th1 = 0;
    elseif th1_1 < 0
    
        th1 = pi+th1_1;
    else
        th1 = th1_1;
    end
       
    %% for theta2
    if abs(th1) < tol
        k1 = (px-90*ax)/cos(th1);
    else
        k1 = (py-90*ay)/sin(th1);
    end  
    k2 = (pz-90*az-60);
    a = k1./(sqrt(k1^2 + k2^2));
    fai_1 = acos(a);
    M= k1^2 + a2^2 + k2^2 - (a3^2 + d4^2);
    L = (2*a2*sqrt(k1^2 + k2^2)) ;% a2 = 150，L should not be 0
    th22 =  pi + asin(-M/L) - fai_1;
    th33 = -asin(-M/L)-fai_1;
    if th22 > pi/3
        th2 = th33;
    else
        th2 = th22;
    end
    
    if abs(th2) < tol
        th2_1 = 0;
    else
        th2_1 = th2;
    end
    
    %% for theta 3
    if abs(th1) < tol
        A = -a2*sin(th2_1) + (px-90*ax)/cos(th1);
    else
        A = -a2*sin(th2_1) + (py-d6*ay)/sin(th1);
    end
    B = -d1 - a2*cos(th2_1) + pz - d6*az;
    Y = (d4*B-a3*A)/(a3*a3+d4*d4); % a3 = 70, Y不会等于0
    th3 = acos(Y) - th2_1;
    if abs(th3) < tol
        th3 = 0;
    end
    
    %% for theta 4, theta 5, theta 6
    sin_th5 = -(az*cos(th2_1 + th3) + ax*cos(th1)*sin(th2_1 + th3) + ay*sin(th1)*sin(th2_1 + th3));
    th5 = asin(sin_th5);
    if abs(th5) < tol
        th5 = 0;
    end
    if abs(cos(th5)) < tol
        Ti36 = az*sin(th2_1 + th3) + ax*sin(th2_1)*cos(th1) + ay*sin(th1)*sin(th2_1) + ax*cos(th1)*cos(th2_1) + ay*sin(th1)*cos(th2_1);
        th4 = acos(-Ti36);
    else
        th44 = -(az*sin(th2_1 + th3) - ax*cos(th1)*cos(th2_1 + th3) - ay*sin(th1)*cos(th2_1 + th3))/cos(th5);
        th444 = acos(th44);
        th4 = real(th444);
    end
    if abs(th4) < tol
        th4 = 0;
    end
    
    %% for theta 6
    if abs(cos(th5)) < tol
        Ti21 = nz*cos(th2_1+th3) + nx*cos(th1)*sin(th2_1+th3) + ny*sin(th1)*cos(th2_1-th3);
        th66 = acos(Ti21);
        th6 = real(th66);
    else
        sin_th6 = -(oz*cos(th2_1 + th3) + ox*cos(th1)*sin(th2_1 + th3) + oy*sin(th1)*sin(th2_1+th3))/cos(th5);
        th66 = asin(sin_th6);
        th6 = real(th66);
    end
    if abs(th6) < tol
        th6 = 0;
    end

    %% The Results of q
    q = [th1, th2_1, th3, th4, th5, th6];
    
end




