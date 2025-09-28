function T = matrixT(a, alph, d, q)
    T = [
        cos(q)              -sin(q)             0           a;
        sin(q)*cos(alph)    cos(q)*cos(alph)    -sin(alph)  -d*sin(alph);
        sin(q)*sin(alph)    cos(q)*sin(alph)    cos(alph)   d*cos(alph);
        0                   0                   0           1;
    ];