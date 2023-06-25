syms s1 s2 s3

%Pre-defined matrices
T01 = [1 0 0 0; 0 1 0 0; 0 0 1 55; 0 0 0 1];
T12 = [cos(s1) 0 -sin(s1) 0; sin(s1) 0 cos(s1) 0; 0 -1 0 40; 0 0 0 1];
T23 = [cos(s2-90) -sin(s2-90) 0 100*cos(s2-90); sin(s2-90) cos(s2-90) 0 100*sin(s2-90); 0 0 1 0; 0 0 0 1];
T34 = [cos(s3+90) -sin(s3+90) 0 100*cos(s3+90); sin(s3+90) cos(s3+90) 0 100*sin(s3+90); 0 0 1 0; 0 0 0 1];


T02 = T01*T12
T03 = T02*T23
T04 = T03*T34

%s1 derivations

s1T04 = diff(T04,s1);
%s2 Derivations

s2T04 = diff(T04,s2);
%s3 Derivations

s3T04 = diff(T04,s3);