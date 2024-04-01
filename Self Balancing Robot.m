clear all;
%---------state space-----------
A = [0 1 0 0;
    0 -126.3 -12.8 5.1;
    0 0 0 1;
    0 1446.9 213.6 -57.9];
B = [ 0;
    10.7;
    0;
    -122.6];
C = [0 0 1 0;
    0 0 0 1];

D = [0;
    0];
%-----------system------------
sys_ss = ss(A,B,C,D);
C1 = [0 0 1 0];
C2 = [0 0 0 1];
D1 = [0];
D2 = [0];
[num1,den1] = ss2tf(A,B,C1,D1);
F1 = tf(num1,den1);
figure(1);
rlocus(F1);
[num2,den2] = ss2tf(A,B,C2,D2);
F2 = tf(num2,den2);
figure(2);
rlocus(F2);
%---------eigen values---------
eigs(A)
%---------observability--------
phi_o1 = obsv(A,C1);
rank(phi_o1)

phi_o2 = obsv(A,C2);
rank(phi_o2)
%---------controlability-------
phi_c = ctrb(A,B);
rank(phi_c)
null(phi_c)

%---------state feedback far poles-------
%desired_poles = [-185 -100 -60 -90];
%K = acker(A, B, desired_poles);
%Acl = A - B*K;
%Bcl = B;
%Ccl = C;
%eig(Acl)
%---------state feedback close poles-------
desired_poles = [-5 -4 -3 -2];
K = acker(A, B, desired_poles);
Acl = A - B*K;
Bcl = B;
Ccl = C;
eig(Acl)
%------closed loop system------
[numcl1,dencl1] = ss2tf(Acl,B,C1,D1);
Fcl1 = tf(numcl1,dencl1);
figure(3);
rlocus(Fcl1);
[numcl2,dencl2] = ss2tf(Acl,B,C2,D2);
Fcl2 = tf(numcl2,dencl2);
figure(4);
rlocus(Fcl2);
%---------open-loop time responce--------
figure(5);
X0 = [0, 0, 0.1, 0.1];
t = 0:0.001:10;
u = 12 * ones(1,length(t));
lsim(sys_ss, u, t, X0)
%---------closed-loop time responce--------
figure(6);
SYScl = ss(Acl, Bcl, Ccl, D)
lsim(SYScl, u, t, X0)
%---------observer--------

%---------observable eigen values--------
U1 = phi_o1 + [0 0 0 0;
                0 0 0 0;
                0 0 0 0;
                1000 0 0 0];
U2 = phi_o2 + [0 0 0 0;
                0 0 0 0;
                0 0 0 0;
                1000 0 0 0];
            
Anew1 = U1*A*U1^-1;
Anew2 = U2*A*U2^-1;

Cnew1 = C1*U1^-1;
Cnew2 = C2*U2^-1;

A1obs = Anew1;
A2obs = Anew2;
B1obs = Cnew1;
B2obs = Cnew2;

A1obs(4,:)=[];
A2obs(4,:)=[];
A1obs(:,4)=[];
A2obs(:,4)=[];
B1obs(:,4)=[];
B2obs(:,4)=[];

A1obs = A1obs';
A2obs = A2obs';
B1obs = B1obs';
B2obs = B2obs';

%---------first observer--------
desired_poles = [-185.4640 -6.1504 -2];
L1 = acker(A1obs, B1obs, desired_poles);
L1(4) = 0

Aclobs1 = Anew1 - L1'*Cnew1;
eig(Aclobs1)

figure(7);
SYSclobs1 = ss(Aclobs1, U1*B, Cnew1, [0]);
lsim(SYSclobs1, u, t, X0)
figure(8);
SYSolobs1 = ss(Anew1, U1*B, Cnew1, [0]);
lsim(SYSolobs1, u, t, X0)
%---------second observer--------
eig(A2obs)
desired_poles = [-185.4640 -6.1504 -2];
L2 = acker(A2obs, B2obs, desired_poles);
L2(4) = 0

Aclobs2 = Anew2 - L2'*Cnew2;
eig(Aclobs2)

figure(9);
SYSolobs2 = ss(Anew2, U2*B, Cnew2, [0]);
lsim(SYSolobs2, u, t, X0)
figure(10);
SYSclobs2 = ss(Aclobs2, U2*B, Cnew2, [0]);
lsim(SYSclobs2, u, t, X0)
