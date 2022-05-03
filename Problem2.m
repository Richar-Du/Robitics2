% 符号变量
syms theta1 theta2 theta3 theta4 theta5 theta6; %定义旋转角度变量
syms d1 d3 d4 d5 d6; 
syms a2 a3 a6; 
syms nx ny nz ox oy oz ax ay az px py pz;

a1=0;
a4=0;
a5=0;
d2=0;

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = -pi/2;
alpha5 = pi/2;


%位姿变换矩阵
A1 = trotz(theta1)*transl(0,0,d1)*transl(a1,0,0)*trotx(alpha1);
A2 = trotz(theta2)*transl(0,0,d2)*transl(a2,0,0)*trotx(alpha2);
A3 = trotz(theta3)*transl(0,0,d3)*transl(a3,0,0)*trotx(alpha3);
A4 = trotz(theta4)*transl(0,0,d4)*transl(a4,0,0)*trotx(alpha4);
A5 = trotz(theta5)*transl(0,0,d5)*transl(a5,0,0)*trotx(alpha5);
A6 = trotz(theta6);

%总变换矩阵
RTH=[nx,ox,ax,px;
	 ny,oy,ay,py;
	 nz,oz,az,pz;
	 0, 0, 0, 1];

	 
inv(A1)*RTH
%不同坐标系位姿变换矩阵
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5; 
T6 = A1*A2*A3*A4*A5*A6;


