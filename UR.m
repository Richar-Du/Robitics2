clear;
clc;
%��ת�Ƕ�
syms theta1 theta2 theta3 theta4 theta5 theta6; 


%DH�������е���������
d1 = 86.85;        a1 = 0;     alpha1 = 90;
d2 = 0;            a2 = 243.65;alpha2 = 0;
d3 = -92.85;      a3 = 213  ; alpha3 = 0;
d4 = 83.4  ;       a4 = 0;     alpha4 = -90;
d5 = 83.4  ;          a5 = 0;  alpha5 = 90;

%�������任
A1 = trotz(deg2rad(theta1))*transl(0,0,d1)*transl(a1,0,0)*trotx(deg2rad(alpha1));
A2 = trotz(deg2rad(theta2))*transl(0,0,d2)*transl(a2,0,0)*trotx(deg2rad(alpha2));
A3 = trotz(deg2rad(theta3))*transl(0,0,d3)*transl(a3,0,0)*trotx(deg2rad(alpha3));
A4 = trotz(deg2rad(theta4))*transl(0,0,d4)*transl(a4,0,0)*trotx(deg2rad(alpha4));
A5 = trotz(deg2rad(theta5))*transl(0,0,d5)*transl(a5,0,0)*trotx(deg2rad(alpha5));
A6 = trotz(deg2rad(theta6))*transl(0,0,40);

%λ�˱任����
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5; 
T6 = A1*A2*A3*A4*A5*A6;

%���ɻ�е��ģ��
L(1) = Link([0,d1,a1,deg2rad(alpha1)]);
L(2) = Link([0,d2,a2,deg2rad(alpha2)]);
L(3) = Link([0,d3,a3,deg2rad(alpha3)]);
L(4) = Link([0,d4,a4,deg2rad(alpha4)]);
L(5) = Link([0,d5,a5,deg2rad(alpha5)]);
L(6) = Link([0,160,0,0]);
%�޶������˻��Χ
L(1).qlim = [-2*pi,2*pi];
L(2).qlim = [-2*pi,2*pi];
L(3).qlim = [-2*pi,2*pi];
L(4).qlim = [-2*pi,2*pi];
L(5).qlim = [-2*pi,2*pi];
L(6).qlim = [-2*pi,2*pi];

%��е�ۺϳ�
UR3 = SerialLink(L,'name','UR3');
%figure(1);
view(3);
q0 = deg2rad([0,0,0,0,0,0]);
UR3.plot(q0,'workspace',[-600 800 -600 800 -600 600],'tilesize',100);
UR3.teach

%��ʾ����
hold on;
[x, y, z] = ellipsoid(0,-1020,0,500,500,500,70);
surfl(x, y, z)
colormap white
axis equal
center=[0 -1020 0];         % Բ������
radius=500;          % �뾶

%�����ǹ켣�滮����

% ��ɽ���м����
N=(0:0.01:3/20)';           % ��Բ�ȷ�
theta=N*pi;      % ���ֽǶ�				
phi=pi/18;			
points1=(center+radius*[cos(theta)*sin(phi) cos(theta)*cos(phi) sin(theta)])';
points1=fliplr(points1);			% ��������˳��ת��ʹ���ֱ۰��ձʻ�˳��
hold on;
plot3(points1(1,:),points1(2,:),points1(3,:),'r');
T1 = transl(points1');			%�����м�ÿһ����λ�˾���
q1 = UR3.ikine(T1,'mask',[1 1 1 0 0 0]);			%���˶�ѧ��
hold on;
UR3.plot(q1,'tilesize',200);


%����Ĳ��ֻ���һ����ֻ�Ǽ���Ƕȵĵط���һ��
% ��ɽ����߶���
N=(0:0.01:3/32)';           
theta=N*pi;
phi=(pi/18+pi/18);
points2=(center+radius*[cos(theta)*sin(phi) cos(theta)*cos(phi) sin(theta)])';
points2=fliplr(points2);
hold on;
plot3(points2(1,:),points2(2,:),points2(3,:),'r');
T2 = transl(points2');
q2 = UR3.ikine(T2,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q2,'tilesize',200);

% ��ɽ������ĺ�
N=(1/18-1/18:0.01:1/18+1/18)';           
phi=N*pi;
theta=0;
points3=(center+radius*[sin(phi) cos(phi) zeros(size(phi))])';
points3=fliplr(points3);		% ��������˳��ת��ʹ���ֱ۰��ձʻ�˳��
hold on;
plot3(points3(1,:),points3(2,:),points3(3,:),'r');
T3 = transl(points3');
q3 = UR3.ikine(T3,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q3,'tilesize',200);

% ��ɽ���ұ߶���
N=(0:0.01:3/32)';           
theta=N*pi;
phi=(pi/18-pi/18);
points4=(center+radius*[cos(theta)*sin(phi) cos(theta)*cos(phi) sin(theta)])';
points4=fliplr(points4);
hold on;
plot3(points4(1,:),points4(2,:),points4(3,:),'r');
T4 = transl(points4');
q4 = UR3.ikine(T4,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q4,'tilesize',200);

% ���󡱵ĺ�
N=(-3/20:0.01:-1/20)';          
phi=N*pi;
theta=3/32*pi*ones(size(phi));
points5=(center+radius*[cos(theta).*sin(phi) cos(theta).*cos(phi) sin(theta)])';
points5=fliplr(points5);
hold on;
plot3(points5(1,:),points5(2,:),points5(3,:),'r');
T5 = transl(points5');
q5 = UR3.ikine(T5,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q5,'tilesize',200);

% ���󡱵�Ʋ��һ����
N=(3/32:0.005:3/20)';           
theta=N*pi;      
phi=-pi/10;
points6=(center+radius*[cos(theta).*sin(phi) cos(theta).*cos(phi) sin(theta)])';
points6=fliplr(points6);
hold on;
plot3(points6(1,:),points6(2,:),points6(3,:),'r');
T6 = transl(points6');
q6 = UR3.ikine(T6,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q6,'tilesize',200);

% ���󡰵�Ʋ�ĵڶ�����
N=linspace(3/32,0,20)';		%�������½Ƕ�
theta=N*pi;			%�°벿�ֵ������ϰ벿�ֵ��յ㱣��һ��
theta=fliplr(theta);
M=linspace(-2/20,-1/20,20)';		%�������ҽǶ�
phi=M*pi;
points7=(center+radius*[cos(theta).*sin(phi) cos(theta).*cos(phi) sin(theta)])';
hold on;
plot3(points7(1,:),points7(2,:),points7(3,:),'r');
T7 = transl(points7');
q7 = UR3.ikine(T7,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q7,'tilesize',200);

% "��"����
N=linspace(3/32,0,20)';		%�������½Ƕ�
theta=N*pi;			%�°벿�ֵ������ϰ벿�ֵ��յ㱣��һ��
theta=fliplr(theta);
M=linspace(-2/20,-3/20,20)';		%�������ҽǶ�
phi=M*pi;
points7=(center+radius*[cos(theta).*sin(phi) cos(theta).*cos(phi) sin(theta)])';
hold on;
plot3(points7(1,:),points7(2,:),points7(3,:),'r');
T7 = transl(points7');
q7 = UR3.ikine(T7,'mask',[1 1 1 0 0 0]);
hold on;
UR3.plot(q7,'tilesize',200);