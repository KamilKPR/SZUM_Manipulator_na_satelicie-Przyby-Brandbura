%pobranie pozycji manipulatora z symulatora gazebo
pozycje=rossubscriber('/gazebo/link_states');
odbior_pozycji=receive(pozycje);

%pobranie pozycji k¹towej manipulatora z symulatora gazebo
katy=rossubscriber('/open_manipulator/joint_states');
odbior_katow=receive(katy);

%przypisanie odczytanych zmiennych konfiguracyjnych
theta1=odbior_katow.Position(3,1);
theta2=odbior_katow.Position(4,1);
theta3=odbior_katow.Position(5,1);
theta4=odbior_katow.Position(6,1);

%opis czlonu 1 robota wzglêdem jego pozycji pocz¹tkowej
p1=[0;0;0];
l1=[odbior_pozycji.Pose(3,1).Position.X;odbior_pozycji.Pose(3,1).Position.Y;odbior_pozycji.Pose(3,1).Position.Z];
r10=p1+l1;
omega1=[0;0;1];
v1=cross(-omega1,p1);
s1=[omega1;v1];
omegahat=[0 -s1(3) s1(2) s1(4); s1(3) 0 -s1(1) s1(5);-s1(2) s1(1) 0 s1(6);0 0 0 0];
r10a=[r10;1];


A1=expm(omegahat*theta1);
r1=expm(omegahat*theta1)*r10a;
r1kropka=diff(r1)




%opis czlonu 2 robota wzglêdem jego pozycji pocz¹tkowej
p2=r10;
l2=p2+[odbior_pozycji.Pose(4,1).Position.X;odbior_pozycji.Pose(4,1).Position.Y;odbior_pozycji.Pose(4,1).Position.Z];
r20=p2+l2;
omega2=[1;0;0];
v2=cross(-omega2,p2);
s2=[omega2;v2];
omegahat2=[0 -s2(3) s2(2) s2(4); s2(3) 0 -s2(1) s2(5);-s2(2) s2(1) 0 s2(6);0 0 0 0];
r20a=[r20;1];


r2=expm(omegahat*theta1)*expm(omegahat2*theta2)*r20a;
r2kropka=diff(r2)




%opis czlonu 3 robota wzglêdem jego pozycji pocz¹tkowej
p3=r20;
l3=p3+[odbior_pozycji.Pose(5,1).Position.X;odbior_pozycji.Pose(5,1).Position.Y;odbior_pozycji.Pose(5,1).Position.Z];
r30=p3+l3;
omega3=[1;0;0];
v3=cross(-omega3,p3);
s3=[omega3;v3];
omegahat3=[0 -s3(3) s3(2) s3(4); s3(3) 0 -s3(1) s3(5);-s3(2) s3(1) 0 s3(6);0 0 0 0];
r30a=[r30;1];
r3=expm(omegahat*theta1)*expm(omegahat2*theta2)*expm(omegahat3*theta3)*r30a;
r3kropka=diff(r3)

%opis czlonu 4 robota wzglêdem jego pozycji pocz¹tkowej
p4=r30;
l4=p4+[odbior_pozycji.Pose(6,1).Position.X;odbior_pozycji.Pose(6,1).Position.Y;odbior_pozycji.Pose(6,1).Position.Z];
r40=p4+l4;
omega4=[1;0;0];
v4=cross(-omega4,p4);
s4=[omega4;v4];
omegahat4=[0 -s4(3) s4(2) s4(4); s4(3) 0 -s4(1) s4(5);-s4(2) s4(1) 0 s4(6);0 0 0 0];
r40a=[r40;1];
r4=expm(omegahat*theta1)*expm(omegahat2*theta2)*expm(omegahat3*theta3)*expm(omegahat4*theta4)*r40a;
r4kropka=diff(r4)
r4kropkaa=[r4kropka;0];


A1=expm(omegahat*theta1);
A2=expm(omegahat2*theta2);
A3=expm(omegahat3*theta2);
A4=expm(omegahat4*theta3);

%%J1=simplify([omegahat*r2,A1*omegahat2*A2*r20a,[0;0;0;0],[0;0;0;0]])
%J=simplify([omegahat*A1*A2*A3*A4*r40a,A1*omegahat2*A2*A3*A4*r40a, A1*A2*omegahat3*A3*A4*r40a, A1*A2*A3*omegahat4*A4*r40a])

%wyznaczanie Jakobiana dla opisywanego manipulatora

J=([omegahat*A1*A2*A3*A4*r40a,A1*omegahat2*A2*A3*A4*r40a, A1*A2*omegahat3*A3*A4*r40a, A1*A2*A3*omegahat4*A4*r40a])
%J1=([omegahat*A1*A2*A3*A4*r40a,A1*omegahat2*A2*A3*A4*r40a, A1*A2*omegahat3*A3*A4*r40a, A1*A2*A3*omegahat4*A4*r40a]);

%obróbka macierzy potrzebna do dalszych obliczeñ
J3x3=[J(1,1),J(1,2),J(1,3),J(1,4);J(2,1),J(2,2),J(2,3),J(2,4);J(3,1),J(3,2),J(3,3),J(3,4)]

%wyznaczanie macierzy J^-1
odwrotnojakobianu=pinv(J3x3)
Qkropa3x3=pinv(J3x3)*r4kropka
Calkazqkropka=diff(J3x3)
%Jminus1=inv(J)
%Qkropa = inv(J)*r4kropkaa

%przypisanie nowych zmiennych konfiguracyjnych do napêdów robota
ruch1=rospublisher('/open_manipulator/joint1_position/command');
zadane=rosmessage(ruch1.MessageType);
zadane.Data= 1.5;
send(ruch1,zadane);

ruch4=rospublisher('/open_manipulator/joint4_position/command');
zadane4=rosmessage(ruch4.MessageType);
zadane4.Data= -1.0;
send(ruch4,zadane4);

ruch2=rospublisher('/open_manipulator/joint2_position/command');
zadane2=rosmessage(ruch2.MessageType);
zadane2.Data= 0.7;
send(ruch2,zadane2);

ruch3=rospublisher('/open_manipulator/joint3_position/command');
zadane3=rosmessage(ruch3.MessageType);
zadane3.Data= 0.0;
send(ruch3,zadane3);

%pozycje_chwytak=rossubscriber('/gazebo/link_states');
%odbior_pozycji_chwyataka=receive(pozycje_chwytak)

pause(3);

ruch2=rospublisher('/open_manipulator/joint2_position/command');
zadane2=rosmessage(ruch2.MessageType);
zadane2.Data= 0.0;
send(ruch2,zadane2);

ruch1=rospublisher('/open_manipulator/joint1_position/command');
zadane=rosmessage(ruch1.MessageType);
zadane.Data= 0.0;
send(ruch1,zadane);



ruch3=rospublisher('/open_manipulator/joint3_position/command');
zadane3=rosmessage(ruch3.MessageType);
zadane3.Data= 0.0;
send(ruch3,zadane3);

ruch4=rospublisher('/open_manipulator/joint4_position/command');
zadane4=rosmessage(ruch4.MessageType);
zadane4.Data= 0.0;
send(ruch4,zadane4);
