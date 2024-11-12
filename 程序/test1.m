% clear;
% %          theta  d/Z      a/X          alpha
% T(1)=Link([0      0.09744   0          0    0],'modified');
% T(2)=Link([0      0         0          pi/2   0],'modified');
% T(2).offset=pi;
% T(3)=Link([0      0.39911    0     pi/2    0],'modified');
% T(4)=Link([0      0   0           pi/2    0],'modified');
% T(4).offset=pi/2;
% T(5)=Link([0      0         0.44284     pi/2    0],'modified');
% T(5).offset=pi/2;
% T(6)=Link([0        0.1344    0         pi/2   0 ],'modified');
% T(6).offset=pi/2;
% T(7)=Link([0      0   0           pi/2    0],'modified');
% T(7).offset=pi/2;
% % T(8)=Link([0      0   0.2           0   0],'modified');
% robotArm1 = SerialLink(T, 'name', 'Arm1');
% % robotArm1.tool.t=[0.05 0 0]';
% % robotArm1.teach;
% % robotArm1.maniplty
% q(1).qlim=[-180,180]/180*pi;
% q(2).qlim=[-180,180]/180*pi;
% q(3).qlim=[-180,180]/180*pi;
% q(4).qlim=[-180,180]/180*pi;
% q(5).qlim=[-180,180]/180*pi;
% q(6).qlim=[-180,180]/180*pi;
% q(7).qlim=[-180,180]/180*pi;
% %% 
% 
% 
% % 绘制连线
% 
% 
% W=10000;
% j=7;
% Q_1=cell(W,1);
% for i=1:W
%     for u=1:j
%         qs(u)=q(u).qlim(1)+rand*(q(u).qlim(2)-q(u).qlim(1));
%     end
%     Q_1{i}=robotArm1.fkine(qs);
% end
% 
% %% 
% figure;
% 
% for i=1:W
%     plot3(Q_1{i}.t(1),Q_1{i}.t(2),Q_1{i}.t(3),'r.');
%     hold on;
% end
% %% 
% 
% % 定义旋转角度
% angle_deg = -45;
% % 计算旋转矩阵
% R = rotx(angle_deg);
% 
% % 定义原始点
% point_B = [0.1 0.1 0.5];
% point_D = [0.1 0.2 0.5];
% 
% % 将点乘以旋转矩阵
% rotated_B = point_B * R;
% rotated_D = point_D * R;
% 
% % 提取点的坐标
% x_values = [rotated_B(1), rotated_D(1)];
% y_values = [rotated_B(2), rotated_D(2)];
% z_values = [rotated_B(3), rotated_D(3)];
% 
% plot3(x_values, y_values, z_values, 'b.-', 'LineWidth', 2);
% hold on;
% scatter3(x_values, y_values, z_values, 100, 'b', 'filled');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('两个点的连线');
% axis equal;
% grid on;


%% 
% clear Q_2 Q_3 Q_4
j=7;
t=8;
l=1;
vertex_A = Population(t,1:3);
vertex_B = Population(t,4:6);
vertex_C = Population(t,7:9);
vertex_D = Population(t,10:12);
x = [ vertex_A(1),vertex_B(1),vertex_C(1),vertex_D(1)];
y = [ vertex_A(2),vertex_B(2),vertex_C(2),vertex_D(2)];
z = [ vertex_A(3),vertex_B(3),vertex_C(3),vertex_D(3)];

% 计算坐标转换矩阵
line_BA = vertex_B - vertex_A;
line_CA = vertex_C - vertex_A;
line_DA = vertex_D - vertex_A;
% 单位化连线向量
vector_BA = line_BA / norm(line_BA);
vector_CA = line_CA / norm(line_CA);
vector_DA = line_DA / norm(line_DA);
% 创建偏移矩阵
offset_B = [x(2)-x(1); y(2)-y(1); z(2)-z(1)];
offset_C = [x(3)-x(1); y(3)-y(1); z(3)-z(1)];
offset_D = [x(4)-x(1); y(4)-y(1); z(4)-z(1)];

n_BA=cross(vector_BA,[0,0,1])';
o_BA=cross(n_BA,vector_BA)';
R_BA=tr2eul([n_BA,o_BA,vector_BA']);
trans_matixBA = rotz(R_BA(1)*180/pi)*roty(R_BA(2)*180/pi)*rotz(R_BA(3)*180/pi);

n_CA=cross(vector_CA,[0,0,1])';
o_CA=cross(n_CA,vector_CA)';
R_CA=tr2eul([n_CA,o_CA,vector_CA']);
trans_matixCA = rotz(R_CA(1)*180/pi)*roty(R_CA(2)*180/pi)*rotz(R_CA(3)*180/pi);

n_DA=cross(vector_DA,[0,0,1])';
o_DA=cross(n_DA,vector_DA)';
R_DA=tr2eul([n_DA,o_DA,vector_DA']);
trans_matixDA = rotz(R_DA(1)*180/pi)*roty(R_DA(2)*180/pi)*rotz(R_DA(3)*180/pi);

% 将 endlocation_1 的位置信息和关节角信息分开
endlocation_1(:, 1:3)=endlocation_1(:, 1:3)+vertex_A;
positions = endlocation_1(:, 1:3);
joint_angles = endlocation_1(:, 4:end);

Q_2(:, 1:3) = (trans_matixBA * (positions' + offset_B - vertex_B') + vertex_B')';
Q_3(:, 1:3) = (trans_matixCA * (positions' + offset_C - vertex_C') + vertex_C')';
Q_4(:, 1:3) = (trans_matixDA * (positions' + offset_D - vertex_D') + vertex_D')';

% 复制关节角信息
Q_2(:, 4:j+3) = joint_angles;
Q_3(:, 4:j+3) = joint_angles;
Q_4(:, 4:j+3) = joint_angles;



%计算各个根部之间的方向向量
vector_12=vertex_A-vertex_B;
vector_13=vertex_A-vertex_C;
vector_14=vertex_A-vertex_D;
vector_23=vertex_B-vertex_C;
vector_24=vertex_B-vertex_D;
vector_34=vertex_C-vertex_D;

pos_switchBA=l*(vector_12 / norm(vector_12));
pos_switchCA=l*(vector_13 / norm(vector_13));
pos_switchDA=l*(vector_14 / norm(vector_14));
pos_switchCB=l*(vector_23 / norm(vector_23));
pos_switchDB=l*(vector_24 / norm(vector_24));
pos_switchDC=l*(vector_34 / norm(vector_34));




%% 

points=[vertex_A;vertex_B;vertex_C;vertex_D];
alpha_shape=alphaShape(points);
% 粗略筛除与基座干涉末端点，未考虑自避障
in_alpha_shape = inShape(alpha_shape, endlocation_1(:,1), endlocation_1(:,2), endlocation_1(:,3));
endlocation_1=endlocation_1(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_2(:,1), Q_2(:,2), Q_2(:,3));
Q_2=Q_2(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_3(:,1), Q_3(:,2), Q_3(:,3));
Q_3=Q_3(~in_alpha_shape, :);
clear in_alpha_shape
in_alpha_shape = inShape(alpha_shape, Q_4(:,1), Q_4(:,2), Q_4(:,3));
Q_4=Q_4(~in_alpha_shape, :);
clear in_alpha_shape
% scatter3(Q_2(:,1), Q_2(:,2), Q_2(:,3), 10, 'b', 'filled');
% 计算夹持物体后物体末端相对位置
pos_switchBA_mat = repmat(pos_switchBA, length(Q_2), 1);
pos_switchCA_mat = repmat(pos_switchCA, length(Q_3), 1);
pos_switchDA_mat = repmat(pos_switchDA, length(Q_4), 1);
pos_switchCB_mat = repmat(pos_switchCB, length(Q_3), 1);
pos_switchDB_mat = repmat(pos_switchDB, length(Q_4), 1);
pos_switchDC_mat = repmat(pos_switchDC, length(Q_4), 1);

% 平移末端点
Q_21trans = Q_2(:, 1:3) + pos_switchBA_mat;
Q_31trans = Q_3(:, 1:3) + pos_switchCA_mat;
Q_41trans = Q_4(:, 1:3) + pos_switchDA_mat;
Q_32trans = Q_3(:, 1:3) + pos_switchCB_mat;
Q_42trans = Q_4(:, 1:3) + pos_switchDB_mat;
Q_43trans = Q_4(:, 1:3) + pos_switchDC_mat;


% 筛除夹持物块后与基座干涉点
in_alpha_shape = inShape(alpha_shape, Q_21trans(:,1), Q_21trans(:,2), Q_21trans(:,3));
Q_21trans= Q_21trans(~in_alpha_shape, :);
clear in_alpha_shape
Q_31trans=shaixuan(Q_31trans,alpha_shape);
Q_41trans=shaixuan(Q_41trans,alpha_shape);
Q_32trans=shaixuan(Q_32trans,alpha_shape);
Q_42trans=shaixuan(Q_42trans,alpha_shape);
Q_43trans=shaixuan(Q_43trans,alpha_shape);


alphaQ1=alphaShape(endlocation_1(:,1:3));
alphaQ2=alphaShape(Q_2(:,1:3));
alphaQ3=alphaShape(Q_3(:,1:3));
alphaQ4=alphaShape(Q_4(:,1:3));
% 绘制四面体的 alpha shape
% figure;
% plot(alpha_shape);
% hold on;
% plot(alphaQ1);
% plot(alphaQ2);
% plot(alphaQ3);
% plot(alphaQ4);
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('四面体的 Alpha Shape');


function A1 = shaixuan(A,alpha_shape)
        in_alpha_shape = inShape(alpha_shape,A(:,1), A(:,2), A(:,3));
        A1=A(~in_alpha_shape, :);
end
