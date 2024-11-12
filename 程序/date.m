% load matlab.mat
% if M == 2
%     plot(chromosome(:,V + 1),chromosome(:,V + 2),'*');
%     xlabel('f_1'); ylabel('f_2');
%     title('Pareto Optimal Front');
% elseif M == 3
%     plot3(chromosome(:,V + 1),chromosome(:,V + 2),chromosome(:,V + 3),'*');
%     xlabel('f_1'); ylabel('f_2'); zlabel('f_3');
%     title('Pareto Optimal Surface');

% end
% toc;
chromosome=Population;
chromosome(127,:)=[0.5 0 0 0 0.5 0 -0.5 0 0 0 -0.5 0];%边长为1正方体基座
chromosome(128,:)=[5/2 8.66/2 0 0 0 0 -5/2 8.66/2 0 0 5.77/2 4.0824/2]/10;%边长为0.5正四面体基座
% 风筝型构型点位参考
chromosome(129,:)=[1.46 0 0 0.32 -0.675 0.23 0 0 0 0.32 0.675 0.23];
Popobj(:,M+1)=1:length(Popobj);
%% 

for i=75
    figure;
    scatter3(chromosome(i,1),chromosome(i,2),chromosome(i,3),'filled','DisplayName', '点A');
    hold on;
    scatter3(chromosome(i,4),chromosome(i,5),chromosome(i,6),'filled','DisplayName', '点B');
    scatter3(chromosome(i,7),chromosome(i,8),chromosome(i,9),'filled','DisplayName', '点C');
    scatter3(chromosome(i,10),chromosome(i,11),chromosome(i,12),'filled','DisplayName', '点D');
    legend;

%     drawSphere([chromosome(i,1),chromosome(i,2),chromosome(i,3)], 1);
%     drawSphere([chromosome(i,4),chromosome(i,5),chromosome(i,6)], 1);
%     drawSphere([chromosome(i,7),chromosome(i,8),chromosome(i,9)], 1);
%     drawSphere([chromosome(i,10),chromosome(i,11),chromosome(i,12)], 1);
    % 确定四臂根部位置
    vertex_A = chromosome(i,1:3);
    vertex_B = chromosome(i,4:6);
    vertex_C = chromosome(i,7:9);
    vertex_D = chromosome(i,10:12);
    
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
    % 创建偏移矩阵
    offset_B = [x(2)-x(1); y(2)-y(1); z(2)-z(1)];
    offset_C = [x(3)-x(1); y(3)-y(1); z(3)-z(1)];
    offset_D = [x(4)-x(1); y(4)-y(1); z(4)-z(1)];
    positions = endlocation_1(:, 1:3)+vertex_A;
    Q_2(:, 1:3) = (trans_matixBA * (positions' + offset_B - vertex_B') + vertex_B')';
    Q_3(:, 1:3) = (trans_matixCA * (positions' + offset_C - vertex_C') + vertex_C')';
    Q_4(:, 1:3) = (trans_matixDA * (positions' + offset_D - vertex_D') + vertex_D')';

    alpha_rete1 = alphaShape(positions);
    alpha_rete2 = alphaShape(Q_2(:,1:3));
    alpha_rete3 = alphaShape(Q_3(:,1:3));
    alpha_rete4 = alphaShape(Q_4(:,1:3));
    volume_all=volume(alpha_rete1);
    plot(alpha_rete1, 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
%     hold on;
    plot(alpha_rete2, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
    plot(alpha_rete3, 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
    plot(alpha_rete4, 'FaceColor', 'g', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.1);
    k=1;
end
mean1=mean(Popobj);
%计算前i个个体的各指标
%% 校验指标

tic;
% alphashape1=alphaShape(endlocation_1(:, 1:3));
% volume(alphashape1);
for i=[127 128]
     [indicators(i,1:5),INDEX(i,:),robotArm1,robotArm2,robotArm3,robotArm4]=date_pro_2(chromosome(i,1:12),robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF);
end
toc;
[indicators(127,1:5),INDEX(127,:),robotArm1,robotArm2,robotArm3,robotArm4]=cubeobj(chromosome(127,1:12),robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF);

%% 
% 计算向量 CD
vector_CD = points(4, :) - points(3, :);

% 计算 xoy 平面的法向量
normal_vector_xoy = [0, 0, 1]; % xoy 平面的法向量

% 计算向量 CD 和 xoy 平面的夹角（弧度）
cosine_angle = dot(vector_CD, normal_vector_xoy) / (norm(vector_CD) * norm(normal_vector_xoy));
angle_in_radians = acos(cosine_angle);

% 将夹角转换为度数
angle_in_degrees = rad2deg(angle_in_radians);

disp(['向量 CD 和 xoy 平面的夹角为：', num2str(angle_in_degrees), ' 度']);
%% 

% 计算向量 AB 和 AD
vector_AB = points(2, :) - points(1, :);
vector_AD = points(4, :) - points(1, :);

% 计算 yoz 面的法向量
normal_vector_yoz = [1, 0, 0]; % yoz 面的法向量

% 计算向量 AB 和 AD 构成的平面与 yoz 面的夹角（弧度）
cross_product = cross(vector_AB, vector_AD);
cosine_angle = dot(cross_product, normal_vector_yoz) / (norm(cross_product) * norm(normal_vector_yoz));
angle_in_radians = acos(cosine_angle);

% 将夹角转换为度数
angle_in_degrees = rad2deg(angle_in_radians);

disp(['面 ABD 和 yoz 面的夹角为：', num2str(angle_in_degrees), ' 度']);

%% 



% 计算向量 AB、AD 和 AE
vector_AB = points(2, :) - points(1, :);
vector_AD = points(4, :) - points(1, :);
vector_AE = [vector_AB(1), vector_AB(2), 0];

% 计算面 ABD 和面 ABE 的法向量
normal_ABD = cross(vector_AB, vector_AD);
normal_ABE = cross(vector_AB, vector_AE);

% 计算两个法向量的夹角（弧度）
cosine_angle = dot(normal_ABD, normal_ABE) / (norm(normal_ABD) * norm(normal_ABE));
angle_in_radians = acos(cosine_angle);

% 将夹角转换为度数
angle_in_degrees = rad2deg(angle_in_radians);

disp(['面 ABD 和面 ABE 的二面角为：', num2str(angle_in_degrees), ' 度']);

% 计算向量 AE 和 BE
vector_AE = points(1, :) - points(3, :);
vector_BE = points(2, :) - points(3, :);

% 计算向量长度
length_AE = norm(vector_AE);
length_BE = norm(vector_BE);

disp(['AE 的长度为：', num2str(length_AE)]);
disp(['BE 的长度为：', num2str(length_BE)]);
%% 

indicator=zeros(3,5);
indicator(:,1:2)=indicators(:,1:2);
indicator(:,3:5)=indicators(:,3:5)/1000;
indicator(:,3)=-indicator(:,3);
data1 = indicator(1,:);
data2 = indicator(2,:);
data3 = indicator(3,:);

% 定义角度
theta = linspace(0, 2*pi, 6); % 添加一个额外的点，确保第一个点和最后一个点重合

% 绘制雷达图
polarplot(theta, [data1, data1(1)], '-o', 'LineWidth', 2); % 绘制第一组数据
hold on;
polarplot(theta, [data2, data2(1)], '-s', 'LineWidth', 2); % 绘制第二组数据
polarplot(theta, [data3, data3(1)], '-d', 'LineWidth', 2); % 绘制第三组数据

% 添加图例
legend('Data 1', 'Data 2', 'Data 3', 'Location', 'best');

% 添加标题
title('三组数据的雷达图');
%% 
% 原始数据
data1 = [34.4728130067696	2.01758603063310	11905.6046914841	13994.9185177671	6776.23646950181];
data2 = [30.1843334762613	1.83011408388014	11209.8153657975	8538.89586531398	5408.12192405637];
data3 = [30.6675413532669	0.959664686742005	11979.6644447413	8889.23292153020	5445.23546851259];

% 计算百分比
% 最优构型对比
percentage1 = data1 ./ data3;
% 正四面体对比
percentage2 = data2 ./ data3;

% 定义角度
theta = linspace(0, 2*pi, 6); % 添加一个额外的点，确保第一个点和最后一个点重合

% 绘制雷达图
polarplot(theta, [ones(1,5), 1], '-k'); % 绘制第三组数据，值均为1
hold on;
polarplot(theta, [percentage1, percentage1(1)], '-o', 'LineWidth', 2); % 绘制第一组数据
polarplot(theta, [percentage2, percentage2(1)], '-s', 'LineWidth', 2); % 绘制第二组数据

% 添加图例
legend({'Data 3 (Baseline)', '', '', '', '', ...
       'Kinematic Manipulability', 'Synergy Volume', 'Stiffness', 'Dynamic Manipulability', 'Base Accuracy', ...
       '', '', '', '', ''}, 'Location', 'best');

% 添加标题
title('前两组数据相对于第三组数据的百分比雷达图');

% 移除雷达图边缘的角度值
thetaticks([]); % 移除角度刻度



