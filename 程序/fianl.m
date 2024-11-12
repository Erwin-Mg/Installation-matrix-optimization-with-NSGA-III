% 假设有四个空间点
% finalpoints=[chromosome1(1:3);chromosome1(4:6);chromosome1(7:9);chromosome1(10:12)];
points = finalpoints*roty(30); % 4×3矩阵
%points = [1.46 0 0; 0.32 -0.675 0.25; 0 0 0; 0.32 0.675 0.25];
% 画出四面体
figure;
scatter3(points(:, 1), points(:, 2), points(:, 3), 100, 'o', 'filled');
hold on;

% 连接四个点构成四面体，并标注每个点
for i = 1:4
    text(points(i, 1), points(i, 2), points(i, 3), sprintf('  %c', 'A' + i - 1), 'FontSize', 12);
    for j = i+1:4
        mid_point = (points(i, :) + points(j, :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf('  %.2f', norm(points(i, :) - points(j, :))), 'FontSize', 10);
        plot3([points(i, 1), points(j, 1)], [points(i, 2), points(j, 2)], [points(i, 3), points(j, 3)], 'k-', 'LineWidth', 2);
    end
end

xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
grid on;

% 计算每条边的长度
edge_lengths = zeros(6, 1);
edge_index = 1;
for i = 1:4
    for j = i+1:4
        edge_lengths(edge_index) = norm(points(i, :) - points(j, :));
        edge_index = edge_index + 1;
    end
end

% 显示每条边的长度
disp('每条边的长度：');
disp(edge_lengths);

% 计算向量 BC、BD、AC 和 AD
vector_BC = points(3, :) - points(2, :);
vector_BD = points(4, :) - points(2, :);
vector_AC = points(3, :) - points(1, :);
vector_AD = points(4, :) - points(1, :);

% 计算平面 BCD 和平面 ACD 的法向量
normal_vector_BCD = cross(vector_BC, vector_BD);
normal_vector_ACD = cross(vector_AC, vector_AD);
% normal_vector_ACD = [0 0 1];
% 计算两法向量之间的夹角（弧度）
cosine_angle = dot(normal_vector_BCD, normal_vector_ACD) / (norm(normal_vector_BCD) * norm(normal_vector_ACD));
angle_in_radians = acos(cosine_angle);

% 将夹角转换为度数
angle_in_degrees = rad2deg(angle_in_radians);

disp(['平面 BCD 和平面 ACD 的二面角为：', num2str(angle_in_degrees), ' 度']);
%% 计算点A基座的位置向量
points = finalpoints; % 4×3矩阵
points(4,:)=points(1,:)+[0 0 1];
% 画出四面体
figure;
scatter3(points(:, 1), points(:, 2), points(:, 3), 100, 'o', 'filled');
hold on;

% 连接四个点构成四面体，并标注每个点
for i = 1:4
    text(points(i, 1), points(i, 2), points(i, 3), sprintf('  %c', 'A' + i - 1), 'FontSize', 12);
    for j = i+1:4
        mid_point = (points(i, :) + points(j, :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf('  %.2f', norm(points(i, :) - points(j, :))), 'FontSize', 10);
        plot3([points(i, 1), points(j, 1)], [points(i, 2), points(j, 2)], [points(i, 3), points(j, 3)], 'k-', 'LineWidth', 2);
    end
end

xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
grid on;

% 计算每条边的长度
edge_lengths = zeros(6, 1);
edge_index = 1;
for i = 1:4
    for j = i+1:4
        edge_lengths(edge_index) = norm(points(i, :) - points(j, :));
        edge_index = edge_index + 1;
    end
end

% 显示每条边的长度
disp('每条边的长度：');
disp(edge_lengths);

% 计算向量 BC、BD、AC 和 AD
vector_BC = points(3, :) - points(2, :);
vector_BD = points(4, :) - points(2, :);
vector_AC = points(3, :) - points(1, :);
vector_AD = points(4, :) - points(1, :);
vector_AB = points(2, :) - points(1, :);

% 计算平面 BCD 和平面 ACD 的法向量
normal_vector_ABC = cross(vector_AB, vector_AC);
normal_vector_ACD = cross(vector_AC, vector_AD);
% normal_vector_ACD = [0 0 1];
% 计算两法向量之间的夹角（弧度）
cosine_angle = dot(normal_vector_ABC, normal_vector_ACD) / (norm(normal_vector_ABC) * norm(normal_vector_ACD));
angle_in_radians = acos(cosine_angle);

% 将夹角转换为度数
angle_in_degrees = rad2deg(angle_in_radians);

disp(['平面 ABC 和平面 ACD 的二面角为：', num2str(angle_in_degrees), ' 度']);