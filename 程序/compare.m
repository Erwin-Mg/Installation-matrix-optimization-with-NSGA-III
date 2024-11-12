M=5;
W=300000;
tic;
[robotArm_initial,q]=buildrobot_III(M);
[R_BA1,Jacob,endlocation_1,MF]=endlocation_III(robotArm_initial,q,W);
toc;
a=2; %搜索空间边长、正方形构型边长
% load 20240222.mat
point_coordinate=[0.5 0 0 0 0.5 0 -0.5 0 0 0 -0.5 0]; %边长为2正方体基座
cube=CalObj(point_coordinate,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF); % 计算适应度函数值
i=1;
for b=0.1:0.1:sqrt(2)*a
tic;
% 正四面体底面三角形顶点坐标
A = [0, b/2, 0];
B = [0, -b/2, 0];
C = [b/2, 0, -b/sqrt(2)];
D = [-b/2, 0,-b/sqrt(2)];
point_rete=[A B C D];
% point_cube=[b/2 0 0 0 b/2 0 -b/2 0 0 0 -b/2 0];
rete(i,:)=CalObj(point_rete,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF); % 计算适应度函数值
%cube(i,:)=CalObj(point_cube,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF); % 计算适应度函数值
i=i+1;
elapsed_time=toc; % 记录耗时
disp(['第 ', num2str(i), ' 次计算完毕，耗时 ',num2str(elapsed_time),' 秒' ]);
end