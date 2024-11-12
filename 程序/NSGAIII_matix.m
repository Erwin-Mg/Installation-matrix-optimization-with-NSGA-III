% clc,clear
global N M D  PopCon name gen encoding lower upper
tic;
N = 20;                          % 目标个数
name = 'DTLZ3';                 % 测试函数选择，目前只有：DTLZ1、DTLZ2、DTLZ3
gen = 5;                      %迭代次数
L = 2;                          %正方形搜索空间边长
W = 30000;                      %单臂采样次数
D = 12;      %决策变量个数
lower    = zeros(1,D)-L/2;    %决策变量下界
upper    = zeros(1,D)+L/2;     %决策变量上界
encoding = 'real';  %real-实数编码
%% Generate the reference points and random population
[robotArm_initial,q]=buildrobot_III(M);
[R_BA1,Jacob,endlocation_1,MF]=endlocation_III(robotArm_initial,q,W);
elapsed_time=toc; % 记录耗时
disp(['点云计算完成，耗时 ', num2str(elapsed_time), ' 秒']);
[Z,N] = UniformPoint(N,M);        % 生成一致性参考解
[Population,PF] = funfun(L); % 生成初始种群与设计变量
Pop_objs = CalObj(Population,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF); % 计算适应度函数值
Zmin  = min(Pop_objs(all(PopCon<=0,2),:),[],1); %求理想点，其实PopCon是处理有约束问题的，这里并没有用到
elapsed_time=toc; % 记录耗时
disp(['初始化完毕，耗时', num2str(elapsed_time), ' 秒']);
%% Optimization
tic;
for i = 20:gen
    MatingPool = TournamentSelection(2,N,sum(max(0,PopCon),2));
    Offspring  = GA(Population(MatingPool,:));
    Offspring_objs = CalObj(Offspring,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF);
    Zmin       = min([Zmin;Offspring_objs],[],1);
    Population = EnvironmentalSelection([Population;Offspring],N,Z,Zmin,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF);
    Popobj = CalObj(Population,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF);
    if(M<=3)
        plot3(Popobj(:,1),Popobj(:,2),Popobj(:,3),'ro')
        title(num2str(i));
        drawnow
    end
    elapsed_time=toc; % 记录耗时
    disp(['第 ', num2str(i), ' 代计算完毕，耗时 ',num2str(elapsed_time),' 秒' ]);
end
if(M<=3)
    hold on
    plot3(PF(:,1),PF(:,2),PF(:,3),'g*')
else
    for i=1:size(Popobj,1)
        plot(Popobj(i,:))
        hold on
    end
end
toc;
%%IGD
%获取最终优化结果chromosome
chromosome=Population;
score = IGD(Popobj,PF);
% 获取当前日期的字符串表示
current_date = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% 构建保存文件名
save_file_name = ['workspace_', current_date, '.mat'];
% 保存当前工作区的所有变量到文件
save(save_file_name);
% 显示保存的文件名
disp(['当前工作区已保存到文件: ', save_file_name]);