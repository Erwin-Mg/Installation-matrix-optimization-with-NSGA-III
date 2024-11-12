% 确定机械臂参数
% 定义D-H参数
function [R_BA,Jacob,endlocation_1,MF]=endlocation_III(robotArm,q,W)
clc;
% N=10000; %随机采样数
robotArm1 = robotArm{1};
% robotArm2 = robotArm{2};
% robotArm3 = robotArm{3};
% robotArm4 = robotArm{4};

% 创建一个名为four_arm_robot的结构体
four_arm_robot.arm1 = robotArm1;


j=7;%j=关节数
% Q_1={N,j+3};
endlocation_1=zeros(W,j+3);
qs=zeros(1,7);
Q_1=cell(W,1);
Jacob=cell(W,1);
R_BA=cell(W,1);
for i=1:W
    for u=1:j
        qs(u)=q(u).qlim(1)+rand*(q(u).qlim(2)-q(u).qlim(1));
    end
    Q_1{i}=four_arm_robot.arm1.fkine(qs);
    R_BA{i} = [Q_1{i}.n,Q_1{i}.o,Q_1{i}.a];
    Jacob{i}= four_arm_robot.arm1.jacob0(qs);
    MF{i}=robotArm1.inertia(qs);
    endlocation_1(i,1:3)=Q_1{i}.t;
    endlocation_1(i,4:j+3)=qs;    
end
end