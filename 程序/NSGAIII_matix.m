% clc,clear
global N M D  PopCon name gen encoding lower upper
tic;
N = 20;                          % Ŀ�����
name = 'DTLZ3';                 % ���Ժ���ѡ��Ŀǰֻ�У�DTLZ1��DTLZ2��DTLZ3
gen = 5;                      %��������
L = 2;                          %�����������ռ�߳�
W = 30000;                      %���۲�������
D = 12;      %���߱�������
lower    = zeros(1,D)-L/2;    %���߱����½�
upper    = zeros(1,D)+L/2;     %���߱����Ͻ�
encoding = 'real';  %real-ʵ������
%% Generate the reference points and random population
[robotArm_initial,q]=buildrobot_III(M);
[R_BA1,Jacob,endlocation_1,MF]=endlocation_III(robotArm_initial,q,W);
elapsed_time=toc; % ��¼��ʱ
disp(['���Ƽ�����ɣ���ʱ ', num2str(elapsed_time), ' ��']);
[Z,N] = UniformPoint(N,M);        % ����һ���Բο���
[Population,PF] = funfun(L); % ���ɳ�ʼ��Ⱥ����Ʊ���
Pop_objs = CalObj(Population,robotArm_initial,R_BA1,Jacob,endlocation_1,W,MF); % ������Ӧ�Ⱥ���ֵ
Zmin  = min(Pop_objs(all(PopCon<=0,2),:),[],1); %������㣬��ʵPopCon�Ǵ�����Լ������ģ����ﲢû���õ�
elapsed_time=toc; % ��¼��ʱ
disp(['��ʼ����ϣ���ʱ', num2str(elapsed_time), ' ��']);
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
    elapsed_time=toc; % ��¼��ʱ
    disp(['�� ', num2str(i), ' ��������ϣ���ʱ ',num2str(elapsed_time),' ��' ]);
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
%��ȡ�����Ż����chromosome
chromosome=Population;
score = IGD(Popobj,PF);
% ��ȡ��ǰ���ڵ��ַ�����ʾ
current_date = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% ���������ļ���
save_file_name = ['workspace_', current_date, '.mat'];
% ���浱ǰ�����������б������ļ�
save(save_file_name);
% ��ʾ������ļ���
disp(['��ǰ�������ѱ��浽�ļ�: ', save_file_name]);