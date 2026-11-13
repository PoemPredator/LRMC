%%
% 本代码是根据《动力过程姿态控制研究报告》所写
% 2025.06.13
%%
% clear workspace / all figure windows / command window
clear;
% close all;
clc;
%%
tic;
ON = 1;
OFF = 0;
FITTING = 0;
INTERPOLATION = 1;
LEN = 0;
PRO = 1;
COUNT = 0;
dest_path = './pic_res_20250612/';  % 图片保存地址的目录，当无对应文件夹时，需要创建
%%
for iter_time = 1:30 % 组合拉偏的次数 %1:30 %1:1000
for rand_m = 3 % 1:3:7  % [1612,  1712,  1765,  1912,  2100,  1874] % D1种(6,6)  D2种(5,6)  D3种(3,1)
for rand_k = 1 % 1:3:7  % [4918,  4718,  4518,  4318,  4000,  3016,  2453]
%%         index  =    [  1,    2,    3,    4,    5,    6,    7]

% 与拉偏量相关的开关
M_TK_ROBUST_KEY = OFF; % mT_k 拉偏是否打开
L_K_ROBUST_KEY = OFF;  % L_k 拉偏是否打开
PENDULUM_ROBUST_KEY = PRO; % PRO; % LEN; % l_k --摆长 拉偏以何种方式（比例还是加和）是否打开
PENDULUM_ROBUST_KEY2 = OFF;
M_K_ROBUST_KEY = OFF;  % m_k 拉偏是否打开

% 根据拉偏验证时 参数遍历下的各个结果，选出较差的参数，设成固定参数，修改控制增益，以再次验证该参数下的性能
if M_TK_ROBUST_KEY == ON % m_k 拉偏是否打开 
    
    m_T12_total = 100 * (-1+2*rand(1));  % [-100,100]  
    m_T1_add_fac = m_T12_total/(1+1458/874.437);
    m_T2_add_fac = (1458/874.437) * m_T1_add_fac;
   
    m_T34_total = 200 * (-1+2*rand(1));   % [-200,200]
    m_T3_add_fac = (2/3) * m_T34_total;
    m_T4_add_fac = (1/3) * m_T34_total;
else
    m_T1_add_fac = 0; % m_1 ... m_4 设置的是极限值
    m_T2_add_fac = 0;
    m_T3_add_fac = 0;
    m_T4_add_fac = 0;
end

m_T1_4_add_fac_group = [m_T1_add_fac, m_T2_add_fac, m_T3_add_fac, m_T4_add_fac];

if L_K_ROBUST_KEY == ON   % L_k 拉偏是否打开
    
    L12_1 = 0.3 * (-1+2*rand(1));  % -0.3 ~ 0.3
    L12_2 = -0.3 +(0.2-(-0.3))*rand(1);  % -0.3 ~ 0.2
    L34_3 = 0.5 * (-1+2*rand(1));  % -0.5 ~ 0.5
    L34_4 = 0.5 * (-1+2*rand(1));  % -0.5 ~ 0.5
   
else
    L12_1 = 0; % 0.3
    L12_2 = 0; % 0.05---2.5°% 0.1---5° % 0.03
    L34_3 = 0; % 0.5
    L34_4 = 0; % -0.5
end    
L12_rand_val = [L12_1 , L12_2];
L34_rand_val = [L34_3 , L34_4];

if PENDULUM_ROBUST_KEY == LEN  % L_k 拉偏是否打开
    
    pendulum_length_rob_1_set = [0, 0.3, -0.3, 0.3,   0,    0,    0,     0, -0.3,    0,     0,     0,   0.3,    0,     -0.3,     0,  0.3, 0.3, 0.3,   0]; % 2024.12.27 的设置
    pendulum_length_rob_2_set = [0, 0.3, -0.3,   0, 0.3,    0,    0,     0,    0, -0.3,     0,     0,   0.3,    0,     -0.3,     0,  0.3, 0.3,   0, 0.3]; % 2024.12.27 的设置
    pendulum_length_rob_3_set = [0, 0.5, -0.5,   0,   0,  0.5,  0.2,     0,    0,    0,  -0.5,     0,     0,  0.5,        0,  -0.5,  0.5,   0, 0.5, 0.5]; % 2024.12.27 的设置
    pendulum_length_rob_4_set = [0, 0.5, -0.5,   0,   0,    0,    0,   0.5,    0,    0,     0,  -0.5,     0,  0.5,        0,  -0.5,    0, 0.5, 0.5, 0.5]; % 2024.12.27 的设置
% pendulum_length_rob_index   = [1,   2,    3,   4,   5,    6,    7,     8,    9,    10,    11,    12,   13,   14,       15,    16,   17,  18,  19,  20]
    
%     pendulum_length_rob_1 = pendulum_length_rob_1_set(pendulum_length_rob_index);
%     pendulum_length_rob_2 = pendulum_length_rob_2_set(pendulum_length_rob_index);
%     pendulum_length_rob_3 = pendulum_length_rob_3_set(pendulum_length_rob_index);
%     pendulum_length_rob_4 = pendulum_length_rob_4_set(pendulum_length_rob_index);

else
    
    if PENDULUM_ROBUST_KEY2 == ON
    pendulum_length_rob_1 = 0.8 + (1.2-0.8)*rand(1);  %  -20% ~ 20%
    pendulum_length_rob_2 = 0.8 + (1.2-0.8)*rand(1);  %  -20% ~ 20%
    pendulum_length_rob_3 = 0.8 + (1.12-0.8)*rand(1);  %  -20% ~ 12%
    pendulum_length_rob_4 = 0.8 + (1.12-0.8)*rand(1);  %  -20% ~ 12%
    else
    pendulum_length_rob_1 = 1;
    pendulum_length_rob_2 = 1;
    pendulum_length_rob_3 = 1;
    pendulum_length_rob_4 = 1;
    end
end
pendulum_length_rob_set = [pendulum_length_rob_1, pendulum_length_rob_2, pendulum_length_rob_3, pendulum_length_rob_4];

if M_K_ROBUST_KEY == ON % m_k 拉偏是否打开 
    m_1_mul_fac = 0.9+(1.1-0.9)*rand(1);  %  -10% ~ 10%
    m_2_mul_fac = 0.9+(1.1-0.9)*rand(1);  %  -10% ~ 10%
    m_3_mul_fac = 0.9+(1.1-0.9)*rand(1);  %  -10% ~ 10%
    m_4_mul_fac = 0.9+(1.1-0.9)*rand(1);  %  -10% ~ 10%
else
    m_1_mul_fac = 1;
    m_2_mul_fac = 1;
    m_3_mul_fac = 1;
    m_4_mul_fac = 1;
end
m_1_4_mul_fac_group = [m_1_mul_fac, m_2_mul_fac, m_3_mul_fac, m_4_mul_fac];


% 不管上面随机为什么值，均重新设为固定值，以看某值下的仿真结果

% excel_rand_row = 5
% m_1_4_mul_fac_group = [0.94, 0.91, 1.03, 0.96];
% pendulum_length_rob_set = [0.88, 0.85, 0.87, 0.85];
% L12_rand_val = [0.07, -0.12];
% L34_rand_val = [-0.45, -0.01];
% m_T1_4_add_fac_group = [9.74, 16.24, -124.8, -62.4];
%%
% 在此处关闭所有图片主要体现在用for循环时，
% 如果不关闭，图片一直生成，会极大拖慢运行速度。    
close all; 

% 与保存变量相关的开关
SAVE_TANK_INFO = OFF; % 保存贮箱信息
SAVE_RES = OFF; % 保存结果

% 与画图相关的开关
FIG_4_SEE_ABNORMAL_DATA = OFF;
FIG_4_CHECK_ABNORMAL_DATA = OFF;
FIG_SAV_OMEGA_WOS = OFF; % 保存图时，图名称为 omega_0 和 w_o 的参数，以方便看出不同控制参数下的结果，便于观察不同控制参数对被控量等的影响。可用于拉偏验证结束后，选出较差的拉偏量参数，修改控制增益，以再次验证该参数下的性能
FIG_SAV_LM = ON;  % 记录拉偏量的索引值（大小）

% 与一阶滤波环节是否生效相关的开关
DISTURB_FILTER_KEY = ON;  % 抵抗gamma 扰动的滤波开关是否打开

% 与 控制输入的延迟 是否打开相关的开关--具体延迟几个步长需要另外修改
T_CTR_DEFERING = OFF;

% 与数据拟合方式(插值还是拟合)相关的开关
EXCEL_VALUE = OFF; % ON \OFF；%决定贮箱1和贮箱2是用拟合值 还是用 EXCEL表的值; ON是用拟合值，OFF是用EXCEL表的值
PRE_INTERPOLATION = OFF;

FITTING_INTERPOLATION = FITTING; % VALUE = INTERPOLATION / FITTING 
PENDULUM_1 = FITTING; % VALUE = INTERPOLATION / FITTING
FITTING_ERROR = 0.07; % 仅针对摆长1是拟合的设定时，可选 VALUE = 0.07/0.2

CHECK_FITTING_RES = OFF;

%%
% 读入储箱数据

% 导入Excel文件数据,读取4个贮箱的数据
tank1_fuel = xlsread('tank1tank2.xls', 1);
tank2_oxgen = xlsread('tank1tank2.xls', 2);
tank3_oxgen = xlsread('tank3tank4.xls', 2);
tank4_fuel = xlsread('tank3tank4.xls', 1);
if SAVE_TANK_INFO == ON
    savefile = 'four_tanks_info';
    save(savefile,'tank1_fuel', 'tank2_oxgen','tank3_oxgen','tank4_fuel');
end

% calculate the size of the tankdata;
[row_num, cln_num] = size(tank3_oxgen);
% return the value of any-row as the initial state;
init_row = randi(row_num);

% 做成元胞，便于循环
tank_dataset = cell(1,4);
tank_dataset{1} = tank1_fuel;
tank_dataset{2} = tank2_oxgen;
tank_dataset{3} = tank3_oxgen;
tank_dataset{4} = tank4_fuel;

%%
% 20241010
% 定义4个元胞，用于存储4个贮箱中各列的“原始值”，用cell的好处是各个维度可以不同
mT_info = cell(1,4);
m0_info = cell(1,4);
m_info = cell(1,4);
L_0_x_info = cell(1,4);
L_x_info = cell(1,4);
pendulum_length_info = cell(1,4);
I0xx_info = cell(1,4);
I0yy_info = cell(1,4);
I0zz_info = cell(1,4);
I0xy_info = cell(1,4);
I0xz_info = cell(1,4);
I0yz_info = cell(1,4);
zeta_ratio_info = cell(1,4);

% 读取4个贮箱中各列的值，便于拟合变化趋势
for i = 1 : 4
     mT_info{i} = tank_dataset{i}(:,2);
     m0_info{i} = tank_dataset{i}(:,3);
     m_info{i} = tank_dataset{i}(:,11);
     L_0_x_info{i} = tank_dataset{i}(:,4);
     L_x_info{i} = tank_dataset{i}(:,12);
     pendulum_length_info{i} = tank_dataset{i}(:,14);
     I0xx_info{i} = tank_dataset{i}(:,5);
     I0yy_info{i} = tank_dataset{i}(:,6);
     I0zz_info{i} = tank_dataset{i}(:,7);
     I0xy_info{i} = tank_dataset{i}(:,8);
     I0xz_info{i} = tank_dataset{i}(:,9);
     I0yz_info{i} = tank_dataset{i}(:,10);
     zeta_ratio_info{i} = tank_dataset{i}(:,16);
end

% 画出4个贮箱中各个量随推进剂质量的变化图，以便查看是否有异常数据点
if FIG_4_SEE_ABNORMAL_DATA == ON
    figure(1001)
    subplot(4,3,1),plot( mT_info{1}, m0_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([0 2000]),legend('m01','Fontsize',15),grid
    subplot(4,3,2),plot( mT_info{1}, m_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([0 800]),legend('m1','Fontsize',15),grid
    subplot(4,3,3),plot( mT_info{1}, L_0_x_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-1  1]),legend('L01x','Fontsize',15),grid
    subplot(4,3,4),plot( mT_info{1}, L_x_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-0.6 0.6]),legend('L1x','Fontsize',15),grid
    subplot(4,3,5),plot( mT_info{1}, pendulum_length_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([0 15]),legend('length1','Fontsize',15),grid
    subplot(4,3,6),plot( mT_info{1}, zeta_ratio_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([0 0.03]),legend('ckesa1','Fontsize',15),grid
    subplot(4,3,7),plot( mT_info{1}, I0xx_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-200 1500]),legend('I01xx','Fontsize',15),grid
    subplot(4,3,8),plot( mT_info{1}, I0yy_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-200 1500]),legend('I01yy','Fontsize',15),grid
    subplot(4,3,9),plot( mT_info{1}, I0zz_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-4e-5 4e-5]),legend('I01zz','Fontsize',15),grid
    subplot(4,3,10),plot( mT_info{1}, I0xy_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-9e-4 9e-4]),legend('I01xy','Fontsize',15),grid
    subplot(4,3,11),plot( mT_info{1}, I0xz_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-9e-4 9e-4]),legend('I01xz','Fontsize',15),grid
    subplot(4,3,12),plot( mT_info{1}, I0yz_info{1},'o','LineWidth',1.5),xlim([0 3000]),ylim([-5e-3 5e-3]),legend('I01yz','Fontsize',15),grid
    
    figure(1002)
    subplot(4,3,1),plot( mT_info{2}, m0_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([0 3000]),legend('m02','Fontsize',15),grid
    subplot(4,3,2),plot( mT_info{2}, m_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([0 2000]),legend('m2','Fontsize',15),grid
    subplot(4,3,3),plot( mT_info{2}, L_0_x_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-0.5 1.2]),legend('L02x','Fontsize',15),grid
    subplot(4,3,4),plot( mT_info{2}, L_x_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-1 0]),legend('L2x','Fontsize',15),grid
    subplot(4,3,5),plot( mT_info{2}, pendulum_length_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([0 2]),legend('length2','Fontsize',15),grid
    subplot(4,3,6),plot( mT_info{2}, zeta_ratio_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-4e-3 4e-3]),legend('ckesa2','Fontsize',15),grid
    subplot(4,3,7),plot( mT_info{2}, I0xx_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-200 400]),legend('I02xx','Fontsize',15),grid
    subplot(4,3,8),plot( mT_info{2}, I0yy_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-20 400]),legend('I02yy','Fontsize',15),grid
    subplot(4,3,9),plot( mT_info{2}, I0zz_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-6e-4 6e-4]),legend('I02zz','Fontsize',15),grid
    subplot(4,3,10),plot( mT_info{2}, I0xy_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-20 100]),legend('I02xy','Fontsize',15),grid
    subplot(4,3,11),plot( mT_info{2}, I0xz_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-0.03 0.03]),legend('I02xz','Fontsize',15),grid
    subplot(4,3,12),plot( mT_info{2}, I0yz_info{2},'o','LineWidth',1.5),xlim([0 4000]),ylim([-3e-3 20e-3]),legend('I02yz','Fontsize',15),grid
    
    figure(1003)
    subplot(4,3,1),plot( mT_info{3}, m0_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([0 6000]),legend('m03','Fontsize',15),grid
    subplot(4,3,2),plot( mT_info{3}, m_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([0 8000]),legend('m3','Fontsize',15),grid
    subplot(4,3,3),plot( mT_info{3}, L_0_x_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([0 4]),legend('L03x','Fontsize',15),grid
    subplot(4,3,4),plot( mT_info{3}, L_x_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([0 2]),legend('L3x','Fontsize',15),grid
    subplot(4,3,5),plot( mT_info{3}, pendulum_length_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([0 3]),legend('length3','Fontsize',15),grid
    subplot(4,3,6),plot( mT_info{3}, zeta_ratio_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-2e-3 4e-3]),legend('ckesa3','Fontsize',15),grid
    subplot(4,3,7),plot( mT_info{3}, I0xx_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-500 3000]),legend('I03xx','Fontsize',15),grid
    subplot(4,3,8),plot( mT_info{3}, I0yy_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-500 2000]),legend('I03yy','Fontsize',15),grid
    subplot(4,3,9),plot( mT_info{3}, I0zz_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-0.02 0.02]),legend('I03zz','Fontsize',15),grid
    subplot(4,3,10),plot( mT_info{3}, I0xy_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-1500 1500]),legend('I03xy','Fontsize',15),grid
    subplot(4,3,11),plot( mT_info{3}, I0xz_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-0.2 0.7]),legend('I03xz','Fontsize',15),grid
    subplot(4,3,12),plot( mT_info{3}, I0yz_info{3},'o','LineWidth',1.5),xlim([0 10000]),ylim([-0.05 0.07]),legend('I03yz','Fontsize',15),grid
    
    figure(1004)
    subplot(4,3,1),plot( mT_info{4}, m0_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 2000]),legend('m04','Fontsize',15),grid
    subplot(4,3,2),plot( mT_info{4}, m_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 4000]),legend('m4','Fontsize',15),grid
    subplot(4,3,3),plot( mT_info{4}, L_0_x_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 10]),legend('L04x','Fontsize',15),grid
    subplot(4,3,4),plot( mT_info{4}, L_x_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 1]),legend('L4x','Fontsize',15),grid
    subplot(4,3,5),plot( mT_info{4}, pendulum_length_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 3]),legend('length4','Fontsize',15),grid
    subplot(4,3,6),plot( mT_info{4}, zeta_ratio_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([0 8e-3]),legend('ckesa4','Fontsize',15),grid
    subplot(4,3,7),plot( mT_info{4}, I0xx_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-300 300]),legend('I04xx','Fontsize',15),grid
    subplot(4,3,8),plot( mT_info{4}, I0yy_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-300 300]),legend('I04yy','Fontsize',15),grid
    subplot(4,3,9),plot( mT_info{4}, I0zz_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-1 1]),legend('I04zz','Fontsize',15),grid
    subplot(4,3,10),plot( mT_info{4}, I0xy_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-4e-3 4e-3]),legend('I04xy','Fontsize',15),grid
    subplot(4,3,11),plot( mT_info{4}, I0xz_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-5e-3 5e-3]),legend('I04xz','Fontsize',15),grid
    subplot(4,3,12),plot( mT_info{4}, I0yz_info{4},'o','LineWidth',1.5),xlim([0 6000]),ylim([-5e-3 5e-3]),legend('I04yz','Fontsize',15),grid
end

if FITTING_INTERPOLATION == INTERPOLATION

% 定义4个元胞，用于存储4个贮箱中各列的“预处理后的值”，用cell的好处是各个维度可以不同
% 增加变量方便看处理前后的差异
% predealed_mT_info = cell(1,4);
predealed_m0_info = cell(1,4);
predealed_m_info = cell(1,4);
predealed_L_0_x_info = cell(1,4);
predealed_L_x_info = cell(1,4);
predealed_pendulum_length_info = cell(1,4);
predealed_I0xx_info = cell(1,4);
predealed_I0yy_info = cell(1,4);
predealed_I0zz_info = cell(1,4);
predealed_I0xy_info = cell(1,4);
predealed_I0xz_info = cell(1,4);
predealed_I0yz_info = cell(1,4);
predealed_zeta_ratio_info = cell(1,4);

% 正常数据存储于变量predealed_***中，两组变量可方便看处理前后的差异
for i = 1 : 4
    predealed_m0_info{i} =  m0_info{i};
    predealed_m_info{i} =  m_info{i};
    predealed_L_0_x_info{i} =  L_0_x_info{i};
    predealed_L_x_info{i} =  L_x_info{i};
    predealed_pendulum_length_info{i} =  pendulum_length_info{i};
    predealed_I0xx_info{i} =  I0xx_info{i};
    predealed_I0yy_info{i} =  I0yy_info{i};
    predealed_I0zz_info{i} =  I0zz_info{i};
    predealed_I0xy_info{i} =  I0xy_info{i};
    predealed_I0xz_info{i} =  I0xz_info{i};
    predealed_I0yz_info{i} =  I0yz_info{i};
    predealed_zeta_ratio_info{i} = zeta_ratio_info{i};
end

% 异常数据的处理
predealed_L_0_x_info{1} = utils().abnorm_data_dealing('L01x', L_0_x_info{1});
predealed_L_x_info{1} = utils().abnorm_data_dealing('L1x', L_x_info{1});
predealed_zeta_ratio_info{1} = utils().abnorm_data_dealing("ckesa1", zeta_ratio_info{1});

predealed_L_x_info{2} = utils().abnorm_data_dealing('L2x', L_x_info{2});
predealed_I0xx_info{2} = utils().abnorm_data_dealing('I02xx', I0xx_info{2});
predealed_I0zz_info{2} = utils().abnorm_data_dealing('I02zz', I0zz_info{2});

predealed_I0xx_info{3} = utils().abnorm_data_dealing('I03xx', I0xx_info{3});
predealed_I0yy_info{3} = utils().abnorm_data_dealing('I03yy', I0yy_info{3});
predealed_I0zz_info{3} = utils().abnorm_data_dealing('I03zz', I0zz_info{3});

% 查看异常数据处理前后的差异
if FIG_4_CHECK_ABNORMAL_DATA == ON
    figure(1005)
    subplot(3,3,1),plot(mT_info{1}, L_0_x_info{1},'o',mT_info{1},predealed_L_0_x_info{1},':.','LineWidth',1.5),xlim([0 3000]),ylim([-1  1]),legend('raw L01x','corrected L01x','Fontsize',15),grid
    subplot(3,3,2),plot(mT_info{1}, L_x_info{1},'o',mT_info{1},predealed_L_x_info{1},':.','LineWidth',1.5),xlim([0 3000]),ylim([-0.6 0.6]),legend('raw L1x','corrected L1x','Fontsize',15),grid
    subplot(3,3,3),plot(mT_info{1}, zeta_ratio_info{1},'o',mT_info{1},predealed_zeta_ratio_info{1},':.','LineWidth',1.5),xlim([0 3000]),ylim([0 0.03]),legend('raw ckesa1','corrected ckesa1','Fontsize',15),grid

    subplot(3,3,4),plot(mT_info{2}, L_x_info{2},'o',mT_info{2},predealed_L_x_info{2},':.','LineWidth',1.5),xlim([0 4000]),ylim([-1  0]),legend('raw L2x','corrected L2x','Fontsize',15),grid
    subplot(3,3,5),plot(mT_info{2}, I0xx_info{2},'o',mT_info{2},predealed_I0xx_info{2},':.','LineWidth',1.5),xlim([0 4000]),ylim([-200  400]),legend('raw I02xx','corrected I02xx','Fontsize',15),grid
    subplot(3,3,6),plot(mT_info{2}, I0zz_info{2},'o',mT_info{2},predealed_I0zz_info{2},':.','LineWidth',1.5),xlim([0 4000]),ylim([-6e-4 6e-4]),legend('raw I02zz','corrected I02zz','Fontsize',15),grid
 
    subplot(3,3,7),plot(mT_info{3}, I0xx_info{3},'o',mT_info{3},predealed_I0xx_info{3},':.','LineWidth',1.5),xlim([0 10000]),ylim([-500 3000]),legend('raw I03xx','corrected I03xx','Fontsize',15),grid
    subplot(3,3,8),plot(mT_info{3}, I0yy_info{3},'o',mT_info{3},predealed_I0yy_info{3},':.','LineWidth',1.5),xlim([0 10000]),ylim([-500 2000]),legend('raw I03yy','corrected I03yy','Fontsize',15),grid
    subplot(3,3,9),plot(mT_info{3}, I0zz_info{3},'o',mT_info{3},predealed_I0zz_info{3},':.','LineWidth',1.5),xlim([0 10000]),ylim([-0.02 0.02]),legend('raw I03zz','corrected I03zz','Fontsize',15),grid
end

% ****************************
% 4个贮箱异常数据处理后的 数据插值结果--如果保存后再读值，会更快
% 查询点的步进大小 对插值结果无影响。以特定点为采样点，若该点不存在 不影响该点的结果。
if PRE_INTERPOLATION == ON
    mT_info_query_res1 = 0:0.001:max(mT_info{1});
    m0_info_query_res1 = interp1(mT_info{1}, predealed_m0_info{1}, mT_info_query_res1, 'linear','extrap');
    m_info_query_res1 = interp1(mT_info{1}, predealed_m_info{1}, mT_info_query_res1, 'linear','extrap');
    L_0_x_info_query_res1 = interp1(mT_info{1}, predealed_L_0_x_info{1}, mT_info_query_res1, 'linear','extrap');
    L_x_info_query_res1 = interp1(mT_info{1}, predealed_L_x_info{1}, mT_info_query_res1, 'linear','extrap');
    I0xx_info_query_res1 = interp1(mT_info{1}, predealed_I0xx_info{1}, mT_info_query_res1, 'linear','extrap');
    I0yy_info_query_res1 = interp1(mT_info{1}, predealed_I0yy_info{1}, mT_info_query_res1, 'linear','extrap');
    I0zz_info_query_res1 = interp1(mT_info{1}, predealed_I0zz_info{1}, mT_info_query_res1, 'linear','extrap');
    I0xy_info_query_res1 = interp1(mT_info{1}, predealed_I0xy_info{1}, mT_info_query_res1, 'linear','extrap');
    I0xz_info_query_res1 = interp1(mT_info{1}, predealed_I0xz_info{1}, mT_info_query_res1, 'linear','extrap');
    I0yz_info_query_res1 = interp1(mT_info{1}, predealed_I0yz_info{1}, mT_info_query_res1, 'linear','extrap');
    pendulum_query_res1 = interp1(mT_info{1}, predealed_pendulum_length_info{1}, mT_info_query_res1, 'linear','extrap');
    zeta_ratio_query_res1 = interp1(mT_info{1}, predealed_zeta_ratio_info{1}, mT_info_query_res1, 'linear','extrap');
    
    mT_info_query_res2 = 0:0.001:max(mT_info{2});
    m0_info_query_res2 = interp1(mT_info{2}, predealed_m0_info{2}, mT_info_query_res2, 'linear','extrap');
    m_info_query_res2 = interp1(mT_info{2}, predealed_m_info{2}, mT_info_query_res2, 'linear','extrap');
    L_0_x_info_query_res2 = interp1(mT_info{2}, predealed_L_0_x_info{2}, mT_info_query_res2, 'linear','extrap');
    L_x_info_query_res2 = interp1(mT_info{2}, predealed_L_x_info{2}, mT_info_query_res2, 'linear','extrap');
    I0xx_info_query_res2 = interp1(mT_info{2}, predealed_I0xx_info{2}, mT_info_query_res2, 'linear','extrap');
    I0yy_info_query_res2 = interp1(mT_info{2}, predealed_I0yy_info{2}, mT_info_query_res2, 'linear','extrap');
    I0zz_info_query_res2 = interp1(mT_info{2}, predealed_I0zz_info{2}, mT_info_query_res2, 'linear','extrap');
    I0xy_info_query_res2 = interp1(mT_info{2}, predealed_I0xy_info{2}, mT_info_query_res2, 'linear','extrap');
    I0xz_info_query_res2 = interp1(mT_info{2}, predealed_I0xz_info{2}, mT_info_query_res2, 'linear','extrap');
    I0yz_info_query_res2 = interp1(mT_info{2}, predealed_I0yz_info{2}, mT_info_query_res2, 'linear','extrap');
    pendulum_query_res2 = interp1(mT_info{2}, predealed_pendulum_length_info{2}, mT_info_query_res2, 'linear','extrap');
    zeta_ratio_query_res2 = interp1(mT_info{2}, predealed_zeta_ratio_info{2}, mT_info_query_res2, 'linear','extrap');
    
    mT_info_query_res3 = 0:0.001:max(mT_info{3});
    m0_info_query_res3 = interp1(mT_info{3}, predealed_m0_info{3}, mT_info_query_res3, 'linear','extrap');
    m_info_query_res3 = interp1(mT_info{3}, predealed_m_info{3}, mT_info_query_res3, 'linear','extrap');
    L_0_x_info_query_res3 = interp1(mT_info{3}, predealed_L_0_x_info{3}, mT_info_query_res3, 'linear','extrap');
    L_x_info_query_res3 = interp1(mT_info{3}, predealed_L_x_info{3}, mT_info_query_res3, 'linear','extrap');
    I0xx_info_query_res3 = interp1(mT_info{3}, predealed_I0xx_info{3}, mT_info_query_res3, 'linear','extrap');
    I0yy_info_query_res3 = interp1(mT_info{3}, predealed_I0yy_info{3}, mT_info_query_res3, 'linear','extrap');
    I0zz_info_query_res3 = interp1(mT_info{3}, predealed_I0zz_info{3}, mT_info_query_res3, 'linear','extrap');
    I0xy_info_query_res3 = interp1(mT_info{3}, predealed_I0xy_info{3}, mT_info_query_res3, 'linear','extrap');
    I0xz_info_query_res3 = interp1(mT_info{3}, predealed_I0xz_info{3}, mT_info_query_res3, 'linear','extrap');
    I0yz_info_query_res3 = interp1(mT_info{3}, predealed_I0yz_info{3}, mT_info_query_res3, 'linear','extrap');
    pendulum_query_res3 = interp1(mT_info{3}, predealed_pendulum_length_info{3}, mT_info_query_res3, 'linear','extrap');
    zeta_ratio_query_res3 = interp1(mT_info{3}, predealed_zeta_ratio_info{3}, mT_info_query_res3, 'linear','extrap');
    
    mT_info_query_res4 = 0:0.001:max(mT_info{4});
    m0_info_query_res4 = interp1(mT_info{4}, predealed_m0_info{4}, mT_info_query_res4, 'linear','extrap');
    m_info_query_res4 = interp1(mT_info{4}, predealed_m_info{4}, mT_info_query_res4, 'linear','extrap');
    L_0_x_info_query_res4 = interp1(mT_info{4}, predealed_L_0_x_info{4}, mT_info_query_res4, 'linear','extrap');
    L_x_info_query_res4 = interp1(mT_info{4}, predealed_L_x_info{4}, mT_info_query_res4, 'linear','extrap');
    I0xx_info_query_res4 = interp1(mT_info{4}, predealed_I0xx_info{4}, mT_info_query_res4, 'linear','extrap');
    I0yy_info_query_res4 = interp1(mT_info{4}, predealed_I0yy_info{4}, mT_info_query_res4, 'linear','extrap');
    I0zz_info_query_res4 = interp1(mT_info{4}, predealed_I0zz_info{4}, mT_info_query_res4, 'linear','extrap');
    I0xy_info_query_res4 = interp1(mT_info{4}, predealed_I0xy_info{4}, mT_info_query_res4, 'linear','extrap');
    I0xz_info_query_res4 = interp1(mT_info{4}, predealed_I0xz_info{4}, mT_info_query_res4, 'linear','extrap');
    I0yz_info_query_res4 = interp1(mT_info{4}, predealed_I0yz_info{4}, mT_info_query_res4, 'linear','extrap');
    pendulum_query_res4 = interp1(mT_info{4}, predealed_pendulum_length_info{4}, mT_info_query_res4, 'linear','extrap');
    zeta_ratio_query_res4 = interp1(mT_info{4}, predealed_zeta_ratio_info{4}, mT_info_query_res4, 'linear','extrap');
end
end

% 拟合4个贮箱中各列的值
% 拟合使用了原始值。使用原始数据拟合前，如果该数据列含有异常数据，已在函数内部对异常数据做了“最近邻”处理。
% *************拟合第1个贮箱***************
[ m01_params,  m01_MU] = utils().poly_func_fit(mT_info{1}, m0_info{1});
m1_segm_point = 1558;
[m1_params1, m1_params2] = utils().segm_func_fit("m1",   mT_info{1}, m_info{1},    m1_segm_point);

I01xx_segm_point = 358;
I01yy_segm_point = 358;
[I01xx_params1, I01xx_params2] = utils().segm_func_fit("I01xx", mT_info{1}, I0xx_info{1}, I01xx_segm_point);
[I01yy_params1, I01yy_params2] = utils().segm_func_fit("I01yy", mT_info{1}, I0yy_info{1}, I01yy_segm_point);
% 下面四个拟合的参数 实则没用到，在读取时 默认以下四个变量 全0.
[I01zz_params, I01zz_MU] = utils().poly_func_fit(mT_info{1},I0zz_info{1});
[I01xy_params, I01xy_MU] = utils().poly_func_fit(mT_info{1},I0xy_info{1});
[I01xz_params, I01xz_MU] = utils().poly_func_fit(mT_info{1},I0xz_info{1});
[I01yz_params, I01yz_MU] = utils().poly_func_fit(mT_info{1},I0yz_info{1});

% 抛物线--拟合, 适用于如下变量的变化-- 第一个贮箱 L01x/L1x/length1/ckesa1
L01x_fluc_amp = 0.2;
L1x_fluc_amp = 0.05;
length1_fluc_amp = 1000;
% ckesa1_fluc_amp = 0.0005;
% [L01x_params]         = utils().parabola_func_fit(mT_info{1}, L_0_x_info{1}, L01x_fluc_amp);
% [L1x_params]          = utils().parabola_func_fit(mT_info{1}, L_x_info{1}, L1x_fluc_amp);
% [length1_params]      = utils().parabola_func_fit(mT_info{1}, pendulum_length_info{1}, length1_fluc_amp);
% [ckesa1_params]       = utils().parabola_func_fit(mT_info{1}, zeta_ratio_info{1}, ckesa1_fluc_amp);

% 第一个贮箱 L01x/L1x/length1-->改为三次多项式--拟合
% 以下前两个变量 先做了预处理，再用三次多项式拟合，用的poly_func_fit2函数。
% 由于poly_func_fit2加了个amp的阈值，因此poly_func_fit2未和poly_func_fit函数统一，
% 统一的话需要确认其它各个量的最大差值，越大越好。
[L01x_params, L01x_MU]       = utils().poly_func_fit2(mT_info{1}, L_0_x_info{1}, L01x_fluc_amp);
[L1x_params, L1x_MU]         = utils().poly_func_fit2(mT_info{1}, L_x_info{1}, L1x_fluc_amp);
% length1, ckesa1 不做处理效果也很好
if PENDULUM_1 == FITTING
    if FITTING_ERROR == 0.2
        [length1_params, length1_MU] = utils().poly_func_fit(mT_info{1}, pendulum_length_info{1});%20250310弃用
    % length1_segm_point = 794.9;
    % [length1_params1, length1_params2] = utils().segm_func_fit("length1",   mT_info{1}, pendulum_length_info{1}, length1_segm_point);
    else % FITTING_ERROR == 0.07
        [length1_params, length1_MU] = utils().pendulum_1_func_fit(mT_info{1}, pendulum_length_info{1});
    end
else
    [length1_xi,length1_yi] = utils().pendulum_1_func_fit2(mT_info{1}, pendulum_length_info{1}); %20250312启用
    % 此处多做了一次插值，该步骤不影响各时刻 贮箱1的摆长读的插值结果。此处仅是多耗了一行指令时间。
    % 为了后续不出错，不再删除了-2025.04.18
end
[ckesa1_params, ckesa1_MU]   = utils().poly_func_fit(mT_info{1}, zeta_ratio_info{1});

% *************拟合第2个贮箱***************
[ m02_params,  m02_MU]   = utils().poly_func_fit(mT_info{2}, m0_info{2});
[m2_params  ,   m2_MU]   = utils().poly_func_fit(mT_info{2}, m_info{2});
I02xx_segm_point = 1800;
I02yy_segm_point = 1800;
[I02xx_params1, I02xx_params2] = utils().segm_func_fit("I02xx", mT_info{2}, I0xx_info{2}, I02xx_segm_point);
[I02yy_params1, I02yy_params2] = utils().segm_func_fit("I02yy", mT_info{2}, I0yy_info{2}, I02yy_segm_point);
% 下面四个拟合的参数 实则没用到，在读取时 默认以下四个变量 全0.
[I02zz_params, I02zz_MU] = utils().poly_func_fit(mT_info{2},I0zz_info{2});
[I02xy_params, I02xy_MU] = utils().poly_func_fit(mT_info{2},I0xy_info{2});
[I02xz_params, I02xz_MU] = utils().poly_func_fit(mT_info{2},I0xz_info{2});
[I02yz_params, I02yz_MU] = utils().poly_func_fit(mT_info{2},I0yz_info{2});

[L02x_params,  L02x_MU] = utils().poly_func_fit(mT_info{2},L_0_x_info{2});
[L2x_params,    L2x_MU] = utils().poly_func_fit(mT_info{2},L_x_info{2});
[length2_params, length2_MU] = utils().poly_func_fit(mT_info{2},pendulum_length_info{2});
[ckesa2_params, ckesa2_MU] = utils().poly_func_fit(mT_info{2},zeta_ratio_info{2});

% *************拟合第3个贮箱***************
[ m03_params,  m03_MU]   = utils().poly_func_fit(mT_info{3}, m0_info{3});
[m3_params  ,   m3_MU]   = utils().poly_func_fit(mT_info{3}, m_info{3});

I03xx_segm_point = 6000;
I03yy_segm_point = 6000;
[I03xx_params1, I03xx_params2] = utils().segm_func_fit("I03xx", mT_info{3}, I0xx_info{3}, I03xx_segm_point);
[I03yy_params1, I03yy_params2] = utils().segm_func_fit("I03yy", mT_info{3}, I0yy_info{3}, I03yy_segm_point);
[I03zz_params, I03zz_MU] = utils().poly_func_fit(mT_info{3},I0zz_info{3});
[I03xy_params, I03xy_MU] = utils().poly_func_fit(mT_info{3},I0xy_info{3});
[I03xz_params, I03xz_MU] = utils().poly_func_fit(mT_info{3},I0xz_info{3});
[I03yz_params, I03yz_MU] = utils().poly_func_fit(mT_info{3},I0yz_info{3});

[L03x_params,  L03x_MU]  = utils().poly_func_fit(mT_info{3},L_0_x_info{3});
[L3x_params,    L3x_MU]  = utils().poly_func_fit(mT_info{3},L_x_info{3});
[length3_params, length3_MU] = utils().poly_func_fit(mT_info{3},pendulum_length_info{3});
[ckesa3_params, ckesa3_MU] = utils().poly_func_fit(mT_info{3},zeta_ratio_info{3});

% *************拟合第4个贮箱***************
[ m04_params,  m04_MU]   = utils().poly_func_fit(mT_info{4}, m0_info{4});
[m4_params  ,   m4_MU]   = utils().poly_func_fit(mT_info{4}, m_info{4});

% [I04xx_params, I04xx_MU] = utils().poly_func_fit(mT_info{4},I0xx_info{4});
% [I04yy_params, I04yy_MU] = utils().poly_func_fit(mT_info{4},I0yy_info{4});
I04xx_segm_point = 3500;
I04yy_segm_point = 3500;
[I04xx_params1, I04xx_params2] = utils().segm_func_fit("I04xx", mT_info{4}, I0xx_info{4}, I04xx_segm_point);
[I04yy_params1, I04yy_params2] = utils().segm_func_fit("I04yy", mT_info{4}, I0yy_info{4}, I04yy_segm_point);
[I04zz_params, I04zz_MU] = utils().poly_func_fit(mT_info{4},I0zz_info{4});
[I04xy_params, I04xy_MU] = utils().poly_func_fit(mT_info{4},I0xy_info{4});
[I04xz_params, I04xz_MU] = utils().poly_func_fit(mT_info{4},I0xz_info{4});
[I04yz_params, I04yz_MU] = utils().poly_func_fit(mT_info{4},I0yz_info{4});

[L04x_params,  L04x_MU]  = utils().poly_func_fit(mT_info{4},L_0_x_info{4});
[L4x_params,    L4x_MU]  = utils().poly_func_fit(mT_info{4},L_x_info{4});
[length4_params, length4_MU] = utils().poly_func_fit(mT_info{4},pendulum_length_info{4});
[ckesa4_params, ckesa4_MU] = utils().poly_func_fit(mT_info{4},zeta_ratio_info{4});

%%
% ...（1）步长为0.01，控制间隔为10个步长的时候，可以控住
% ...（2）加入ESO的时候，需要把 w_o减小。
% ...（3）步长为0.01，控制间隔和ESO间隔均为10个步长，
%      且 w_o = 100 改小为 w_os=5 的时候，输出可以控住

% 给定贮箱1234的初始质量
m_T1_init_state = [1612,1712,1765.361,1912,2100,1874];  % 选定的某个值 在整个600秒内 都一直固定
% rand_m = randi([1,5]);
% rand_m =  3; % 3
m_T1_init_value = m_T1_init_state(rand_m);
m_T2_init_value =(1458/874.4) * m_T1_init_value;
disp(['随机数 rand_m ：', num2str(rand_m)]);
disp(['m_T1_init_value：', num2str(m_T1_init_value)]);
disp(['m_T2_init_value：', num2str(m_T2_init_value)]);

m_T4_init_state = [4918.232, 4718, 4518, 4318, 4000,3016.67,2453.3]; % 4000 2000 1000
% rand_k = randi([1,5]);
% rand_k = 5; % 5
m_T4_init_value = m_T4_init_state(rand_k);
m_T3_init_value = 2 * m_T4_init_value;
disp(['随机数 rand_k ：', num2str(rand_k)]);
disp(['m_T3_init_value：', num2str(m_T3_init_value)]);
disp(['m_T4_init_value：', num2str(m_T4_init_value)]);


% Time vector
t_end = 600;
t_interval = 0.0128;

B0_INSTANENOUS = OFF;

T_CTR = ON; % T_CTR标识是否是多次步长后更新U；值为1 表示T_ctr个步长后更新U，值为0则每次步长更新U。
T_ctr = 10 * t_interval; % 10个步长控一次

T_ESO = ON; % T_ESO标识是否是多次步长后更新ESO的参数；值为1 表示T_eso个步长后更新，值为0则每次步长更新。
T_eso = 10 * t_interval;

T_fig = 10 * t_interval + t_interval; % 每隔 T_ctr 个周期更新U后，更新值在下一个循环会被记录，故将 T_fig 多加个 t_interval。

% omegaa_c = 0.05 & w_os = 20(factor = 5)
% 控制增益
omegaa_c = 0.25; %--0.25--0.1--0.5--selected 0.25 %%%new selected 0.05  0.25
kk1 = omegaa_c ^2;
k11 = kk1;
k12 = kk1;
k13 = kk1;
k1 = [k11 0 0; 0 k12 0; 0 0 k13;] ;

kk2 = 1.5*2 * omegaa_c;
k21 = kk2;
k22 = kk2;
k23 = kk2;
k2 = [k21 0 0; 0 k22 0; 0 0 k23;] ;

% 观测器参数
ww = 100;
% ww = 0.2 + (20/m_T4_init_value) * m_T4_init_value;
w_o1 = ww;
w_o2 = ww;
w_o3 = ww;
w_o = [w_o1 0 0; 0  w_o2 0; 0 0 w_o3;]; % T_CTR_DEFERING ==ON时，改为 0.1/0.01*w_o1
w_os = w_o/5;  % 5 %10/20--hao/50--factor selected 10%%new  selected  5
w_os_norm = w_os;

% 初始状态
% theta_init = [0.02 -0.04 0.04]'; %初始 姿态角 --2024.11.05以前用
theta_init = [0.0017 -0.0035 0.0035]'; %初始 姿态角 --2024.11.05修改
Z_init = [0.000  0.000  0.000]'; %初始 扰动
omegaa_init = [0  0.01 0.01]'; %theta_init/t_interval;
eta_init = -w_o * omegaa_init;
gamma_init = [0   0   0]'; %初始液体晃动摆角
alpha_init = 0;


% 状态初始化
theta = theta_init; % 姿态角初始化
Z = Z_init;  % 扰动初始化
eta = eta_init; % 估扰动用的量 初始化
omegaa = omegaa_init; % 旋转角速度初始化

omegaa_eso = omegaa_init;

d_omegaa = omegaa_init; % 旋转角速度初始化
dd_omegaa = omegaa_init; % 旋转角速度初始化

% 状态初始化
theta_actu = theta_init; % 姿态角初始化
Z_actu = Z_init;  % 扰动初始化
eta_actu = eta_init; % 估扰动用的量 初始化
omegaa_actu = omegaa_init; % 旋转角速度初始化
d_omegaa_actu = omegaa_init; % 旋转角速度初始化
dd_omegaa_actu = omegaa_init; % 旋转角速度初始化

% 摇摆发动机摆角各量初始化
alpha_y = alpha_init;
alpha_z = alpha_init;
d_alpha_y = alpha_init;
d_alpha_z = alpha_init;
d_Mx = alpha_init;
dd_alpha_y = alpha_init;
dd_alpha_z = alpha_init;
dd_Mx = alpha_init;

alpha_y_actu = alpha_init;
alpha_z_actu = alpha_init;
d_alpha_y_actu = alpha_init;
d_alpha_z_actu = alpha_init;
dd_alpha_y_actu = alpha_init;
dd_alpha_z_actu = alpha_init;

% 4个贮箱内晃动液体的摆角量初始化
gamma_1 = gamma_init;
gamma_2 = gamma_init;
gamma_3 = gamma_init;
gamma_4 = gamma_init;

d_gamma_1 = gamma_init;
d_gamma_2 = gamma_init;
d_gamma_3 = gamma_init;
d_gamma_4 = gamma_init;
dd_gamma_1 = gamma_init;
dd_gamma_2 = gamma_init;
dd_gamma_3 = gamma_init;
dd_gamma_4 = gamma_init;

gamma_1_actu = gamma_init;
gamma_2_actu = gamma_init;
gamma_3_actu = gamma_init;
gamma_4_actu = gamma_init;

d_gamma_1_actu = gamma_init;
d_gamma_2_actu = gamma_init;
d_gamma_3_actu = gamma_init;
d_gamma_4_actu = gamma_init;
dd_gamma_1_actu = gamma_init;
dd_gamma_2_actu = gamma_init;
dd_gamma_3_actu = gamma_init;
dd_gamma_4_actu = gamma_init;

theta_res = [];
omegaa_res = [];

U_adrc_res = [];
EST_disturb_res = [];
a_R_res = [];
F_E1_res = [];
F_E_res = [];
T_E1_res = [];

disturb_func_res = [];

U_adrc_actu_res = [];
alpha_y_res = [];
alpha_z_res = [];
d_alpha_y_res = [];
d_alpha_z_res = [];
dd_alpha_y_res = [];
dd_alpha_z_res = [];

gamma_1_res = [];
gamma_2_res = [];
gamma_3_res = [];
gamma_4_res = [];
d_gamma_1_res = [];
d_gamma_2_res = [];
d_gamma_3_res = [];
d_gamma_4_res = [];
dd_gamma_1_res = [];
dd_gamma_2_res = [];
dd_gamma_3_res = [];
dd_gamma_4_res = [];

D_res = [];
f_gamma_res = [];

disturb_func_actu_res = [];

alpha_y_actu_res = [];
alpha_z_actu_res = [];
d_alpha_y_actu_res = [];
d_alpha_z_actu_res = [];
dd_alpha_y_actu_res = [];
dd_alpha_z_actu_res = [];

t4fig = [];
t4ctr = [];
U_adrc_upd_res = [];
t4eso = [];
omegaa_eso_res = [];

matrix_size = floor(t_end/t_interval) + 1;
vary_res_axis = 0:t_interval:t_end;

% 定义矩阵，用于存储每个时刻第二个贮箱内各个变量的值；
zata2_res = zeros(1,matrix_size);

% 定义矩阵，用于存储每个时刻第一个贮箱内各个变量的值；
mT1_res = zeros(1,matrix_size);
m01_res = zeros(1,matrix_size);
m1_res = zeros(1,matrix_size);
L_01_res = zeros(1,matrix_size);
L_1_res = zeros(1,matrix_size);
pendulum_length1_res = zeros(1,matrix_size);
I01xx_res = zeros(1,matrix_size);
I01yy_res = zeros(1,matrix_size);
I01zz_res = zeros(1,matrix_size);
I01xy_res = zeros(1,matrix_size);
I01xz_res = zeros(1,matrix_size);
I01yz_res = zeros(1,matrix_size);
zeta1_ratio_res = zeros(1,matrix_size);

% 定义矩阵，用于存储每个时刻第二个贮箱内各个变量的值；
mT2_res = zeros(1,matrix_size);
m02_res = zeros(1,matrix_size);
m2_res = zeros(1,matrix_size);
L_02_res = zeros(1,matrix_size);
L_2_res = zeros(1,matrix_size);
pendulum_length2_res = zeros(1,matrix_size);
I02xx_res = zeros(1,matrix_size);
I02yy_res = zeros(1,matrix_size);
I02zz_res = zeros(1,matrix_size);
I02xy_res = zeros(1,matrix_size);
I02xz_res = zeros(1,matrix_size);
I02yz_res = zeros(1,matrix_size);
zeta2_ratio_res = zeros(1,matrix_size);

% 定义矩阵，用于存储每个时刻第三个贮箱内各个变量的值；
mT3_res = zeros(1,matrix_size);
m03_res = zeros(1,matrix_size);
m3_res = zeros(1,matrix_size);
L_03_res = zeros(1,matrix_size);
L_3_res = zeros(1,matrix_size);
pendulum_length3_res = zeros(1,matrix_size);
I03xx_res = zeros(1,matrix_size);
I03yy_res = zeros(1,matrix_size);
I03zz_res = zeros(1,matrix_size);
I03xy_res = zeros(1,matrix_size);
I03xz_res = zeros(1,matrix_size);
I03yz_res = zeros(1,matrix_size);
zeta3_ratio_res = zeros(1,matrix_size);

% 定义矩阵，用于存储每个时刻第四个贮箱内各个变量的值；
mT4_res = zeros(1,matrix_size);
m04_res = zeros(1,matrix_size);
m4_res = zeros(1,matrix_size);
L_04_res = zeros(1,matrix_size);
L_4_res = zeros(1,matrix_size);
pendulum_length4_res = zeros(1,matrix_size);
I04xx_res = zeros(1,matrix_size);
I04yy_res = zeros(1,matrix_size);
I04zz_res = zeros(1,matrix_size);
I04xy_res = zeros(1,matrix_size);
I04xz_res = zeros(1,matrix_size);
I04yz_res = zeros(1,matrix_size);
zeta4_ratio_res = zeros(1,matrix_size);
eig_value_res = zeros(30,matrix_size);

Res_before_filter = zeros(3,matrix_size);
Res_after_filter = zeros(3,matrix_size);

B_res = zeros(matrix_size,3,3 );
B_0_res = zeros(matrix_size,3,3);

z_set = cell(3,3,matrix_size);
p_set = cell(3,3,matrix_size);
k_set = cell(3,3,matrix_size);
F_y_zp_set = cell(3,3,matrix_size);
A_mul_F_set = cell(1,matrix_size);

z_unfiltered_set = cell(3,3,matrix_size);
p_unfiltered_set = cell(3,3,matrix_size);
k_unfiltered_set = cell(3,3,matrix_size);

F_y_unfiltered_z_set = cell(3,3,matrix_size);
F_y_unfiltered_p_set = cell(3,3,matrix_size);
F_y_unfiltered_k_set = cell(3,3,matrix_size);


% 定义矩阵，用于存储每个时刻第三个贮箱内各个变量的值；
rate_mT_res = zeros(4,matrix_size);
rate_m0_res = zeros(4,matrix_size);
rate_m_res = zeros(4,matrix_size);
rate_L_0_x_res = zeros(4,matrix_size);
rate_L_x_res = zeros(4,matrix_size);
rate_pendulum_length_res = zeros(4,matrix_size);
rate_delta_L_x_res = zeros(4,matrix_size);
rate_delta_pendulum_length_res = zeros(4,matrix_size);
rate_Ixx_res = zeros(4,matrix_size);
rate_Iyy_res = zeros(4,matrix_size);
rate_Izz_res = zeros(4,matrix_size);
rate_Ixy_res = zeros(4,matrix_size);
rate_Ixz_res = zeros(4,matrix_size);
rate_Iyz_res = zeros(4,matrix_size);
rate_zeta_ratio_res = zeros(4,matrix_size);

fitted_tank1_m0   = zeros(4,matrix_size);
fitted_tank1_I0zz = zeros(4,matrix_size);
fitted_tank1_I0xy = zeros(4,matrix_size);
fitted_tank1_I0xz = zeros(4,matrix_size);
fitted_tank1_I0yz = zeros(4,matrix_size);
fitted_tank1_m    = zeros(4,matrix_size);
fitted_tank1_I0xx = zeros(4,matrix_size); 
fitted_tank1_I0yy = zeros(4,matrix_size);
fitted_tank1_L_0_x = zeros(4,matrix_size);
fitted_tank1_L_x   = zeros(4,matrix_size);
fitted_tank1_pendulum_length = zeros(4,matrix_size);
fitted_tank1_zeta = zeros(4,matrix_size);

fitted_tank2_m0   = zeros(4,matrix_size);
fitted_tank2_I0zz = zeros(4,matrix_size);
fitted_tank2_I0xy = zeros(4,matrix_size);
fitted_tank2_I0xz = zeros(4,matrix_size);
fitted_tank2_I0yz = zeros(4,matrix_size);
fitted_tank2_m    = zeros(4,matrix_size);
fitted_tank2_I0xx = zeros(4,matrix_size); 
fitted_tank2_I0yy = zeros(4,matrix_size);
fitted_tank2_L_0_x = zeros(4,matrix_size);
fitted_tank2_L_x   = zeros(4,matrix_size);
fitted_tank2_pendulum_length = zeros(4,matrix_size);
fitted_tank2_zeta = zeros(4,matrix_size);

fitted_tank3_m0   = zeros(4,matrix_size);
fitted_tank3_I0zz = zeros(4,matrix_size);
fitted_tank3_I0xy = zeros(4,matrix_size);
fitted_tank3_I0xz = zeros(4,matrix_size);
fitted_tank3_I0yz = zeros(4,matrix_size);
fitted_tank3_m    = zeros(4,matrix_size);
fitted_tank3_I0xx = zeros(4,matrix_size); 
fitted_tank3_I0yy = zeros(4,matrix_size);
fitted_tank3_L_0_x = zeros(4,matrix_size);
fitted_tank3_L_x   = zeros(4,matrix_size);
fitted_tank3_pendulum_length = zeros(4,matrix_size);
fitted_tank3_zeta = zeros(4,matrix_size);

fitted_tank4_m0   = zeros(4,matrix_size);
fitted_tank4_I0zz = zeros(4,matrix_size);
fitted_tank4_I0xy = zeros(4,matrix_size);
fitted_tank4_I0xz = zeros(4,matrix_size);
fitted_tank4_I0yz = zeros(4,matrix_size);
fitted_tank4_m    = zeros(4,matrix_size);
fitted_tank4_I0xx = zeros(4,matrix_size); 
fitted_tank4_I0yy = zeros(4,matrix_size);
fitted_tank4_L_0_x = zeros(4,matrix_size);
fitted_tank4_L_x   = zeros(4,matrix_size);
fitted_tank4_pendulum_length = zeros(4,matrix_size);
fitted_tank4_zeta = zeros(4,matrix_size);

fitted_tank1_m0_excel   = zeros(35,1);
fitted_tank1_I0zz_excel = zeros(35,1);
fitted_tank1_I0xy_excel = zeros(35,1);
fitted_tank1_I0xz_excel = zeros(35,1);
fitted_tank1_I0yz_excel = zeros(35,1);
fitted_tank1_m_excel    = zeros(35,1);
fitted_tank1_I0xx_excel = zeros(35,1); 
fitted_tank1_I0yy_excel = zeros(35,1);
fitted_tank1_L_0_x_excel = zeros(35,1);
fitted_tank1_L_x_excel   = zeros(35,1);
% fitted_tank1_pendulum_length_excel = zeros(35,1);
fitted_tank1_zeta_excel = zeros(35,1);

rate_fitted_tank1_m0_excel   = zeros(35,1);
rate_fitted_tank1_I0zz_excel = zeros(35,1);
rate_fitted_tank1_I0xy_excel = zeros(35,1);
rate_fitted_tank1_I0xz_excel = zeros(35,1);
rate_fitted_tank1_I0yz_excel = zeros(35,1);
rate_fitted_tank1_m_excel    = zeros(35,1);
rate_fitted_tank1_I0xx_excel = zeros(35,1); 
rate_fitted_tank1_I0yy_excel = zeros(35,1);
rate_fitted_tank1_L_0_x_excel = zeros(35,1);
rate_fitted_tank1_L_x_excel   = zeros(35,1);
rate_fitted_tank1_pendulum_length_excel = zeros(35,1);
rate_fitted_tank1_zeta_excel = zeros(35,1);

fitted_tank1_pendulum_length_excel  = zeros(35,1); %20250310
delta1_pendulum_length = zeros(35,1); %20250310
rate1_pendulum_length = zeros(35,1); %20250310
fitted_tank2_pendulum_length_excel  = zeros(35,1); %20250310
delta2_pendulum_length = zeros(35,1); %20250310
rate2_pendulum_length = zeros(35,1); %20250310

fitted_tank2_m0_excel   = zeros(35,1);
fitted_tank2_I0zz_excel = zeros(35,1);
fitted_tank2_I0xy_excel = zeros(35,1);
fitted_tank2_I0xz_excel = zeros(35,1);
fitted_tank2_I0yz_excel = zeros(35,1);
fitted_tank2_m_excel    = zeros(35,1);
fitted_tank2_I0xx_excel = zeros(35,1);
fitted_tank2_I0yy_excel = zeros(35,1);
fitted_tank2_L_0_x_excel = zeros(35,1);
fitted_tank2_L_x_excel   = zeros(35,1);
% fitted_tank2_pendulum_length_excel = zeros(35,1);
fitted_tank2_zeta_excel = zeros(35,1);

rate_fitted_tank2_m0_excel   = zeros(35,1);
rate_fitted_tank2_I0zz_excel = zeros(35,1);
rate_fitted_tank2_I0xy_excel = zeros(35,1);
rate_fitted_tank2_I0xz_excel = zeros(35,1);
rate_fitted_tank2_I0yz_excel = zeros(35,1);
rate_fitted_tank2_m_excel    = zeros(35,1);
rate_fitted_tank2_I0xx_excel = zeros(35,1);
rate_fitted_tank2_I0yy_excel = zeros(35,1);
rate_fitted_tank2_L_0_x_excel = zeros(35,1);
rate_fitted_tank2_L_x_excel   = zeros(35,1);
rate_fitted_tank2_pendulum_length_excel = zeros(35,1);
rate_fitted_tank2_zeta_excel = zeros(35,1);

fitted_tank3_m0_excel   = zeros(20,1);
fitted_tank3_I0zz_excel = zeros(20,1);
fitted_tank3_I0xy_excel = zeros(20,1);
fitted_tank3_I0xz_excel = zeros(20,1);
fitted_tank3_I0yz_excel = zeros(20,1);
fitted_tank3_m_excel    = zeros(20,1);
fitted_tank3_I0xx_excel = zeros(20,1);
fitted_tank3_I0yy_excel = zeros(20,1);
fitted_tank3_L_0_x_excel = zeros(20,1);
fitted_tank3_L_x_excel   = zeros(20,1);
fitted_tank3_pendulum_length_excel = zeros(20,1);
fitted_tank3_zeta_excel = zeros(20,1);

rate_fitted_tank3_m0_excel   = zeros(20,1);
rate_fitted_tank3_I0zz_excel = zeros(20,1);
rate_fitted_tank3_I0xy_excel = zeros(20,1);
rate_fitted_tank3_I0xz_excel = zeros(20,1);
rate_fitted_tank3_I0yz_excel = zeros(20,1);
rate_fitted_tank3_m_excel    = zeros(20,1);
rate_fitted_tank3_I0xx_excel = zeros(20,1);
rate_fitted_tank3_I0yy_excel = zeros(20,1);
rate_fitted_tank3_L_0_x_excel = zeros(20,1);
rate_fitted_tank3_L_x_excel   = zeros(20,1);
rate_fitted_tank3_pendulum_length_excel = zeros(20,1);
rate_fitted_tank3_zeta_excel = zeros(20,1);

fitted_tank4_m0_excel   = zeros(20,1);
fitted_tank4_I0zz_excel = zeros(20,1);
fitted_tank4_I0xy_excel = zeros(20,1);
fitted_tank4_I0xz_excel = zeros(20,1);
fitted_tank4_I0yz_excel = zeros(20,1);
fitted_tank4_m_excel    = zeros(20,1);
fitted_tank4_I0xx_excel = zeros(20,1);
fitted_tank4_I0yy_excel = zeros(20,1);
fitted_tank4_L_0_x_excel = zeros(20,1);
fitted_tank4_L_x_excel   = zeros(20,1);
fitted_tank4_pendulum_length_excel = zeros(20,1);
fitted_tank4_zeta_excel = zeros(20,1);

rate_fitted_tank4_m0_excel   = zeros(20,1);
rate_fitted_tank4_I0zz_excel = zeros(20,1);
rate_fitted_tank4_I0xy_excel = zeros(20,1);
rate_fitted_tank4_I0xz_excel = zeros(20,1);
rate_fitted_tank4_I0yz_excel = zeros(20,1);
rate_fitted_tank4_m_excel    = zeros(20,1);
rate_fitted_tank4_I0xx_excel = zeros(20,1);
rate_fitted_tank4_I0yy_excel = zeros(20,1);
rate_fitted_tank4_L_0_x_excel = zeros(20,1);
rate_fitted_tank4_L_x_excel   = zeros(20,1);
rate_fitted_tank4_pendulum_length_excel = zeros(20,1);
rate_fitted_tank4_zeta_excel = zeros(20,1);

eigenvalues_res = cell(1,matrix_size);
mag_res = cell(1,matrix_size);
phase_res = cell(1,matrix_size);
equal_w_res = cell(1,matrix_size);

t_start_list     =     [0  64  128  192  256  320  384  448  512  576];
% t_start_list_index = [1   2    3    4    5    6    7    8    9   10];
t_start_list_index = 1;
K = 0;
f0 = [0; 0; 0;];
t_start = t_start_list(t_start_list_index); % 0  64  128  192  256  320  384  448  512  576
% t_start = 0;
disp(['t_start：', num2str(t_start)]);

%%
for t = t_start:t_interval:t_end
    
    tank_info_group = cell(1,4);
    
    % 读取4个贮箱内推进剂的瞬时值 x1 x2 x3 x4    
    % 设定一二箱推进剂（满足约束范围内）的初始值，以读取1-2贮箱中其它各量的初始值
    x1 = m_T1_init_value + m_T1_4_add_fac_group(1);
    % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
    % 故需要新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。此处是将 m_0_k_4ctr 和 L_0_k 继续
    % 向前递推到推进剂不拉偏时的量。
    x1_4ctr = m_T1_init_value;
    
    x2 = m_T2_init_value + m_T1_4_add_fac_group(2);
    x2_4ctr = m_T2_init_value;

    x3 = (m_T3_init_value - 2 * 80000 * t /(330 * 9.8 * 3)) + m_T1_4_add_fac_group(3); 
    x3_4ctr = (m_T3_init_value - 2 * 80000 * t /(330 * 9.8 * 3)); 
    x4 = (m_T4_init_value - 1 * 80000 * t /(330 * 9.8 * 3)) + m_T1_4_add_fac_group(4); 
    x4_4ctr = (m_T4_init_value - 1 * 80000 * t /(330 * 9.8 * 3)); 
    
    % 推进剂消耗完时停止仿真过程
    if x3 <= 0 || x4 <= 0
        disp('停止循环。');
        disp(['当前时刻t: ', num2str(t)]);
        disp(['当前x3: ', num2str(x3)]);
        disp(['当前x4: ', num2str(x4)]);
        break;
    end
   
    % 读取贮箱1-4的瞬时值：选用拟合函数算出来的值。
    % 这些值既用在了对象的传递函数模型中，也用在了用于控制量计算（即控制器模型）的变量中
    if FITTING_INTERPOLATION == FITTING
        % 根据拟合的函数，实时计算 第1个贮箱 内的各个量
        tank_info_group{1}.mT = x1;
        [tank_info_group{1}.m0] = utils().poly_func_pre(x1, m01_params, m01_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{1}.m0_4ctr] = utils().poly_func_pre(x1_4ctr, m01_params, m01_MU);
        %         [tank_info_group{1}.I0zz] = utils().poly_func_pre(x, I01zz_params, I01zz_MU);
        %         [tank_info_group{1}.I0xy] = utils().poly_func_pre(x, I01xy_params, I01xy_MU);
        %         [tank_info_group{1}.I0xz] = utils().poly_func_pre(x, I01xz_params, I01xz_MU);
        %         [tank_info_group{1}.I0yz] = utils().poly_func_pre(x, I01yz_params, I01yz_MU);
        [tank_info_group{1}.I0zz] = utils().zero_func_pre(x1, I01zz_params, I01zz_MU); %% 最开始拟合时，I01zz已当0处理了
        [tank_info_group{1}.I0xy] = utils().zero_func_pre(x1, I01xy_params, I01xy_MU);
        [tank_info_group{1}.I0xz] = utils().zero_func_pre(x1, I01xz_params, I01xz_MU);
        [tank_info_group{1}.I0yz] = utils().zero_func_pre(x1, I01yz_params, I01yz_MU);
        [tank_info_group{1}.I0zz_4ctr] = utils().zero_func_pre(x1_4ctr, I01zz_params, I01zz_MU);
        [tank_info_group{1}.I0xy_4ctr] = utils().zero_func_pre(x1_4ctr, I01xy_params, I01xy_MU);
        [tank_info_group{1}.I0xz_4ctr] = utils().zero_func_pre(x1_4ctr, I01xz_params, I01xz_MU);
        [tank_info_group{1}.I0yz_4ctr] = utils().zero_func_pre(x1_4ctr, I01yz_params, I01yz_MU);
        
        [tank_info_group{1}.m]    = utils().segm_func_pre(x1,  m1_params1,   m1_params2   , m1_segm_point, "m1") * m_1_4_mul_fac_group(1);
        [tank_info_group{1}.m_4ctr]=utils().segm_func_pre(x1_4ctr,  m1_params1,   m1_params2   , m1_segm_point, "m1");
        [tank_info_group{1}.I0xx] = utils().segm_func_pre(x1, I01xx_params1, I01xx_params2, I01xx_segm_point, "I01xx");
        [tank_info_group{1}.I0xx_4ctr] = utils().segm_func_pre(x1_4ctr, I01xx_params1, I01xx_params2, I01xx_segm_point, "I01xx");
        [tank_info_group{1}.I0yy] = utils().segm_func_pre(x1, I01yy_params1, I01yy_params2, I01yy_segm_point, "I01yy");
        [tank_info_group{1}.I0yy_4ctr] = utils().segm_func_pre(x1_4ctr, I01yy_params1, I01yy_params2, I01yy_segm_point, "I01yy");
        
        % L_0_x L_x 做了预处理，仅影响拟合过程，不影响变量读数。
        [tank_info_group{1}.L_0_x] = utils().poly_func_pre(x1, L01x_params, L01x_MU)+4.745;
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{1}.L_0_x_4ctr] = utils().poly_func_pre(x1_4ctr, L01x_params, L01x_MU)+4.745;
        [tank_info_group{1}.L_x] = utils().poly_func_pre(x1, L1x_params, L1x_MU)+4.745 + L12_rand_val(1);
        [tank_info_group{1}.L_x_4ctr] = utils().poly_func_pre(x1_4ctr, L1x_params, L1x_MU)+4.745;
        
        % ckesa1 未做预处理，直接读数就可以。
        if PENDULUM_ROBUST_KEY == LEN
            if PENDULUM_1 == FITTING
                if FITTING_ERROR == 0.2
                    [tank_info_group{1}.pendulum_length] = utils().poly_func_pre(x1, length1_params, length1_MU) + pendulum_length_rob_set(1);%20250310弃用
                      % [tank_info_group{1}.pendulum_length] = utils().segm_func_pre(x1, length1_params1, length1_params2, length1_segm_point, "length1") + pendulum_length_rob_set(1);
                else % FITTING_ERROR == 0.07
                    [tank_info_group{1}.pendulum_length] = utils().pendulum_1_func_pre(x1, length1_params, length1_MU)+ pendulum_length_rob_set(1);
                end
            else % PENDULUM_1 == INTERPOLATION
                    [tank_info_group{1}.pendulum_length] = utils().pendulum_1_func_pre2(x1, length1_xi, length1_yi)+ pendulum_length_rob_set(1);  %20250312启用
            end
        else % PENDULUM_ROBUST_KEY == PRO
            if PENDULUM_1 == FITTING
                if FITTING_ERROR == 0.2
                    [tank_info_group{1}.pendulum_length] = utils().poly_func_pre(x1, length1_params, length1_MU) * pendulum_length_rob_set(1);%20250310弃用
            %         [tank_info_group{1}.pendulum_length] = utils().segm_func_pre(x1, length1_params1, length1_params2, length1_segm_point, "length1")* pendulum_length_rob_set(1);
                else % FITTING_ERROR == 0.07
                    [tank_info_group{1}.pendulum_length] = utils().pendulum_1_func_pre(x1, length1_params, length1_MU)* pendulum_length_rob_set(1);
                end
            else % PENDULUM_1 == INTERPOLATION
                [tank_info_group{1}.pendulum_length] = utils().pendulum_1_func_pre2(x1, length1_xi, length1_yi)* pendulum_length_rob_set(1); %20250312启用
            end
        end
        [tank_info_group{1}.zeta] = utils().poly_func_pre(x1, ckesa1_params, ckesa1_MU);
        
        % 不用于控制量和动力学模型，仅用于计算推进剂质量拉偏时，摆长和阻尼的拉偏量--20250118
        if PENDULUM_1 == FITTING
             if FITTING_ERROR == 0.2
                [tank_info_group{1}.pendulum_length_rbst] = utils().poly_func_pre(x1_4ctr, length1_params, length1_MU);%20250310弃用
        %     [tank_info_group{1}.pendulum_length_rbst] = utils().segm_func_pre(x1_4ctr, length1_params1, length1_params2, length1_segm_point, "length1");
             else % FITTING_ERROR == 0.07
                [tank_info_group{1}.pendulum_length_rbst] = utils().pendulum_1_func_pre(x1_4ctr, length1_params, length1_MU);
             end
        else
            [tank_info_group{1}.pendulum_length_rbst] = utils().pendulum_1_func_pre2(x1_4ctr,length1_xi, length1_yi); %20250312启用
        end
        [tank_info_group{1}.zeta_rbst] = utils().poly_func_pre(x1_4ctr, ckesa1_params, ckesa1_MU);
        
        % 根据拟合的函数，实时计算 第2个贮箱 内的各个量
        tank_info_group{2}.mT = x2;
        [tank_info_group{2}.m0]   = utils().poly_func_pre(x2, m02_params, m02_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{2}.m0_4ctr]   = utils().poly_func_pre(x2_4ctr, m02_params, m02_MU);
        %         [tank_info_group{2}.I0zz] = utils().poly_func_pre(x2, I02zz_params, I02zz_MU);
        %         [tank_info_group{2}.I0xy] = utils().poly_func_pre(x2, I02xy_params, I02xy_MU);
        %         [tank_info_group{2}.I0xz] = utils().poly_func_pre(x2, I02xz_params, I02xz_MU);
        %         [tank_info_group{2}.I0yz] = utils().poly_func_pre(x2, I02yz_params, I02yz_MU);
        [tank_info_group{2}.I0zz] = utils().zero_func_pre(x2, I02zz_params, I02zz_MU);
        [tank_info_group{2}.I0xy] = utils().zero_func_pre(x2, I02xy_params, I02xy_MU);
        [tank_info_group{2}.I0xz] = utils().zero_func_pre(x2, I02xz_params, I02xz_MU);
        [tank_info_group{2}.I0yz] = utils().zero_func_pre(x2, I02yz_params, I02yz_MU);
        [tank_info_group{2}.I0zz_4ctr] = utils().zero_func_pre(x2_4ctr, I02zz_params, I02zz_MU);
        [tank_info_group{2}.I0xy_4ctr] = utils().zero_func_pre(x2_4ctr, I02xy_params, I02xy_MU);
        [tank_info_group{2}.I0xz_4ctr] = utils().zero_func_pre(x2_4ctr, I02xz_params, I02xz_MU);
        [tank_info_group{2}.I0yz_4ctr] = utils().zero_func_pre(x2_4ctr, I02yz_params, I02yz_MU);
        
        [tank_info_group{2}.m]    = utils().poly_func_pre(x2, m2_params, m2_MU) * m_1_4_mul_fac_group(2);
        [tank_info_group{2}.m_4ctr]    = utils().poly_func_pre(x2_4ctr, m2_params, m2_MU);
        [tank_info_group{2}.I0xx] = utils().segm_func_pre(x2, I02xx_params1, I02xx_params2, I02xx_segm_point, "I02xx");
        [tank_info_group{2}.I0yy] = utils().segm_func_pre(x2, I02yy_params1, I02yy_params2, I02yy_segm_point, "I02yy");
        [tank_info_group{2}.I0xx_4ctr] = utils().segm_func_pre(x2_4ctr, I02xx_params1, I02xx_params2, I02xx_segm_point, "I02xx");
        [tank_info_group{2}.I0yy_4ctr] = utils().segm_func_pre(x2_4ctr, I02yy_params1, I02yy_params2, I02yy_segm_point, "I02yy");
        [tank_info_group{2}.L_0_x]  = utils().poly_func_pre(x2, L02x_params, L02x_MU)+4.745;
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{2}.L_0_x_4ctr]  = utils().poly_func_pre(x2_4ctr, L02x_params, L02x_MU)+4.745;
        [tank_info_group{2}.L_x]  = utils().poly_func_pre(x2, L2x_params, L2x_MU)+4.745 + L12_rand_val(2);
        [tank_info_group{2}.L_x_4ctr]  = utils().poly_func_pre(x2_4ctr, L2x_params, L2x_MU)+4.745;
        if PENDULUM_ROBUST_KEY == LEN
            [tank_info_group{2}.pendulum_length] = utils().poly_func_pre(x2, length2_params, length2_MU) + pendulum_length_rob_set(2);
        else
            [tank_info_group{2}.pendulum_length] = utils().poly_func_pre(x2, length2_params, length2_MU) * pendulum_length_rob_set(2);
        end
        [tank_info_group{2}.zeta] = utils().poly_func_pre(x2, ckesa2_params, ckesa2_MU);
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{2}.pendulum_length_rbst] = utils().poly_func_pre(x2_4ctr, length2_params, length2_MU);
        [tank_info_group{2}.zeta_rbst] = utils().poly_func_pre(x2_4ctr, ckesa2_params, ckesa2_MU);
        
        % 根据拟合的函数，实时计算 第3个贮箱 内的各个量
        tank_info_group{3}.mT = x3;
        [tank_info_group{3}.m0]   = utils().poly_func_pre(x3, m03_params, m03_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{3}.m0_4ctr]   = utils().poly_func_pre(x3_4ctr, m03_params, m03_MU);
        %     [tank_info_group{3}.I0zz] = utils().poly_func_pre(x3, I03zz_params, I03zz_MU);
        %     [tank_info_group{3}.I0xy] = utils().poly_func_pre(x3, I03xy_params, I03xy_MU);
        %     [tank_info_group{3}.I0xz] = utils().poly_func_pre(x3, I03xz_params, I03xz_MU);
        %     [tank_info_group{3}.I0yz] = utils().poly_func_pre(x3, I03yz_params, I03yz_MU);
        [tank_info_group{3}.I0zz] = utils().zero_func_pre(x3, I03zz_params, I03zz_MU);
        [tank_info_group{3}.I0xy] = utils().zero_func_pre(x3, I03xy_params, I03xy_MU);
        [tank_info_group{3}.I0xz] = utils().zero_func_pre(x3, I03xz_params, I03xz_MU);
        [tank_info_group{3}.I0yz] = utils().zero_func_pre(x3, I03yz_params, I03yz_MU);
        [tank_info_group{3}.I0zz_4ctr] = utils().zero_func_pre(x3_4ctr, I03zz_params, I03zz_MU);
        [tank_info_group{3}.I0xy_4ctr] = utils().zero_func_pre(x3_4ctr, I03xy_params, I03xy_MU);
        [tank_info_group{3}.I0xz_4ctr] = utils().zero_func_pre(x3_4ctr, I03xz_params, I03xz_MU);
        [tank_info_group{3}.I0yz_4ctr] = utils().zero_func_pre(x3_4ctr, I03yz_params, I03yz_MU);
        
        [tank_info_group{3}.m]    = utils().poly_func_pre(x3, m3_params, m3_MU) * m_1_4_mul_fac_group(3);
        [tank_info_group{3}.m_4ctr]    = utils().poly_func_pre(x3_4ctr, m3_params, m3_MU);
        %     [tank_info_group{3}.I0xx] = utils().segm_func_pre(x3, I03xx_params1, I03xx_params2, I03xx_segm_point, "I03xx");
        %     [tank_info_group{3}.I0yy] = utils().segm_func_pre(x3, I03yy_params1, I03yy_params2, I03yy_segm_point, "I03yy");
        [tank_info_group{3}.I0xx] = utils().segm_func_pre(x3, I03xx_params1, I03xx_params2, I03xx_segm_point,"I03xx");
        [tank_info_group{3}.I0yy] = utils().segm_func_pre(x3, I03yy_params1, I03yy_params2, I03yy_segm_point,"I03yy");
        [tank_info_group{3}.I0xx_4ctr] = utils().segm_func_pre(x3_4ctr, I03xx_params1, I03xx_params2, I03xx_segm_point,"I03xx");
        [tank_info_group{3}.I0yy_4ctr] = utils().segm_func_pre(x3_4ctr, I03yy_params1, I03yy_params2, I03yy_segm_point,"I03yy");
        [tank_info_group{3}.L_0_x]  = utils().poly_func_pre(x3, L03x_params, L03x_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k_4ctr。
        [tank_info_group{3}.L_0_x_4ctr]  = utils().poly_func_pre(x3_4ctr, L03x_params, L03x_MU);
        [tank_info_group{3}.L_x]  = utils().poly_func_pre(x3, L3x_params, L3x_MU) + L34_rand_val(1);
        [tank_info_group{3}.L_x_4ctr]  = utils().poly_func_pre(x3_4ctr, L3x_params, L3x_MU);
        if PENDULUM_ROBUST_KEY == LEN
            [tank_info_group{3}.pendulum_length] = utils().poly_func_pre(x3, length3_params, length3_MU) + pendulum_length_rob_set(3);
        else
            [tank_info_group{3}.pendulum_length] = utils().poly_func_pre(x3, length3_params, length3_MU) * pendulum_length_rob_set(3);
        end
        [tank_info_group{3}.zeta] = utils().poly_func_pre(x3, ckesa3_params, ckesa3_MU);
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{3}.pendulum_length_rbst] = utils().poly_func_pre(x3_4ctr, length3_params, length3_MU);
        [tank_info_group{3}.zeta_rbst] = utils().poly_func_pre(x3_4ctr, ckesa3_params, ckesa3_MU);
        
        % 根据拟合的函数，实时计算 第4个贮箱 内的各个量
        tank_info_group{4}.mT = x4;
        [tank_info_group{4}.m0]   = utils().poly_func_pre(x4, m04_params, m04_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k_4ctr。
        [tank_info_group{4}.m0_4ctr]   = utils().poly_func_pre(x4_4ctr, m04_params, m04_MU); %20250108
        %     [tank_info_group{4}.I0zz] = utils().poly_func_pre(x4, I04zz_params, I04zz_MU);
        %     [tank_info_group{4}.I0xy] = utils().poly_func_pre(x4, I04xy_params, I04xy_MU);
        %     [tank_info_group{4}.I0xz] = utils().poly_func_pre(x4, I04xz_params, I04xz_MU);
        %     [tank_info_group{4}.I0yz] = utils().poly_func_pre(x4, I04yz_params, I04yz_MU);
        [tank_info_group{4}.I0zz] = utils().zero_func_pre(x4, I04zz_params, I04zz_MU);
        [tank_info_group{4}.I0xy] = utils().zero_func_pre(x4, I04xy_params, I04xy_MU);
        [tank_info_group{4}.I0xz] = utils().zero_func_pre(x4, I04xz_params, I04xz_MU);
        [tank_info_group{4}.I0yz] = utils().zero_func_pre(x4, I04yz_params, I04yz_MU);
        [tank_info_group{4}.I0zz_4ctr] = utils().zero_func_pre(x4_4ctr, I04zz_params, I04zz_MU);
        [tank_info_group{4}.I0xy_4ctr] = utils().zero_func_pre(x4_4ctr, I04xy_params, I04xy_MU);
        [tank_info_group{4}.I0xz_4ctr] = utils().zero_func_pre(x4_4ctr, I04xz_params, I04xz_MU);
        [tank_info_group{4}.I0yz_4ctr] = utils().zero_func_pre(x4_4ctr, I04yz_params, I04yz_MU);
        
        [tank_info_group{4}.m]    = utils().poly_func_pre(x4, m4_params, m4_MU) * m_1_4_mul_fac_group(4);
        [tank_info_group{4}.m_4ctr]    = utils().poly_func_pre(x4_4ctr, m4_params, m4_MU);
        %     [tank_info_group{4}.I0xx] = utils().poly_func_pre(x4, I04xx_params, I04xx_MU);
        %     [tank_info_group{4}.I0yy] = utils().poly_func_pre(x4, I04yy_params, I04yy_MU);
        [tank_info_group{4}.I0xx] = utils().segm_func_pre(x4, I04xx_params1, I04xx_params2, I04xx_segm_point, "I04xx");
        [tank_info_group{4}.I0yy] = utils().segm_func_pre(x4, I04yy_params1, I04yy_params2, I04yy_segm_point, "I04yy");
        [tank_info_group{4}.I0xx_4ctr] = utils().segm_func_pre(x4_4ctr, I04xx_params1, I04xx_params2, I04xx_segm_point, "I04xx");
        [tank_info_group{4}.I0yy_4ctr] = utils().segm_func_pre(x4_4ctr, I04yy_params1, I04yy_params2, I04yy_segm_point, "I04yy");
        [tank_info_group{4}.L_0_x]  = utils().poly_func_pre(x4, L04x_params, L04x_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k_4ctr。
        [tank_info_group{4}.L_0_x_4ctr]  = utils().poly_func_pre(x4_4ctr, L04x_params, L04x_MU);
        [tank_info_group{4}.L_x]  = utils().poly_func_pre(x4, L4x_params, L4x_MU) + L34_rand_val(2);
        [tank_info_group{4}.L_x_4ctr]  = utils().poly_func_pre(x4_4ctr, L4x_params, L4x_MU);
        if PENDULUM_ROBUST_KEY == LEN
            [tank_info_group{4}.pendulum_length] = utils().poly_func_pre(x4, length4_params, length4_MU) + pendulum_length_rob_set(4);
        else
            [tank_info_group{4}.pendulum_length] = utils().poly_func_pre(x4, length4_params, length4_MU) * pendulum_length_rob_set(4);
        end
        [tank_info_group{4}.zeta] = utils().poly_func_pre(x4, ckesa4_params, ckesa4_MU);
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{4}.pendulum_length_rbst] = utils().poly_func_pre(x4_4ctr, length4_params, length4_MU);
        [tank_info_group{4}.zeta_rbst] = utils().poly_func_pre(x4_4ctr, ckesa4_params, ckesa4_MU);

    end 
    
    % 读取贮箱1-4的瞬时值：既有拟合函数算出来的值，又有基于EXCEL表插值的数据；其中，
    % 基于EXCEL表插值的数据用在了对象的传递函数模型中，拟合函数算出来的值用在了用于控制量计算（即控制器模型）的变量中
    if FITTING_INTERPOLATION == INTERPOLATION       
        % Reading current value in tank_1 -- used for dynamic model
        tank_info_group{1}.mT = x1;
        tank_info_group{1}.m0 = interp1(mT_info{1}, predealed_m0_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.m = interp1(mT_info{1}, predealed_m_info{1}, x1, 'linear','extrap') * m_1_4_mul_fac_group(1);
        tank_info_group{1}.L_0_x = interp1(mT_info{1}, predealed_L_0_x_info{1}, x1, 'linear','extrap')+4.745;
        tank_info_group{1}.L_x = interp1(mT_info{1}, predealed_L_x_info{1}, x1, 'linear','extrap')+4.745 + L12_rand_val(1);
        tank_info_group{1}.I0xx = interp1(mT_info{1}, predealed_I0xx_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.I0yy = interp1(mT_info{1}, predealed_I0yy_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.I0zz = interp1(mT_info{1}, predealed_I0zz_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.I0xy = interp1(mT_info{1}, predealed_I0xy_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.I0xz = interp1(mT_info{1}, predealed_I0xz_info{1}, x1, 'linear','extrap');
        tank_info_group{1}.I0yz = interp1(mT_info{1}, predealed_I0yz_info{1}, x1, 'linear','extrap');
        % ckesa1 未做预处理，直接读数就可以。
        if PENDULUM_ROBUST_KEY == LEN
            tank_info_group{1}.pendulum_length = interp1(mT_info{1}, predealed_pendulum_length_info{1}, x1, 'linear','extrap') + pendulum_length_rob_set(1);  %20250312启用
        else
            tank_info_group{1}.pendulum_length = interp1(mT_info{1}, predealed_pendulum_length_info{1}, x1, 'linear','extrap') * pendulum_length_rob_set(1); %20250312启用
        end
        tank_info_group{1}.zeta = interp1(mT_info{1}, predealed_zeta_ratio_info{1}, x1, 'linear','extrap');

        % Reading current value in tank_1 -- used for ctrl
        % tank_info1 used by B0, mainly for ctrl
        [tank_info_group{1}.m0_4ctr] = utils().poly_func_pre(x1_4ctr, m01_params, m01_MU);
        [tank_info_group{1}.m_4ctr]=utils().segm_func_pre(x1_4ctr,  m1_params1,   m1_params2   , m1_segm_point, "m1");
        [tank_info_group{1}.I0xx_4ctr] = utils().segm_func_pre(x1_4ctr, I01xx_params1, I01xx_params2, I01xx_segm_point, "I01xx");
        [tank_info_group{1}.I0yy_4ctr] = utils().segm_func_pre(x1_4ctr, I01yy_params1, I01yy_params2, I01yy_segm_point, "I01yy");
        [tank_info_group{1}.I0zz_4ctr] = utils().zero_func_pre(x1_4ctr, I01zz_params, I01zz_MU);
        [tank_info_group{1}.I0xy_4ctr] = utils().zero_func_pre(x1_4ctr, I01xy_params, I01xy_MU);
        [tank_info_group{1}.I0xz_4ctr] = utils().zero_func_pre(x1_4ctr, I01xz_params, I01xz_MU);
        [tank_info_group{1}.I0yz_4ctr] = utils().zero_func_pre(x1_4ctr, I01yz_params, I01yz_MU);   
        [tank_info_group{1}.L_0_x_4ctr] = utils().poly_func_pre(x1_4ctr, L01x_params, L01x_MU)+4.745;
        [tank_info_group{1}.L_x_4ctr] = utils().poly_func_pre(x1_4ctr, L1x_params, L1x_MU)+4.745;
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{1}.pendulum_length_rbst] = interp1(mT_info{1}, predealed_pendulum_length_info{1}, x1_4ctr, 'linear','extrap'); %20250312启用
        [tank_info_group{1}.zeta_rbst] = interp1(mT_info{1}, predealed_zeta_ratio_info{1}, x1_4ctr, 'linear','extrap');
        
        % reading current value in tank_2--used for dynamic model
        tank_info_group{2}.mT = x2;
        tank_info_group{2}.m0 = interp1(mT_info{2}, predealed_m0_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.m = interp1(mT_info{2}, predealed_m_info{2}, x2, 'linear','extrap') * m_1_4_mul_fac_group(2);
        tank_info_group{2}.L_0_x = interp1(mT_info{2}, predealed_L_0_x_info{2}, x2, 'linear','extrap')+4.745;
        tank_info_group{2}.L_x = interp1(mT_info{2}, predealed_L_x_info{2}, x2, 'linear','extrap')+4.745+ L12_rand_val(2);
        tank_info_group{2}.I0xx = interp1(mT_info{2}, predealed_I0xx_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.I0yy = interp1(mT_info{2}, predealed_I0yy_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.I0zz = interp1(mT_info{2}, predealed_I0zz_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.I0xy = interp1(mT_info{2}, predealed_I0xy_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.I0xz = interp1(mT_info{2}, predealed_I0xz_info{2}, x2, 'linear','extrap');
        tank_info_group{2}.I0yz = interp1(mT_info{2}, predealed_I0yz_info{2}, x2, 'linear','extrap');
%         tank_info_group{2}.pendulum_length = interp1(mT_info{2}, predealed_pendulum_length_info{2}, x2, 'linear','extrap');
        if PENDULUM_ROBUST_KEY == LEN
            tank_info_group{2}.pendulum_length = interp1(mT_info{2}, predealed_pendulum_length_info{2}, x2, 'linear','extrap') + pendulum_length_rob_set(2);
        else
            tank_info_group{2}.pendulum_length = interp1(mT_info{2}, predealed_pendulum_length_info{2}, x2, 'linear','extrap') * pendulum_length_rob_set(2);
        end
        tank_info_group{2}.zeta = interp1(mT_info{2}, predealed_zeta_ratio_info{2}, x2, 'linear','extrap');

        % reading current value in tank_2--used for ctrl
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{2}.m0_4ctr]   = utils().poly_func_pre(x2_4ctr, m02_params, m02_MU);
        [tank_info_group{2}.m_4ctr]    = utils().poly_func_pre(x2_4ctr, m2_params, m2_MU);
        [tank_info_group{2}.I0xx_4ctr] = utils().segm_func_pre(x2_4ctr, I02xx_params1, I02xx_params2, I02xx_segm_point, "I02xx");
        [tank_info_group{2}.I0yy_4ctr] = utils().segm_func_pre(x2_4ctr, I02yy_params1, I02yy_params2, I02yy_segm_point, "I02yy");
        [tank_info_group{2}.I0zz_4ctr] = utils().zero_func_pre(x2_4ctr, I02zz_params, I02zz_MU);
        [tank_info_group{2}.I0xy_4ctr] = utils().zero_func_pre(x2_4ctr, I02xy_params, I02xy_MU);
        [tank_info_group{2}.I0xz_4ctr] = utils().zero_func_pre(x2_4ctr, I02xz_params, I02xz_MU);
        [tank_info_group{2}.I0yz_4ctr] = utils().zero_func_pre(x2_4ctr, I02yz_params, I02yz_MU);
        % 20250108新增注释：推进剂质量拉偏时，m_0_k，L_0_k会发生变化。这两个量用在了控制量U中；而用于控制的量不应随推进剂拉偏发生变化，
        % 故新增不受推进剂拉偏影响的变量 m_0_k_4ctr,L_0_k。
        [tank_info_group{2}.L_0_x_4ctr]  = utils().poly_func_pre(x2_4ctr, L02x_params, L02x_MU)+4.745;
        [tank_info_group{2}.L_x_4ctr]  = utils().poly_func_pre(x2_4ctr, L2x_params, L2x_MU)+4.745;   
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{2}.pendulum_length_rbst] = interp1(mT_info{2}, predealed_pendulum_length_info{2}, x2_4ctr, 'linear','extrap');
        [tank_info_group{2}.zeta_rbst] = interp1(mT_info{2}, predealed_zeta_ratio_info{2}, x2_4ctr, 'linear','extrap');
        

        % reading current value in tank_3--used for dynamic model
        tank_info_group{3}.mT = x3;
        tank_info_group{3}.m0 = interp1(mT_info{3}, predealed_m0_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.m = interp1(mT_info{3}, predealed_m_info{3}, x3, 'linear','extrap') * m_1_4_mul_fac_group(3);
        tank_info_group{3}.L_0_x = interp1(mT_info{3}, predealed_L_0_x_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.L_x = interp1(mT_info{3}, predealed_L_x_info{3}, x3, 'linear','extrap') + L34_rand_val(1);
        tank_info_group{3}.I0xx = interp1(mT_info{3}, predealed_I0xx_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.I0yy = interp1(mT_info{3}, predealed_I0yy_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.I0zz = interp1(mT_info{3}, predealed_I0zz_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.I0xy = interp1(mT_info{3}, predealed_I0xy_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.I0xz = interp1(mT_info{3}, predealed_I0xz_info{3}, x3, 'linear','extrap');
        tank_info_group{3}.I0yz = interp1(mT_info{3}, predealed_I0yz_info{3}, x3, 'linear','extrap');
%         tank_info_group{3}.pendulum_length = interp1(mT_info{3}, predealed_pendulum_length_info{3}, x3, 'linear','extrap');
        if PENDULUM_ROBUST_KEY == LEN
            tank_info_group{3}.pendulum_length = interp1(mT_info{3}, predealed_pendulum_length_info{3}, x3, 'linear','extrap') + pendulum_length_rob_set(3);
        else
            tank_info_group{3}.pendulum_length = interp1(mT_info{3}, predealed_pendulum_length_info{3}, x3, 'linear','extrap') * pendulum_length_rob_set(3);
        end
        tank_info_group{3}.zeta = interp1(mT_info{3}, predealed_zeta_ratio_info{3}, x3, 'linear','extrap');

        % reading current value in tank_3--used for ctrl
        [tank_info_group{3}.m0_4ctr]   = utils().poly_func_pre(x3_4ctr, m03_params, m03_MU);
        [tank_info_group{3}.m_4ctr]    = utils().poly_func_pre(x3_4ctr, m3_params, m3_MU);
        [tank_info_group{3}.I0xx_4ctr] = utils().segm_func_pre(x3_4ctr, I03xx_params1, I03xx_params2, I03xx_segm_point,"I03xx");
        [tank_info_group{3}.I0yy_4ctr] = utils().segm_func_pre(x3_4ctr, I03yy_params1, I03yy_params2, I03yy_segm_point,"I03yy");
        [tank_info_group{3}.I0zz_4ctr] = utils().zero_func_pre(x3_4ctr, I03zz_params, I03zz_MU);
        [tank_info_group{3}.I0xy_4ctr] = utils().zero_func_pre(x3_4ctr, I03xy_params, I03xy_MU);
        [tank_info_group{3}.I0xz_4ctr] = utils().zero_func_pre(x3_4ctr, I03xz_params, I03xz_MU);
        [tank_info_group{3}.I0yz_4ctr] = utils().zero_func_pre(x3_4ctr, I03yz_params, I03yz_MU);
        [tank_info_group{3}.L_0_x_4ctr]  = utils().poly_func_pre(x3_4ctr, L03x_params, L03x_MU);
        [tank_info_group{3}.L_x_4ctr]  = utils().poly_func_pre(x3_4ctr, L3x_params, L3x_MU);
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{3}.pendulum_length_rbst] = interp1(mT_info{3}, predealed_pendulum_length_info{3}, x3_4ctr, 'linear','extrap');
        [tank_info_group{3}.zeta_rbst] = interp1(mT_info{3}, predealed_zeta_ratio_info{3}, x3_4ctr, 'linear','extrap');

        % reading current value in tank_4--used for dynamic model
        tank_info_group{4}.mT = x4;
        tank_info_group{4}.m0 = interp1(mT_info{4}, predealed_m0_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.m  = interp1(mT_info{4}, predealed_m_info{4}, x4, 'linear','extrap') * m_1_4_mul_fac_group(4);
        tank_info_group{4}.L_0_x  = interp1(mT_info{4}, predealed_L_0_x_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.L_x  = interp1(mT_info{4}, predealed_L_x_info{4}, x4, 'linear','extrap') + L34_rand_val(2);
        tank_info_group{4}.I0xx  = interp1(mT_info{4}, predealed_I0xx_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.I0yy  = interp1(mT_info{4}, predealed_I0yy_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.I0zz  = interp1(mT_info{4}, predealed_I0zz_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.I0xy  = interp1(mT_info{4}, predealed_I0xy_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.I0xz  = interp1(mT_info{4}, predealed_I0xz_info{4}, x4, 'linear','extrap');
        tank_info_group{4}.I0yz  = interp1(mT_info{4}, predealed_I0yz_info{4}, x4, 'linear','extrap');
%         tank_info_group{4}.pendulum_length  = interp1(mT_info{4}, predealed_pendulum_length_info{4}, x4, 'linear','extrap');
        if PENDULUM_ROBUST_KEY == LEN
            [tank_info_group{4}.pendulum_length] = interp1(mT_info{4}, predealed_pendulum_length_info{4}, x4, 'linear','extrap') + pendulum_length_rob_set(4);
        else
            [tank_info_group{4}.pendulum_length] = interp1(mT_info{4}, predealed_pendulum_length_info{4}, x4, 'linear','extrap') * pendulum_length_rob_set(4);
        end
        tank_info_group{4}.zeta  = interp1(mT_info{4}, predealed_zeta_ratio_info{4}, x4, 'linear','extrap');

        % reading current value in tank_4--used for ctrl        
        [tank_info_group{4}.m0_4ctr]   = utils().poly_func_pre(x4_4ctr, m04_params, m04_MU); %20250108
        [tank_info_group{4}.m_4ctr]    = utils().poly_func_pre(x4_4ctr, m4_params, m4_MU);
        [tank_info_group{4}.I0xx_4ctr] = utils().segm_func_pre(x4_4ctr, I04xx_params1, I04xx_params2, I04xx_segm_point, "I04xx");
        [tank_info_group{4}.I0yy_4ctr] = utils().segm_func_pre(x4_4ctr, I04yy_params1, I04yy_params2, I04yy_segm_point, "I04yy");
        [tank_info_group{4}.I0zz_4ctr] = utils().zero_func_pre(x4_4ctr, I04zz_params, I04zz_MU);
        [tank_info_group{4}.I0xy_4ctr] = utils().zero_func_pre(x4_4ctr, I04xy_params, I04xy_MU);
        [tank_info_group{4}.I0xz_4ctr] = utils().zero_func_pre(x4_4ctr, I04xz_params, I04xz_MU);
        [tank_info_group{4}.I0yz_4ctr] = utils().zero_func_pre(x4_4ctr, I04yz_params, I04yz_MU);
        [tank_info_group{4}.L_0_x_4ctr]  = utils().poly_func_pre(x4_4ctr, L04x_params, L04x_MU);
        [tank_info_group{4}.L_x_4ctr]  = utils().poly_func_pre(x4_4ctr, L4x_params, L4x_MU);
        % 不用于控制，仅用于计算推进剂质量拉偏时，摆长的拉偏量--20250118
        [tank_info_group{4}.pendulum_length_rbst] = interp1(mT_info{4}, predealed_pendulum_length_info{4}, x4_4ctr, 'linear','extrap');
        [tank_info_group{4}.zeta_rbst] = interp1(mT_info{4}, predealed_zeta_ratio_info{4}, x4_4ctr, 'linear','extrap');        
    end
    
    % 存储每个时刻第1个贮箱中各个变量的值
    %     K = floor(t/t_interval)+1;
    K = K + 1;
    mT1_res(1, K) = tank_info_group{1}.mT;
    m01_res(1, K) = tank_info_group{1}.m0;
    m1_res(1, K) = tank_info_group{1}.m;
    L_01_res(1, K) = tank_info_group{1}.L_0_x;
    L_1_res(1, K) = tank_info_group{1}.L_x;
    pendulum_length1_res(1, K) = tank_info_group{1}.pendulum_length;
    I01xx_res(1, K) = tank_info_group{1}.I0xx;
    I01yy_res(1, K) = tank_info_group{1}.I0yy;
    I01zz_res(1, K) = tank_info_group{1}.I0zz;
    I01xy_res(1, K) = tank_info_group{1}.I0xy;
    I01xz_res(1, K) = tank_info_group{1}.I0xz;
    I01yz_res(1, K) = tank_info_group{1}.I0yz;
    zeta1_ratio_res(1, K) = tank_info_group{1}.zeta;
    
    % 存储每个时刻第2个贮箱中各个变量的值
    mT2_res(1, K) = tank_info_group{2}.mT;
    m02_res(1, K) = tank_info_group{2}.m0;
    m2_res(1, K) = tank_info_group{2}.m;
    L_02_res(1, K) = tank_info_group{2}.L_0_x;
    L_2_res(1, K) = tank_info_group{2}.L_x;
    pendulum_length2_res(1, K) = tank_info_group{2}.pendulum_length;
    I02xx_res(1, K) = tank_info_group{2}.I0xx;
    I02yy_res(1, K) = tank_info_group{2}.I0yy;
    I02zz_res(1, K) = tank_info_group{2}.I0zz;
    I02xy_res(1, K) = tank_info_group{2}.I0xy;
    I02xz_res(1, K) = tank_info_group{2}.I0xz;
    I02yz_res(1, K) = tank_info_group{2}.I0yz;
    zeta2_ratio_res(1, K) = tank_info_group{2}.zeta;
    
    % 存储每个时刻第3个贮箱中各个变量的值
    mT3_res(1, K) = tank_info_group{3}.mT;
    m03_res(1, K) = tank_info_group{3}.m0;
    m3_res(1, K) = tank_info_group{3}.m;
    L_03_res(1, K) = tank_info_group{3}.L_0_x;
    L_3_res(1, K) = tank_info_group{3}.L_x;
    pendulum_length3_res(1, K) = tank_info_group{3}.pendulum_length;
    I03xx_res(1, K) = tank_info_group{3}.I0xx;
    I03yy_res(1, K) = tank_info_group{3}.I0yy;
    I03zz_res(1, K) = tank_info_group{3}.I0zz;
    I03xy_res(1, K) = tank_info_group{3}.I0xy;
    I03xz_res(1, K) = tank_info_group{3}.I0xz;
    I03yz_res(1, K) = tank_info_group{3}.I0yz;
    zeta3_ratio_res(1, K) = tank_info_group{3}.zeta;
    
    % 存储每个时刻第4个贮箱中各个变量的值
    mT4_res(1, K) = tank_info_group{4}.mT;
    m04_res(1, K) = tank_info_group{4}.m0;
    m4_res(1, K) = tank_info_group{4}.m;
    L_04_res(1, K) = tank_info_group{4}.L_0_x;
    L_4_res(1, K) = tank_info_group{4}.L_x;
    pendulum_length4_res(1, K) = tank_info_group{4}.pendulum_length;
    I04xx_res(1, K) = tank_info_group{4}.I0xx;
    I04yy_res(1, K) = tank_info_group{4}.I0yy;
    I04zz_res(1, K) = tank_info_group{4}.I0zz;
    I04xy_res(1, K) = tank_info_group{4}.I0xy;
    I04xz_res(1, K) = tank_info_group{4}.I0xz;
    I04yz_res(1, K) = tank_info_group{4}.I0yz;
    zeta4_ratio_res(1, K) = tank_info_group{4}.zeta;
    
    tank_info_group{1};
    tank_info_group{2};
    tank_info_group{3};
    tank_info_group{4};
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if CHECK_FITTING_RES == ON
        % 由于第1个贮箱和第2个贮箱 不是动态消耗的，很难直接看到拟合效果，
        % 故在此 设定输入变化区间，通过在每个步长内取一个输入值，根据该输入值计算出与贮箱1和2相关的各个量的值，
        % 运行完后就可以画出拟合曲线了。
        % 而贮箱3和贮箱4 一直动态消耗，运行完就可以看到拟合效果，无需再次设定输入变化区间。
        
        %     fitted_T1_T = linspace(min(mT_info{1}), max(mT_info{1}), matrix_size);
        fitted_T1_T = linspace(0, max(mT_info{1}), matrix_size);
        
        %     fitted_T2_T = linspace(min(mT_info{2}), max(mT_info{2}), matrix_size);
        fitted_T2_T = linspace(0, max(mT_info{2}), matrix_size);
        
        % 贮箱3和贮箱4 虽然一直动态消耗，运行完就可以看到拟合效果，此处设置仅为了一次性全部运行完看结果，不跑整个大循环
        %     fitted_T3_T = linspace(min(mT_info{3}), max(mT_info{3}), matrix_size);
        fitted_T3_T = linspace(0, max(mT_info{3}), matrix_size);
        
        %     fitted_T4_T = linspace(min(mT_info{4}), max(mT_info{4}), matrix_size);
        fitted_T4_T = linspace(0, max(mT_info{4}), matrix_size);
        
        for K = 1:matrix_size %20250414改为单独执行，不随大循环运行,大循环运行时，需要屏蔽 for end 这两行
            % 根据拟合的函数，给出 第1个贮箱 内的各个量的拟合值
            fitted_T1 = fitted_T1_T(K);
            fitted_tank1_m0(1, K) = utils().poly_func_pre(fitted_T1, m01_params, m01_MU);
            fitted_tank1_I0zz(1, K) = utils().zero_func_pre(fitted_T1, I01zz_params, I01zz_MU);
            fitted_tank1_I0xy(1, K) = utils().zero_func_pre(fitted_T1, I01xy_params, I01xy_MU);
            fitted_tank1_I0xz(1, K) = utils().zero_func_pre(fitted_T1, I01xz_params, I01xz_MU);
            fitted_tank1_I0yz(1, K) = utils().zero_func_pre(fitted_T1, I01yz_params, I01yz_MU);
            
            fitted_tank1_m(1, K)    = utils().segm_func_pre(fitted_T1,  m1_params1,   m1_params2   , m1_segm_point, "m1");
            fitted_tank1_I0xx(1, K) = utils().segm_func_pre(fitted_T1, I01xx_params1, I01xx_params2, I01xx_segm_point, "I01xx");
            fitted_tank1_I0yy(1, K) = utils().segm_func_pre(fitted_T1, I01yy_params1, I01yy_params2, I01yy_segm_point, "I01yy");
            fitted_tank1_L_0_x(1, K) = utils().poly_func_pre(fitted_T1, L01x_params, L01x_MU); % +4.745 看拟合值的效果加不加 4.745 不影响            
            fitted_tank1_L_x(1, K) = utils().poly_func_pre(fitted_T1, L1x_params, L1x_MU);  % +4.745 看拟合值的效果加不加 4.745 不影响
    
            %     fitted_tank1_pendulum_length(1, K) = utils().poly_func_pre(fitted_T1, length1_params, length1_MU);
            %     fitted_tank1_pendulum_length(1, K) = utils().segm_func_pre(fitted_T1, length1_params1, length1_params2, length1_segm_point, "length1");
            fitted_tank1_pendulum_length(1, K) = utils().pendulum_1_func_pre(fitted_T1, length1_params, length1_MU);
            %     fitted_tank1_pendulum_length(1, K)  = utils().pendulum_1_func_pre2(fitted_T1,length1_xi, length1_yi);
            
            fitted_tank1_zeta(1, K) = utils().poly_func_pre(fitted_T1, ckesa1_params, ckesa1_MU);
            
            % 根据拟合的函数，实时计算 第2个贮箱 内的各个量
            fitted_T2 = fitted_T2_T(K);
            fitted_tank2_m0(1, K)   = utils().poly_func_pre(fitted_T2, m02_params, m02_MU);
            fitted_tank2_I0zz(1, K) = utils().zero_func_pre(fitted_T2, I02zz_params, I02zz_MU);
            fitted_tank2_I0xy(1, K) = utils().zero_func_pre(fitted_T2, I02xy_params, I02xy_MU);
            fitted_tank2_I0xz(1, K) = utils().zero_func_pre(fitted_T2, I02xz_params, I02xz_MU);
            fitted_tank2_I0yz(1, K) = utils().zero_func_pre(fitted_T2, I02yz_params, I02yz_MU);
            
            fitted_tank2_m(1, K)    = utils().poly_func_pre(fitted_T2, m2_params, m2_MU);
            fitted_tank2_I0xx(1, K) = utils().segm_func_pre(fitted_T2, I02xx_params1, I02xx_params2, I02xx_segm_point, "I02xx");
            fitted_tank2_I0yy(1, K) = utils().segm_func_pre(fitted_T2, I02yy_params1, I02yy_params2, I02yy_segm_point, "I02yy");
            fitted_tank2_L_0_x(1, K)  = utils().poly_func_pre(fitted_T2, L02x_params, L02x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank2_L_x(1, K)  = utils().poly_func_pre(fitted_T2, L2x_params, L2x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank2_pendulum_length(1, K) = utils().poly_func_pre(fitted_T2, length2_params, length2_MU);
            fitted_tank2_zeta(1, K) = utils().poly_func_pre(fitted_T2, ckesa2_params, ckesa2_MU);
            
            % 根据拟合的函数，给出 第3个贮箱 内的各个量的拟合值
            % 贮箱3和贮箱4 虽然一直动态消耗，运行完就可以看到拟合效果，此处设置仅为了一次性全部运行完看结果，不跑整个大循环
            fitted_T3 = fitted_T3_T(K);
            fitted_tank3_m0(1, K)  = utils().poly_func_pre(fitted_T3, m03_params, m03_MU);
            fitted_tank3_m(1, K)    = utils().poly_func_pre(fitted_T3, m3_params, m3_MU);
            fitted_tank3_I0xx(1, K) = utils().segm_func_pre(fitted_T3, I03xx_params1, I03xx_params2, I03xx_segm_point, "I043xx");
            fitted_tank3_I0yy(1, K) = utils().segm_func_pre(fitted_T3, I03yy_params1, I03yy_params2, I03yy_segm_point, "I03yy");
            fitted_tank3_L_0_x(1, K)  = utils().poly_func_pre(fitted_T3, L03x_params, L03x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank3_L_x(1, K)  = utils().poly_func_pre(fitted_T3, L3x_params, L3x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank3_pendulum_length(1, K) = utils().poly_func_pre(fitted_T3, length3_params, length3_MU);
            fitted_tank3_zeta(1, K) = utils().poly_func_pre(fitted_T3, ckesa3_params, ckesa3_MU);
            
            % 根据拟合的函数，给出 第4个贮箱 内的各个量的拟合值
            % 贮箱3和贮箱4 虽然一直动态消耗，运行完就可以看到拟合效果，此处设置仅为了一次性全部运行完看结果，不跑整个大循环
            fitted_T4 = fitted_T4_T(K);
            fitted_tank4_m0(1, K)  = utils().poly_func_pre(fitted_T4, m04_params, m04_MU);
            fitted_tank4_m(1, K)    = utils().poly_func_pre(fitted_T4, m4_params, m4_MU);
            fitted_tank4_I0xx(1, K) = utils().segm_func_pre(fitted_T4, I04xx_params1, I04xx_params2, I04xx_segm_point, "I04xx");
            fitted_tank4_I0yy(1, K) = utils().segm_func_pre(fitted_T4, I04yy_params1, I04yy_params2, I04yy_segm_point, "I04yy");
            fitted_tank4_L_0_x(1, K)  = utils().poly_func_pre(fitted_T4, L04x_params, L04x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank4_L_x(1, K)  = utils().poly_func_pre(fitted_T4, L4x_params, L4x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank4_pendulum_length(1, K) = utils().poly_func_pre(fitted_T4, length4_params, length4_MU);
            fitted_tank4_zeta(1, K) = utils().poly_func_pre(fitted_T4, ckesa4_params, ckesa4_MU);
        end %20250414改为单独执行，不随大循环运行
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 计算（贮箱1和2除推进剂外的）各个量（根据推进剂和拟合函数求出）的拟合值和EXCEL表中数据值的差异比；
        for n = 1:35
            % 根据拟合的函数，给出 第1个贮箱 内的各个量的拟合值
            fitted_tank1_m0_excel(n, 1) = utils().poly_func_pre(tank_dataset{1}(n,2), m01_params, m01_MU);
            fitted_tank1_m_excel(n, 1)    = utils().segm_func_pre(tank_dataset{1}(n,2),  m1_params1,   m1_params2   , m1_segm_point, "m1");
            fitted_tank1_I0xx_excel(n, 1) = utils().segm_func_pre(tank_dataset{1}(n,2), I01xx_params1, I01xx_params2, I01xx_segm_point, "I01xx");
            fitted_tank1_I0yy_excel(n, 1) = utils().segm_func_pre(tank_dataset{1}(n,2), I01yy_params1, I01yy_params2, I01yy_segm_point, "I01yy");
            fitted_tank1_L_0_x_excel(n, 1) = utils().poly_func_pre(tank_dataset{1}(n,2), L01x_params, L01x_MU); % +4.745 看拟合值的效果加不加 4.745 不影响
            fitted_tank1_L_x_excel(n, 1) = utils().poly_func_pre(tank_dataset{1}(n,2), L1x_params, L1x_MU);  % +4.745 看拟合值的效果加不加 4.745 不影响
            
            fitted_tank1_pendulum_length_excel(n, 1) = utils().pendulum_1_func_pre(tank_dataset{1}(n,2), length1_params, length1_MU);
            %         fitted_tank1_pendulum_length_excel(n, 1)  = utils().pendulum_1_func_pre2(tank_dataset{1}(n,2),length1_xi, length1_yi);
           
            fitted_tank1_zeta_excel(n, 1) = utils().poly_func_pre(tank_dataset{1}(n,2), ckesa1_params, ckesa1_MU);
            
            rate_fitted_tank1_m0_excel(n,1) = (fitted_tank1_m0_excel(n, 1))/tank_dataset{1}(n,3);
            rate_fitted_tank1_m_excel(n,1) = (fitted_tank1_m_excel(n, 1))/tank_dataset{1}(n,11);
            rate_fitted_tank1_I0xx_excel(n,1) = (fitted_tank1_I0xx_excel(n, 1))/tank_dataset{1}(n,5);
            rate_fitted_tank1_I0yy_excel(n,1) = (fitted_tank1_I0yy_excel(n, 1))/tank_dataset{1}(n,6);
            rate_fitted_tank1_L_0_x_excel(n,1) = (fitted_tank1_L_0_x_excel(n, 1))/tank_dataset{1}(n,4);
            rate_fitted_tank1_L_x_excel(n,1) = (fitted_tank1_L_x_excel(n, 1))/tank_dataset{1}(n,12);
            rate_fitted_tank1_pendulum_length_excel(n,1) = (fitted_tank1_pendulum_length_excel(n, 1))/tank_dataset{1}(n,14);
            rate_fitted_tank1_zeta_excel(n,1) = (fitted_tank1_zeta_excel(n, 1))/tank_dataset{1}(n,16);
            
            % 根据拟合的函数，实时计算 第2个贮箱 内的各个量
            fitted_tank2_m0_excel(n, 1)  = utils().poly_func_pre(tank_dataset{2}(n,2), m02_params, m02_MU);
            fitted_tank2_m_excel(n, 1)    = utils().poly_func_pre(tank_dataset{2}(n,2), m2_params, m2_MU);
            fitted_tank2_I0xx_excel(n, 1) = utils().segm_func_pre(tank_dataset{2}(n,2), I02xx_params1, I02xx_params2, I02xx_segm_point, "I02xx");
            fitted_tank2_I0yy_excel(n, 1) = utils().segm_func_pre(tank_dataset{2}(n,2), I02yy_params1, I02yy_params2, I02yy_segm_point, "I02yy");
            fitted_tank2_L_0_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{2}(n,2), L02x_params, L02x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank2_L_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{2}(n,2), L2x_params, L2x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank2_pendulum_length_excel(n, 1) = utils().poly_func_pre(tank_dataset{2}(n,2), length2_params, length2_MU);
            fitted_tank2_zeta_excel(n, 1) = utils().poly_func_pre(tank_dataset{2}(n,2), ckesa2_params, ckesa2_MU);
            
            rate_fitted_tank2_m0_excel(n,1) = (fitted_tank2_m0_excel(n, 1))/tank_dataset{2}(n,3);
            rate_fitted_tank2_m_excel(n,1) = (fitted_tank2_m_excel(n, 1))/tank_dataset{2}(n,11);
            rate_fitted_tank2_I0xx_excel(n,1) = (fitted_tank2_I0xx_excel(n, 1))/tank_dataset{2}(n,5);
            rate_fitted_tank2_I0yy_excel(n,1) = (fitted_tank2_I0yy_excel(n, 1))/tank_dataset{2}(n,6);
            rate_fitted_tank2_L_0_x_excel(n,1) = (fitted_tank2_L_0_x_excel(n, 1))/tank_dataset{2}(n,4);
            rate_fitted_tank2_L_x_excel(n,1) = (fitted_tank2_L_x_excel(n, 1))/tank_dataset{2}(n,12);
            rate_fitted_tank2_pendulum_length_excel(n,1) = (fitted_tank2_pendulum_length_excel(n, 1))/tank_dataset{2}(n,14);
            rate_fitted_tank2_zeta_excel(n,1) = (fitted_tank2_zeta_excel(n, 1))/tank_dataset{2}(n,16);
        end
        
        % 计算（贮箱3和4除推进剂外的）各个量（根据推进剂和拟合函数求出）的拟合值和EXCEL表中数据值的差异比；
        for n = 1:20
            % 根据拟合的函数，给出 第3个贮箱 内的各个量的拟合值
            fitted_tank3_m0_excel(n, 1)  = utils().poly_func_pre(tank_dataset{3}(n,2), m03_params, m03_MU);
            fitted_tank3_m_excel(n, 1)    = utils().poly_func_pre(tank_dataset{3}(n,2), m3_params, m3_MU);
            fitted_tank3_I0xx_excel(n, 1) = utils().segm_func_pre(tank_dataset{3}(n,2), I03xx_params1, I03xx_params2, I03xx_segm_point, "I043xx");
            fitted_tank3_I0yy_excel(n, 1) = utils().segm_func_pre(tank_dataset{3}(n,2), I03yy_params1, I03yy_params2, I03yy_segm_point, "I03yy");
            fitted_tank3_L_0_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{3}(n,2), L03x_params, L03x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank3_L_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{3}(n,2), L3x_params, L3x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank3_pendulum_length_excel(n, 1) = utils().poly_func_pre(tank_dataset{3}(n,2), length3_params, length3_MU);
            fitted_tank3_zeta_excel(n, 1) = utils().poly_func_pre(tank_dataset{3}(n,2), ckesa3_params, ckesa3_MU);
            
            rate_fitted_tank3_m0_excel(n,1) = (fitted_tank3_m0_excel(n, 1))/tank_dataset{3}(n,3);
            rate_fitted_tank3_m_excel(n,1) = (fitted_tank3_m_excel(n, 1))/tank_dataset{3}(n,11);
            rate_fitted_tank3_I0xx_excel(n,1) = (fitted_tank3_I0xx_excel(n, 1))/tank_dataset{3}(n,5);
            rate_fitted_tank3_I0yy_excel(n,1) = (fitted_tank3_I0yy_excel(n, 1))/tank_dataset{3}(n,6);
            rate_fitted_tank3_L_0_x_excel(n,1) = (fitted_tank3_L_0_x_excel(n, 1))/tank_dataset{3}(n,4);
            rate_fitted_tank3_L_x_excel(n,1) = (fitted_tank3_L_x_excel(n, 1))/tank_dataset{3}(n,12);
            rate_fitted_tank3_pendulum_length_excel(n,1) = (fitted_tank3_pendulum_length_excel(n, 1))/tank_dataset{3}(n,14);
            rate_fitted_tank3_zeta_excel(n,1) = (fitted_tank3_zeta_excel(n, 1))/tank_dataset{3}(n,16);
            
            
            % 根据拟合的函数，给出 第4个贮箱 内的各个量的拟合值
            fitted_tank4_m0_excel(n, 1)  = utils().poly_func_pre(tank_dataset{4}(n,2), m04_params, m04_MU);
            fitted_tank4_m_excel(n, 1)    = utils().poly_func_pre(tank_dataset{4}(n,2), m4_params, m4_MU);
            fitted_tank4_I0xx_excel(n, 1) = utils().segm_func_pre(tank_dataset{4}(n,2), I04xx_params1, I04xx_params2, I04xx_segm_point, "I04xx");
            fitted_tank4_I0yy_excel(n, 1) = utils().segm_func_pre(tank_dataset{4}(n,2), I04yy_params1, I04yy_params2, I04yy_segm_point, "I04yy");
            fitted_tank4_L_0_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{4}(n,2), L04x_params, L04x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank4_L_x_excel(n, 1)  = utils().poly_func_pre(tank_dataset{4}(n,2), L4x_params, L4x_MU);%  +4.745;  %看拟合值的效果加不加 4.745 不影响
            fitted_tank4_pendulum_length_excel(n, 1) = utils().poly_func_pre(tank_dataset{4}(n,2), length4_params, length4_MU);
            fitted_tank4_zeta_excel(n, 1) = utils().poly_func_pre(tank_dataset{4}(n,2), ckesa4_params, ckesa4_MU);
            
            rate_fitted_tank4_m0_excel(n,1) = (fitted_tank4_m0_excel(n, 1))/tank_dataset{4}(n,3);
            rate_fitted_tank4_m_excel(n,1) = (fitted_tank4_m_excel(n, 1))/tank_dataset{4}(n,11);
            rate_fitted_tank4_I0xx_excel(n,1) = (fitted_tank4_I0xx_excel(n, 1))/tank_dataset{4}(n,5);
            rate_fitted_tank4_I0yy_excel(n,1) = (fitted_tank4_I0yy_excel(n, 1))/tank_dataset{4}(n,6);
            rate_fitted_tank4_L_0_x_excel(n,1) = (fitted_tank4_L_0_x_excel(n, 1))/tank_dataset{4}(n,4);
            rate_fitted_tank4_L_x_excel(n,1) = (fitted_tank4_L_x_excel(n, 1))/tank_dataset{4}(n,12);
            rate_fitted_tank4_pendulum_length_excel(n,1) = (fitted_tank4_pendulum_length_excel(n, 1))/tank_dataset{4}(n,14);
            rate_fitted_tank4_zeta_excel(n,1) = (fitted_tank4_zeta_excel(n, 1))/tank_dataset{4}(n,16);
        end
        
        figure(26)
        subplot(4,3,1),plot(fitted_T1_T(1,1:K), fitted_tank1_m0(1,1:K), 'b-', mT_info{1}, m0_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 固定块质量');
        grid on;
        legend("m01拟合值","m01原始值");
        subplot(4,3,2),plot(fitted_T1_T(1,1:K), fitted_tank1_m(1,1:K), 'b-', mT_info{1}, m_info{1}, 'ro',  'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 800]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 晃动块质量');
        grid on;
        legend("m1拟合值","m1原始值");
        subplot(4,3,3),plot(fitted_T1_T(1,1:K), fitted_tank1_L_0_x(1,1:K), 'b-',  mT_info{1}, L_0_x_info{1}, 'ro',  'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-0.3 0.4]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title(' 固定块质量的高度');
        grid on;
        legend("L01x拟合值","L01x原始值");
        subplot(4,3,4),plot(fitted_T1_T(1,1:K), fitted_tank1_L_x(1,1:K), 'b-',  mT_info{1}, L_x_info{1}, 'ro',  'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-0.4 0.2]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title('晃动块质量的高度 ');
        grid on;
        legend("L1x拟合值","L1x原始值");
        subplot(4,3,5),plot(fitted_T1_T(1,1:K), fitted_tank1_pendulum_length(1,1:K), 'b-', mT_info{1}, pendulum_length_info{1}, 'ro',  'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 15]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 晃动块摆长');
        grid on;
        legend("l1拟合值","l1原始值");
        subplot(4,3,6),plot(fitted_T1_T(1,1:K), fitted_tank1_zeta(1,1:K), 'b-',  mT_info{1}, zeta_ratio_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 0.05]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼');
        grid on;
        legend("zeta1 ratio拟合值","zeta1 ratio原始值");
        subplot(4,3,7),plot(fitted_T1_T(1,1:K), fitted_tank1_I0xx(1,1:K), 'b-', mT_info{1}, I0xx_info{1}, 'ro',  'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 1000])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XX');
        grid on;
        legend("I01xx拟合值","I01xx原始值");
        subplot(4,3,8),plot(fitted_T1_T(1,1:K), fitted_tank1_I0yy(1,1:K), 'b-',  mT_info{1}, I0yy_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([0 1000]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YY');
        grid on;
        legend("I01yy拟合值","I01yy原始值");
        subplot(4,3,9),plot(fitted_T1_T(1,1:K), fitted_tank1_I0zz(1,1:K), 'b-',  mT_info{1}, I0zz_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-4e-5  4e-5]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量ZZ');
        grid on;
        legend("I01zz拟合值","I01zz原始值");
        subplot(4,3,10),plot(fitted_T1_T(1,1:K), fitted_tank1_I0xy(1,1:K), 'b-', mT_info{1}, I0xy_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-0.005 0.005])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XY');
        grid on;
        legend("I01xy拟合值","I01xy原始值");
        subplot(4,3,11),plot(fitted_T1_T(1,1:K), fitted_tank1_I0xz(1,1:K), 'b-',  mT_info{1}, I0xz_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-10e-4 10e-4])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XZ');
        grid on;
        legend("I01xz拟合值","I01xz原始值");
        subplot(4,3,12),plot(fitted_T1_T(1,1:K), fitted_tank1_I0yz(1,1:K), 'b-',  mT_info{1}, I0yz_info{1}, 'ro', 'LineWidth', 1.5);
        xlim([0 3000]);
        ylim([-0.006 0.006])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YZ');
        grid on;
        legend("I01yz拟合值","I01yz原始值");

        figure(27)
        subplot(4,3,1),plot(fitted_T2_T(1,1:K), fitted_tank2_m0(1,1:K), 'b-', mT_info{2}, m0_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 3500]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 固定块质量');
        grid on;
        legend("m02拟合值","m02原始值");
        subplot(4,3,2),plot(fitted_T2_T(1,1:K), fitted_tank2_m(1,1:K), 'b-', mT_info{2}, m_info{2}, 'ro',  'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 2000]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 晃动块质量');
        grid on;
        legend("m2拟合值","m2原始值");
        subplot(4,3,3),plot(fitted_T2_T(1,1:K), fitted_tank2_L_0_x(1,1:K), 'b-',  mT_info{2}, L_0_x_info{2}, 'ro',  'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-0.3  1.2]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title(' 固定块质量的高度');
        grid on;
        legend("L02x拟合值","L02x原始值");
        subplot(4,3,4),plot(fitted_T2_T(1,1:K), fitted_tank2_L_x(1,1:K), 'b-',  mT_info{2}, L_x_info{2}, 'ro',  'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-0.9  -0.4]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title('晃动块质量的高度 ');
        grid on;
        legend("L2x拟合值","L2x原始值");
        subplot(4,3,5),plot(fitted_T2_T(1,1:K), fitted_tank2_pendulum_length(1,1:K), 'b-', mT_info{2}, pendulum_length_info{2}, 'ro',  'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 2]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 晃动块摆长');
        grid on;
        legend("l2拟合值","l2原始值");
        subplot(4,3,6),plot(fitted_T2_T(1,1:K), fitted_tank2_zeta(1,1:K), 'b-',  mT_info{2}, zeta_ratio_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 0.01]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼');
        grid on;
        legend("zeta2 ratio拟合值","zeta2 ratio原始值");
        subplot(4,3,7),plot(fitted_T2_T(1,1:K), fitted_tank2_I0xx(1,1:K), 'b-', mT_info{2}, I0xx_info{2}, 'ro',  'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 350])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XX');
        grid on;
        legend("I02xx拟合值","I02xx原始值");
        subplot(4,3,8),plot(fitted_T2_T(1,1:K), fitted_tank2_I0yy(1,1:K), 'b-',  mT_info{2}, I0yy_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([0 350]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YY');
        grid on;
        legend("I02yy拟合值","I02yy原始值");
        subplot(4,3,9),plot(fitted_T2_T(1,1:K), fitted_tank2_I0zz(1,1:K), 'b-',  mT_info{2}, I0zz_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-10e-4 10e-4]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量ZZ');
        grid on;
        legend("I02zz拟合值","I02zz原始值");
        subplot(4,3,10),plot(fitted_T2_T(1,1:K), fitted_tank2_I0xy(1,1:K), 'b-', mT_info{2}, I0xy_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-0.01 0.01])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XY');
        grid on;
        legend("I02xy拟合值","I02xy原始值");
        subplot(4,3,11),plot(fitted_T2_T(1,1:K), fitted_tank2_I0xz(1,1:K), 'b-',  mT_info{2}, I0xz_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-0.01 0.01])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XZ');
        grid on;
        legend("I02xz拟合值","I02xz原始值");
        subplot(4,3,12),plot(fitted_T2_T(1,1:K), fitted_tank2_I0yz(1,1:K), 'b-',  mT_info{2}, I0yz_info{2}, 'ro', 'LineWidth', 1.5);
        xlim([0 3500]);
        ylim([-0.005 0.005])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YZ');
        grid on;
        legend("I02yz拟合值","I02yz原始值");

        figure(28)
        subplot(4,3,1),plot(fitted_T3_T(1, 1:K), fitted_tank3_m0(1, 1:K), 'b-', mT_info{3}, m0_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([0 6000]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 固定块质量');
        grid on;
        legend("m03拟合值","m03原始值");
        subplot(4,3,2),plot(fitted_T3_T(1, 1:K), fitted_tank3_m(1, 1:K), 'b-', mT_info{3}, m_info{3}, 'ro',  'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([0 8000]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 晃动块质量');
        grid on;
        legend("m3拟合值","m3原始值");
        subplot(4,3,3),plot(fitted_T3_T(1, 1:K), fitted_tank3_L_0_x(1, 1:K), 'b-',  mT_info{3}, L_0_x_info{3}, 'ro',  'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([0 4]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title(' 固定块质量的高度');
        grid on;
        legend("L03x拟合值","L03x原始值");
        subplot(4,3,4),plot(fitted_T3_T(1, 1:K), fitted_tank3_L_x(1,1:K), 'b-',  mT_info{3}, L_x_info{3}, 'ro',  'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([0 2]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title('晃动块质量的高度 ');
        grid on;
        legend("L3x拟合值","L3x原始值");
        subplot(4,3,5),plot(fitted_T3_T(1, 1:K), fitted_tank3_pendulum_length(1,1:K), 'b-', mT_info{3}, pendulum_length_info{3}, 'ro',  'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([0 3]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 晃动块摆长');
        grid on;
        legend("l3拟合值","l3原始值");
        subplot(4,3,6),plot(fitted_T3_T(1, 1:K), fitted_tank3_zeta(1,1:K), 'b-',  mT_info{3}, zeta_ratio_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2e-3 4e-3]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼');
        grid on;
        legend("zeta3 ratio拟合值","zeta3 ratio原始值");
        subplot(4,3,7),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0xx(1,1:K), 'b-', mT_info{3}, I0xx_info{3}, 'ro',  'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-500 3000])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XX');
        grid on;
        legend("I03xx拟合值","I03xx原始值");
        subplot(4,3,8),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0yy(1,1:K), 'b-',  mT_info{3}, I0yy_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-500 2000]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YY');
        grid on;
        legend("I03yy拟合值","I03yy原始值");
        subplot(4,3,9),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0zz(1,1:K), 'b-',  mT_info{3}, I0zz_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-0.02 0.02]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量ZZ');
        grid on;
        legend("I03zz拟合值","I03zz原始值");
        subplot(4,3,10),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0xy(1,1:K), 'b-', mT_info{3}, I0xy_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-1500 1500])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XY');
        grid on;
        legend("I03xy拟合值","I03xy原始值");
        subplot(4,3,11),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0xz(1,1:K), 'b-',  mT_info{3}, I0xz_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-0.2 0.7])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XZ');
        grid on;
        legend("I03xz拟合值","I03xz原始值");
        subplot(4,3,12),plot(fitted_T3_T(1, 1:K), fitted_tank3_I0yz(1,1:K), 'b-',  mT_info{3}, I0yz_info{3}, 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-0.05 0.07])
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YZ');
        grid on;
        legend("I03yz拟合值","I03yz原始值");

        figure(29)
        subplot(4,3,1),plot(fitted_T4_T(1, 1:K), fitted_tank4_m0(1,1:K), 'b-', mT_info{4}, m0_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 2000]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 固定块质量');
        grid on;
        legend("m04拟合值","m04原始值");
        subplot(4,3,2),plot(fitted_T4_T(1, 1:K), fitted_tank4_m(1,1:K), 'b-', mT_info{4}, m_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('质量(kg)');
        title(' 晃动块质量');
        grid on;
        legend("m4拟合值","m4原始值");
        subplot(4,3,3),plot(fitted_T4_T(1, 1:K), fitted_tank4_L_0_x(1,1:K), 'b-', mT_info{4}, L_0_x_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 10]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title(' 固定块质量的高度');
        grid on;
        legend("L04x拟合值","L04x原始值");
        subplot(4,3,4),plot(fitted_T4_T(1, 1:K), fitted_tank4_L_x(1,1:K), 'b-', mT_info{4}, L_x_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 1]);
        xlabel('推进剂质量(kg)');
        ylabel('高度(m)');
        title('晃动块质量的高度 ');
        grid on;
        legend("L4x拟合值","L4x原始值");
        subplot(4,3,5),plot(fitted_T4_T(1, 1:K), fitted_tank4_pendulum_length(1,1:K), 'b-', mT_info{4}, pendulum_length_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 3]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 晃动块摆长');
        grid on;
        legend("l4拟合值","l4原始值");
        subplot(4,3,6),plot(fitted_T4_T(1, 1:K), fitted_tank4_zeta(1,1:K), 'b-', mT_info{4}, zeta_ratio_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([0 8e-3]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼');
        grid on;
        legend("zeta4 ratio拟合值","zeta4 ratio原始值");
        subplot(4,3,7),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0xx(1,1:K), 'b-', mT_info{4}, I0xx_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-300 300]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XX');
        grid on;
        legend("I04xx拟合值","I04xx原始值");
        subplot(4,3,8),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0yy(1,1:K), 'b-', mT_info{4}, I0yy_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-300 300]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YY');
        grid on;
        legend("I04yy拟合值","I04yy原始值");
        subplot(4,3,9),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0zz(1, 1:K), 'b-', mT_info{4}, I0zz_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-1 1]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量ZZ');
        grid on;
        legend("I04zz拟合值","I04zz原始值");
        subplot(4,3,10),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0xy(1, 1:K), 'b-', mT_info{4}, I0xy_info{4}, 'ro', 'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-4e-3 4e-3]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XY');
        grid on;
        legend("I04xy拟合值","I04xy原始值");
        subplot(4,3,11),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0xz(1, 1:K), 'b-',mT_info{4}, I0xz_info{4}, 'ro',  'LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-5e-3 5e-3]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量XZ');
        grid on;
        legend("I04xz拟合值","I04xz原始值");
        subplot(4,3,12),plot(fitted_T4_T(1, 1:K), fitted_tank4_I0yz(1, 1:K), 'b-', mT_info{4}, I0yz_info{4}, 'ro','LineWidth', 1.5);
        xlim([0 6000]);
        ylim([-5e-3 5e-3]);
        xlabel('推进剂质量(kg)');
        ylabel('惯量(kg·m²)');
        title(' 固定块质量等效惯量YZ');
        grid on;
        legend("I04yz拟合值","I04yz原始值");

        figure(1990)
        subplot(2,1,1),plot(tank_dataset{1}(:,2), fitted_tank1_pendulum_length_excel(:, 1), 'b-', tank_dataset{1}(:,2), tank_dataset{1}(:,14), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        ylim([0 13]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 摆长');
        grid on;
        legend("摆长1的拟合值","摆长1的原始值");
        subplot(2,1,2),plot(tank_dataset{2}(:,2), fitted_tank2_pendulum_length_excel(:, 1), 'b-', tank_dataset{2}(:,2), tank_dataset{2}(:,14), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([0.2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 摆长');
        grid on;
        legend("摆长2的拟合值","摆长2的原始值");

        figure(1991)
        subplot(4,1,1),plot(tank_dataset{1}(:,2), fitted_tank1_m0_excel(:, 1), 'b',tank_dataset{1}(:,2), tank_dataset{1}(:, 3), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 贮箱1固定块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{1}(:,2), fitted_tank1_m_excel(:, 1), 'b',tank_dataset{1}(:,2), tank_dataset{1}(:, 11), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 贮箱1晃动块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{1}(:,2), fitted_tank1_I0xx_excel(:, 1), 'b',tank_dataset{1}(:,2), tank_dataset{1}(:, 5),'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' 贮箱1I0xx拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{1}(:,2), fitted_tank1_I0yy_excel(:, 1), 'b',tank_dataset{1}(:,2), tank_dataset{1}(:, 6),'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' 贮箱1I0yy摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1992)
        subplot(4,1,1),plot(tank_dataset{1}(:,2), fitted_tank1_L_0_x_excel(:, 1), 'b',tank_dataset{1}(:,2),  tank_dataset{1}(:, 4), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 贮箱1固定块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{1}(:,2), fitted_tank1_L_x_excel(:, 1), 'b',tank_dataset{1}(:,2),  tank_dataset{1}(:, 12), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 贮箱1晃动块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{1}(:,2), fitted_tank1_pendulum_length_excel(:, 1), 'b',tank_dataset{1}(:,2),  tank_dataset{1}(:, 14), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 贮箱1摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{1}(:,2), fitted_tank1_zeta_excel(:, 1), 'b',tank_dataset{1}(:,2),  tank_dataset{1}(:, 16), 'ro', 'LineWidth', 1.5);
        xlim([0 2500]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 贮箱1阻尼拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1993)
        subplot(4,1,1),plot(tank_dataset{2}(:,2), fitted_tank2_m0_excel(:, 1), 'b',tank_dataset{2}(:,2), tank_dataset{2}(:, 3), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 贮箱2固定块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{2}(:,2), fitted_tank2_m_excel(:, 1), 'b',tank_dataset{2}(:,2), tank_dataset{2}(:, 11), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 贮箱2晃动块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{2}(:,2), fitted_tank2_I0xx_excel(:, 1), 'b',tank_dataset{2}(:,2), tank_dataset{2}(:, 5),'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' 贮箱2I0xx拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{2}(:,2), fitted_tank2_I0yy_excel(:, 1), 'b',tank_dataset{2}(:,2), tank_dataset{2}(:, 6), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' 贮箱2I0yy摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1994)
        subplot(4,1,1),plot(tank_dataset{2}(:,2), fitted_tank2_L_0_x_excel(:, 1), 'b',tank_dataset{2}(:,2),  tank_dataset{2}(:, 4), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 贮箱2固定块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{2}(:,2), fitted_tank2_L_x_excel(:, 1), 'b',tank_dataset{2}(:,2),  tank_dataset{2}(:, 12), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 贮箱2晃动块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{2}(:,2), fitted_tank2_pendulum_length_excel(:, 1), 'b',tank_dataset{2}(:,2),  tank_dataset{2}(:, 14), 'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 贮箱2摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{2}(:,2), fitted_tank2_zeta_excel(:, 1), 'b',tank_dataset{2}(:,2),  tank_dataset{2}(:, 16),'ro', 'LineWidth', 1.5);
        xlim([0 4000]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 贮箱2阻尼拟合值和原始值');
        grid on;
        legend("拟合值","原始值");     
        
        figure(1995)
        subplot(4,1,1),plot(tank_dataset{3}(:,2), fitted_tank3_m0_excel(:, 1), 'b',tank_dataset{3}(:,2), tank_dataset{3}(:, 3), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 贮箱1固定块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{3}(:,2), fitted_tank3_m_excel(:, 1), 'b',tank_dataset{3}(:,2), tank_dataset{3}(:, 11), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 贮箱1晃动块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{3}(:,2), fitted_tank3_I0xx_excel(:, 1), 'b',tank_dataset{3}(:,2), tank_dataset{3}(:, 5),'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' 贮箱1I0xx拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{3}(:,2), fitted_tank3_I0yy_excel(:, 1), 'b',tank_dataset{3}(:,2), tank_dataset{3}(:, 6),'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' 贮箱1I0yy摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1996)
        subplot(4,1,1),plot(tank_dataset{3}(:,2), fitted_tank3_L_0_x_excel(:, 1), 'b',tank_dataset{3}(:,2),  tank_dataset{3}(:, 4), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 贮箱1固定块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{3}(:,2), fitted_tank3_L_x_excel(:, 1), 'b',tank_dataset{3}(:,2),  tank_dataset{3}(:, 12), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 贮箱1晃动块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{3}(:,2), fitted_tank3_pendulum_length_excel(:, 1), 'b',tank_dataset{3}(:,2),  tank_dataset{3}(:, 14), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 贮箱1摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{3}(:,2), fitted_tank3_zeta_excel(:, 1), 'b',tank_dataset{3}(:,2),  tank_dataset{3}(:, 16), 'ro', 'LineWidth', 1.5);
        xlim([0 10000]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 贮箱1阻尼拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1997)
        subplot(4,1,1),plot(tank_dataset{4}(:,2), fitted_tank4_m0_excel(:, 1), 'b',tank_dataset{4}(:,2), tank_dataset{4}(:, 3), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 贮箱2固定块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{4}(:,2), fitted_tank4_m_excel(:, 1), 'b',tank_dataset{4}(:,2), tank_dataset{4}(:, 11), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 贮箱2晃动块质量拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{4}(:,2), fitted_tank4_I0xx_excel(:, 1), 'b',tank_dataset{4}(:,2), tank_dataset{4}(:, 5),'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' 贮箱2I0xx拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{4}(:,2), fitted_tank4_I0yy_excel(:, 1), 'b',tank_dataset{4}(:,2), tank_dataset{4}(:, 6), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' 贮箱2I0yy摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        
        figure(1998)
        subplot(4,1,1),plot(tank_dataset{4}(:,2), fitted_tank4_L_0_x_excel(:, 1), 'b',tank_dataset{4}(:,2),  tank_dataset{4}(:, 4), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 贮箱2固定块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,2),plot(tank_dataset{4}(:,2), fitted_tank4_L_x_excel(:, 1), 'b',tank_dataset{4}(:,2),  tank_dataset{4}(:, 12), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 贮箱2晃动块高度拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,3),plot(tank_dataset{4}(:,2), fitted_tank4_pendulum_length_excel(:, 1), 'b',tank_dataset{4}(:,2),  tank_dataset{4}(:, 14), 'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 贮箱2摆长拟合值和原始值');
        grid on;
        legend("拟合值","原始值");
        subplot(4,1,4),plot(tank_dataset{4}(:,2), fitted_tank4_zeta_excel(:, 1), 'b',tank_dataset{4}(:,2),  tank_dataset{4}(:, 16),'ro', 'LineWidth', 1.5);
        xlim([0 5000]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 贮箱2阻尼拟合值和原始值');
        grid on;
        legend("拟合值","原始值");       
               
        figure(2994)
        subplot(4,1,1),plot(tank_dataset{1}(:,2), rate_fitted_tank1_m0_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_m0_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 固定块质量拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,2),plot(tank_dataset{1}(:,2), rate_fitted_tank1_m_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_m_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 晃动块质量拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,3),plot(tank_dataset{1}(:,2), rate_fitted_tank1_I0xx_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_I0xx_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' I0xx拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,4),plot(tank_dataset{1}(:,2), rate_fitted_tank1_I0yy_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_I0yy_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' I0yy摆长拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        
        figure(2995)
        subplot(4,1,1),plot(tank_dataset{1}(:,2), rate_fitted_tank1_L_0_x_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_L_0_x_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 固定块高度拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,2),plot(tank_dataset{1}(:,2), rate_fitted_tank1_L_x_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_L_x_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 晃动块高度拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,3),plot(tank_dataset{1}(:,2), rate_fitted_tank1_pendulum_length_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_pendulum_length_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 摆长拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        subplot(4,1,4),plot(tank_dataset{1}(:,2), rate_fitted_tank1_zeta_excel(:, 1), 'b',tank_dataset{2}(:,2), rate_fitted_tank2_zeta_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 4000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼拟合值和原始值差的百分比');
        grid on;
        legend("贮箱1","贮箱2");
        
        figure(2996)
        subplot(4,1,1),plot(tank_dataset{3}(:,2), rate_fitted_tank3_m0_excel(:, 1), 'b*',tank_dataset{4}(:,2), rate_fitted_tank4_m0_excel(:, 1), 'g*', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块质量(kg)');
        title(' 固定块质量拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,2),plot(tank_dataset{3}(:,2), rate_fitted_tank3_m_excel(:, 1), 'b*',tank_dataset{4}(:,2), rate_fitted_tank4_m_excel(:, 1), 'g*', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块质量(kg)');
        title(' 晃动块质量拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,3),plot(tank_dataset{3}(:,2), rate_fitted_tank3_I0xx_excel(:, 1), 'b*',tank_dataset{4}(:,2), rate_fitted_tank4_I0xx_excel(:, 1), 'g*', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('I0xx');
        title(' I0xx拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,4),plot(tank_dataset{3}(:,2), rate_fitted_tank3_I0yy_excel(:, 1), 'b*',tank_dataset{4}(:,2), rate_fitted_tank4_I0yy_excel(:, 1), 'g*', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('I0yy');
        title(' I0yy摆长拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        
        figure(2997)
        subplot(4,1,1),plot(tank_dataset{3}(:,2), rate_fitted_tank3_L_0_x_excel(:, 1), 'b',tank_dataset{4}(:,2), rate_fitted_tank4_L_0_x_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('固定块高度(m)');
        title(' 固定块高度拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,2),plot(tank_dataset{3}(:,2), rate_fitted_tank3_L_x_excel(:, 1), 'b',tank_dataset{4}(:,2), rate_fitted_tank4_L_x_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('晃动块高度(m)');
        title(' 晃动块高度拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,3),plot(tank_dataset{3}(:,2), rate_fitted_tank3_pendulum_length_excel(:, 1), 'b',tank_dataset{4}(:,2), rate_fitted_tank4_pendulum_length_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('摆长(m)');
        title(' 摆长拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
        subplot(4,1,4),plot(tank_dataset{3}(:,2), rate_fitted_tank3_zeta_excel(:, 1), 'b',tank_dataset{4}(:,2), rate_fitted_tank4_zeta_excel(:, 1), 'g', 'LineWidth', 1.5);
        xlim([0 10000]);
        ylim([-2 2]);
        xlabel('推进剂质量(kg)');
        ylabel('阻尼');
        title(' 阻尼拟合值和原始值差的百分比');
        grid on;
        legend("贮箱3","贮箱4");
       
    end
    
    run('model_parameter.m');
       
    if t== t_start
        a_R_init = [F_T/(m_R + sigma_m0_1to4 + sigma_m_1to4),0,0]';
        a_R = a_R_init;
        a_R_actu = a_R_init;
        
        % 输入 U_adrc
        U_adrc = inv(B_0) * ( -k1 * theta -k2 * omegaa - Z);
        % 初始状态U_adrc_actu_in和U_adrc相同
        U_adrc_actu = U_adrc;
        % 此处初始化一次U_adrc_filter_new 仅是为了在 DISTURB_FILTER_KEY==OFF 时，代码运行不报错。 当 DISTURB_FILTER_KEY==ON 时 ，可用于将 U_adrc_filter_new 的值传递回 U_adrc_filter_cur 变量。
        U_adrc_filter = U_adrc;
        U_adrc_filter_new = U_adrc; 
        if T_CTR_DEFERING == ON
            U_adrc_0 = U_adrc;
            U_adrc_1 = U_adrc;
            U_adrc_2 = U_adrc;
            U_adrc_3 = U_adrc;
            U_adrc_4 = U_adrc;
            U_adrc_5 = U_adrc;
            U_adrc_6 = U_adrc;
            U_adrc_7 = U_adrc;
            U_adrc_8 = U_adrc;
            U_adrc_9 = U_adrc;
        end
    end
    
    %%
    % 设定:
    % 当前循环:变量均为 *_cur, 更新后的变量均为 *_new,
    % 这样的好处是：使用变量时只需要确认是用 更新前的值 还是 更新后的值即可，无需考虑更新顺序。
    
    theta_cur = theta;
    omegaa_cur = omegaa;
    if T_CTR_DEFERING == ON
        % 2025.02.10
        % result = t/t_interval;
        result = K;
        mod_fac = 5; % 10
        if mod(result,mod_fac) == 0
            U_adrc_cur = U_adrc_0;
        end
        if mod(result,mod_fac) == 1
            U_adrc_cur = U_adrc_1;
        end
        if mod(result,mod_fac) == 2
            U_adrc_cur = U_adrc_2;
        end
        if mod(result,mod_fac) == 3
            U_adrc_cur = U_adrc_3;
        end
        if mod(result,mod_fac) == 4
            U_adrc_cur = U_adrc_4;
        end
%         if mod(result,mod_fac) == 5
%             U_adrc_cur = U_adrc_5;
%         end
%         if mod(result,mod_fac) == 6
%             U_adrc_cur = U_adrc_6;
%         end
%         if mod(result,mod_fac) == 7
%             U_adrc_cur = U_adrc_7;
%         end
%         if mod(result,mod_fac) == 8
%             U_adrc_cur = U_adrc_8;
%         end
%         if mod(result,mod_fac) == 9
%             U_adrc_cur = U_adrc_9;
%         end
    else
        U_adrc_cur = U_adrc;
    end
    % U_adrc_cur = U_adrc;
    Z_cur = Z;
    a_R_cur = a_R;
    eta_cur = eta;
    
    d_omegaa_cur = d_omegaa;
    
    alpha_y_cur = alpha_y;
    alpha_z_cur = alpha_z;
    d_alpha_y_cur = d_alpha_y;
    d_alpha_z_cur = d_alpha_z;
    dd_alpha_y_cur = dd_alpha_y;
    dd_alpha_z_cur = dd_alpha_z;
    
    U_adrc_actu_cur = U_adrc_actu;
    alpha_y_actu_cur = alpha_y_actu;
    alpha_z_actu_cur = alpha_z_actu;
    d_alpha_y_actu_cur = d_alpha_y_actu;
    d_alpha_z_actu_cur = d_alpha_z_actu;
    dd_alpha_y_actu_cur = dd_alpha_y_actu;
    dd_alpha_z_actu_cur = dd_alpha_z_actu;
    dd_Mx_cur = dd_Mx;
    d_Mx_cur = d_Mx;
    
    U_adrc_filter_cur = U_adrc_filter;
    
    gamma_1_cur = gamma_1;
    gamma_2_cur = gamma_2;
    gamma_3_cur = gamma_3;
    gamma_4_cur = gamma_4;
    d_gamma_1_cur = d_gamma_1;
    d_gamma_2_cur = d_gamma_2;
    d_gamma_3_cur = d_gamma_3;
    d_gamma_4_cur = d_gamma_4;
    dd_gamma_1_cur = dd_gamma_1;
    dd_gamma_2_cur = dd_gamma_2;
    dd_gamma_3_cur = dd_gamma_3;
    dd_gamma_4_cur = dd_gamma_4;
    
    f0_cur = f0;
    
    
    % 每隔 t_fig_interval 步长，将当前值添加到输出矩阵 *_res。  --提交时 可以把所有的这种形式写成 元胞
    if mod(t, T_fig) == 0
        
        t4fig = [t4fig, t];
        
        theta_res = [theta_res, theta_cur];
        omegaa_res = [omegaa_res, omegaa_cur];
        U_adrc_res = [U_adrc_res,U_adrc_cur];
        EST_disturb_res = [EST_disturb_res, Z_cur];
        a_R_res = [a_R_res, a_R_cur];
        
        alpha_y_res = [alpha_y_res, alpha_y_cur];
        alpha_z_res = [alpha_z_res, alpha_z_cur];
        d_alpha_y_res = [d_alpha_y_res, d_alpha_y_cur];
        d_alpha_z_res = [d_alpha_z_res, d_alpha_z_cur];
        dd_alpha_y_res = [dd_alpha_y_res, dd_alpha_y_cur];
        dd_alpha_z_res = [dd_alpha_z_res, dd_alpha_z_cur];
        
        gamma_1_res = [gamma_1_res, gamma_1_cur];
        gamma_2_res = [gamma_2_res, gamma_2_cur];
        gamma_3_res = [gamma_3_res, gamma_3_cur];
        gamma_4_res = [gamma_4_res, gamma_4_cur];
        d_gamma_1_res = [d_gamma_1_res, d_gamma_1_cur];
        d_gamma_2_res = [d_gamma_2_res, d_gamma_2_cur];
        d_gamma_3_res = [d_gamma_3_res, d_gamma_3_cur];
        d_gamma_4_res = [d_gamma_4_res, d_gamma_4_cur];
        dd_gamma_1_res = [dd_gamma_1_res, dd_gamma_1_cur];
        dd_gamma_2_res = [dd_gamma_2_res, dd_gamma_2_cur];
        dd_gamma_3_res = [dd_gamma_3_res, dd_gamma_3_cur];
        dd_gamma_4_res = [dd_gamma_4_res, dd_gamma_4_cur];
        
        U_adrc_actu_res = [U_adrc_actu_res, U_adrc_actu_cur];
        
        alpha_y_actu_res = [alpha_y_actu_res, alpha_y_actu_cur];
        alpha_z_actu_res = [alpha_z_actu_res, alpha_z_actu_cur];
        d_alpha_y_actu_res = [d_alpha_y_actu_res, d_alpha_y_actu_cur];
        d_alpha_z_actu_res = [d_alpha_z_actu_res, d_alpha_z_actu_cur];
        dd_alpha_y_actu_res = [dd_alpha_y_actu_res, dd_alpha_y_actu_cur];
        dd_alpha_z_actu_res = [dd_alpha_z_actu_res, dd_alpha_z_actu_cur];
    end
    
    if (T_CTR ==ON) && (mod(t, T_ctr) == 0)
        t4ctr = [t4ctr, t];
    end
    
    if  (T_ESO==ON) && (mod(t,T_eso) == 0)
        t4eso = [t4eso, t];
    end
    % 利用 a_R(0) 、gamma_k(0)、dgamma_k(0)、U(0)、F_E(0)、T_E(0) 求d_omegga
    
    %%    
    % Equation（16）
    F_E1_cur = [F_T; -m_e * r_e * dd_alpha_z_actu_cur; -m_e * r_e * dd_alpha_y_actu_cur;];

    F_E = B_F * U_adrc_cur + F_E1_cur;

    % Equation（15）
    T_E1_cur = [ 0 ; (m_e * r_e * x_T_x + J_e) * dd_alpha_y_actu_cur; (m_e * r_e * x_T_x + J_e) * dd_alpha_z_actu_cur;];
    
    % Equation（20）
    D_t_dd_alpha_y_dd_alpha_z = inv(A_w) * ( T_E1_cur + A_r1' * inv(A_r) * F_E1_cur);
    % Equation（21）
    % B = inv(A_w) * (B_T + (A_r1'* inv(A_r) * B_F)); % 常值，已放循环外面计算
    
    % 计算 d_omegaa - Equation（18）
    d_omegaa_new = - (A_hat_dgamma1 * d_gamma_1_cur + A_hat_dgamma2 * d_gamma_2_cur + A_hat_dgamma3 * d_gamma_3_cur + A_hat_dgamma4 * d_gamma_4_cur)...
        - (A_hat_gamma1 * gamma_1_cur + A_hat_gamma2 * gamma_2_cur + A_hat_gamma3 * gamma_3_cur + A_hat_gamma4 * gamma_4_cur)...
        + D_t_dd_alpha_y_dd_alpha_z + B * U_adrc_cur;  % A_hat_dgamma2/3/4 不是转置！！--上次未全部更正 20241002
      
    % 更新 omegaa
    omegaa_new = omegaa_cur + d_omegaa_new * t_interval;
    theta_new = theta_cur + omegaa_new * t_interval;
    
    % 估计扰动 Z
    if T_ESO == ON
        if  mod(t,T_eso) == 0
            % 更新 omegaa_eso,并记录当前值
            omegaa_eso = omegaa_cur;
            omegaa_eso_res = [omegaa_eso_res, omegaa_eso];
        end        
        
        % 2024.09.28 使用的扰动估计方法
        eta_new = eta_cur + ( -w_os * eta_cur - w_os^2 * omegaa_eso - w_os * B_0 * U_adrc_cur) * t_interval;
        Z_new = eta_cur + w_os * omegaa_eso;
        
    else
        eta_new = eta_cur + ( -w_o * eta_cur - w_o^2 * omegaa_cur - w_o * B_0 * U_adrc_cur) * t_interval;
        Z_new = eta_cur + w_o * omegaa_cur;
    end
    
    % 计算扰动函数 f_func -- % Equation（23）
    f_func = - (A_hat_dgamma1 * d_gamma_1_cur + A_hat_dgamma2 * d_gamma_2_cur + A_hat_dgamma3 * d_gamma_3_cur + A_hat_dgamma4 * d_gamma_4_cur)...
        - (A_hat_gamma1 * gamma_1_cur    + A_hat_gamma2  * gamma_2_cur    + A_hat_gamma3 * gamma_3_cur     + A_hat_gamma4 * gamma_4_cur)...
        + D_t_dd_alpha_y_dd_alpha_z + (B - B_0) * U_adrc_cur; % A_hat_dgamma2/3/4 不是转置！！--上次未全部更正 20241002
    
    f0_new = f_func;
    
    f_gamma = - (A_hat_dgamma1 * d_gamma_1_cur + A_hat_dgamma2 * d_gamma_2_cur + A_hat_dgamma3 * d_gamma_3_cur + A_hat_dgamma4 * d_gamma_4_cur)...
        - (A_hat_gamma1 * gamma_1_cur    + A_hat_gamma2  * gamma_2_cur    + A_hat_gamma3 * gamma_3_cur     + A_hat_gamma4 * gamma_4_cur); % A_hat_dgamma2/3/4 不是转置！！--上次未全部更正 20241002
    
    f_D = D_t_dd_alpha_y_dd_alpha_z;
    
    % Equation（8）
    dd_gamma_1_new = (-P12 * (inv(A_r) * A_r1 - L_bar_faslab_info{1}) * d_omegaa_cur - P12 * inv(A_r) * F_E...
        -P12 * inv(A_r) * ( m_1  *  (2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur)...
        + m_2  *  (2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur)...
        + m_3  *  (2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur)...
        + m_4  *  (2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur)...
        )...
        - (2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur))/l_1;
    
    % Equation（8）
    dd_gamma_2_new = (-P12 * (inv(A_r) * A_r1 - L_bar_faslab_info{2}) * d_omegaa_cur - P12 * inv(A_r) * F_E...
        -P12 * inv(A_r) * ( m_1  *  (2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur)...
        + m_2  *  (2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur)...
        + m_3  *  (2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur)...
        + m_4  *  (2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur)...
        )...
        - (2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur))/l_2;
    
    % Equation（8）
    dd_gamma_3_new = (-P12 * (inv(A_r) * A_r1 - L_bar_faslab_info{3}) * d_omegaa_cur - P12 * inv(A_r) * F_E...
        -P12 * inv(A_r) * ( m_1  *  (2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur)...
        + m_2  *  (2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur)...
        + m_3  *  (2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur)...
        + m_4  *  (2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur)...
        )...
        - (2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur))/l_3;
    % Equation（8）
    dd_gamma_4_new = (-P12 * (inv(A_r) * A_r1 - L_bar_faslab_info{4}) * d_omegaa_cur - P12 * inv(A_r) * F_E...
        -P12 * inv(A_r) * ( m_1  *  (2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur)...
        + m_2  *  (2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur)...
        + m_3  *  (2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur)...
        + m_4  *  (2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur)...
        )...
        - (2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur))/l_4;
    
    
    % 更新 d_gamma_k
    d_gamma_1_new = d_gamma_1_cur + dd_gamma_1_new * t_interval;
    d_gamma_2_new = d_gamma_2_cur + dd_gamma_2_new * t_interval;
    d_gamma_3_new = d_gamma_3_cur + dd_gamma_3_new * t_interval;
    d_gamma_4_new = d_gamma_4_cur + dd_gamma_4_new * t_interval;
    
    % 更新 gamma_k
    gamma_1_new = gamma_1_cur + d_gamma_1_new * t_interval;
    gamma_2_new = gamma_2_cur + d_gamma_2_new * t_interval;
    gamma_3_new = gamma_3_cur + d_gamma_3_new * t_interval;
    gamma_4_new = gamma_4_cur + d_gamma_4_new * t_interval;
    
    % 更新 a_R -- using Equation（7）
    a_R_new = inv(A_r) * (A_r1 * d_omegaa_cur + (m_1 * ( 2 * zeta_1 * sqrt(g * l_1) * d_gamma_1_cur + g * gamma_1_cur)...
        +  m_2 * ( 2 * zeta_2 * sqrt(g * l_2) * d_gamma_2_cur + g * gamma_2_cur)...
        +  m_3 * ( 2 * zeta_3 * sqrt(g * l_3) * d_gamma_3_cur + g * gamma_3_cur)...
        +  m_4 * ( 2 * zeta_4 * sqrt(g * l_4) * d_gamma_4_cur + g * gamma_4_cur)...
        ) ...
        + F_E );
    
    if mod(t, T_fig) == 0
        %display("new iteration:");
        a_R_new(1);
        a_R_new(2);
        a_R_new(3);
    end
    
    if T_CTR == ON % 周期控制否打开，则每隔T_ctr个步长更新下U_adrc_new，其它步长使用上一步的U。
        if B0_INSTANENOUS == OFF
            if  mod(t,T_ctr) == 0
                % 更新 U_adrc -- % 2024.09.28 使用的控制律
                U_adrc_new = inv(B_0) * ( -k1 * theta_cur -k2 * omegaa_cur - Z_cur);
                U_adrc_upd_res = [U_adrc_upd_res, U_adrc_new];
            else
                U_adrc_new = U_adrc_cur;
            end
        else % B0_INSTANENOUS == ON
            if  mod(t,T_ctr) == 0
                % 更新 U_adrc    %%%%% ！！！！！没用到，无需改动此处！！！！！%%%%%
                U_adrc_new = inv(B_0) * ( -k1 * theta_cur -k2 * omegaa_cur - Z_cur);
                % U_adrc_new = inv(B_0) * ( -k1 * theta_new -k2 * omegaa_new);
                U_adrc_upd_res = [U_adrc_upd_res, U_adrc_new];
                theta_avail = theta_cur;
                omegaa_avail = omegaa_cur;
                Z_avail = Z_cur;
            else
                % U_adrc_new = U_adrc_cur;
                U_adrc_new = inv(B_0) * ( -k1 * theta_avail -k2 * omegaa_avail - Z_avail);
            end
        end
    else % 周期控制关闭，则每个步长均更新 U_adrc_new
        U_adrc_new = inv(B_0) * ( -k1 * theta_cur -k2 * omegaa_cur - Z_cur);
    end
    
    % 更新 alpha_k
    delta_U = U_adrc_new - U_adrc_cur;
    alpha_y_new = alpha_y_cur + delta_U(2);
    alpha_z_new = alpha_z_cur + delta_U(3);
    %     d_alpha_y_new = d_alpha_y_cur + delta_U(2)/t_interval;
    %     d_alpha_z_new = d_alpha_z_cur + delta_U(3)/t_interval;
    d_Mx_new = delta_U(1)/t_interval;
    d_alpha_y_new = delta_U(2)/t_interval;
    d_alpha_z_new = delta_U(3)/t_interval;
    
    dd_Mx_new = (d_Mx_new - d_Mx_cur)/t_interval;
    dd_alpha_y_new = (d_alpha_y_new - d_alpha_y_cur)/t_interval;
    dd_alpha_z_new = (d_alpha_z_new - d_alpha_z_cur)/t_interval;
    
    %%
    
    % 对 U_adrc_new(1) 进行限幅
    if  abs(U_adrc_new(1)) > 200
        U_adrc_new(1) = 200 * sign(U_adrc_new(1));
    end
    
    Res_before_filter(:,K) = U_adrc_new;
    
    if DISTURB_FILTER_KEY == ON
        % 2024.12.19新增--滤波环节，扰动频率为 1 Hz
        delta_filter_U = U_adrc_new - U_adrc_filter_cur;

        disturb_freq = 1; % 0.25--0.4--0.5selected
        
        T_filter = 1/(disturb_freq*2*3.14);
        d_filter_u = delta_filter_U/T_filter;
        U_adrc_filter_new = U_adrc_filter_cur + d_filter_u * t_interval;
        U_adrc_new = U_adrc_filter_new;
        Res_after_filter(:,K) = U_adrc_filter_new;
    end
    
    % 更新 alpha_k---bak3
    delta_actu_U = U_adrc_new - U_adrc_actu_cur;
    % 增加摆动角和摆动角速度限幅
    % 设定摇摆发动机的最大摆动角和摆动角速度
    actuator_freq = 10; % 10
    T_actu = 1/(actuator_freq*2*3.14);
    alpha_yz_max = 4 * (2 * 3.14/360);
    alpha_yz_vol_max = 5 * (2 * 3.14/360); % rad/s
    d_delta_uy = delta_actu_U(2)/T_actu;
    d_delta_uz = delta_actu_U(3)/T_actu;
    if abs(d_delta_uy) > alpha_yz_vol_max
        d_delta_uy = alpha_yz_vol_max * sign(d_delta_uy);
    end
    if abs(d_delta_uz) > alpha_yz_vol_max
        d_delta_uz = alpha_yz_vol_max * sign(d_delta_uz) ;
    end
    alpha_y_actu_new = alpha_y_actu_cur + d_delta_uy * t_interval;
    alpha_z_actu_new = alpha_z_actu_cur + d_delta_uz * t_interval;
    
    if abs(alpha_y_actu_new) > alpha_yz_max
        alpha_y_actu_new = alpha_yz_max * sign(alpha_y_actu_new);
    end
    if abs(alpha_z_actu_new) > alpha_yz_max
        alpha_z_actu_new = alpha_yz_max * sign(alpha_z_actu_new);
    end
    
    U_adrc_actu_new = [U_adrc_new(1), alpha_y_actu_new, alpha_z_actu_new ]';% 限幅修正后的U
    
    d_alpha_y_actu_new = d_delta_uy;
    d_alpha_z_actu_new = d_delta_uz;
    
    dd_alpha_y_actu_new = (d_alpha_y_actu_new - d_alpha_y_actu_cur)/t_interval;
    dd_alpha_z_actu_new = (d_alpha_z_actu_new - d_alpha_z_actu_cur)/t_interval;
    
    U_adrc_new = U_adrc_actu_new;
    
    %%
    % 将新值传递回变量，以供下次循环使用
    theta = theta_new;
    omegaa = omegaa_new;
    % U_adrc = U_adrc_new;
    if T_CTR_DEFERING == ON
        if mod(result,mod_fac) == 0
            U_adrc_0 = U_adrc_new;
        end
        if mod(result,mod_fac) == 1
            U_adrc_1 = U_adrc_new;
        end
        if mod(result,mod_fac) == 2
            U_adrc_2 = U_adrc_new;
        end
        if mod(result,mod_fac) == 3
            U_adrc_3 = U_adrc_new;
        end
        if mod(result,mod_fac) == 4
            U_adrc_4 = U_adrc_new;
        end
%         if mod(result,mod_fac) == 5
%             U_adrc_5 = U_adrc_new;
%         end
%         if mod(result,mod_fac) == 6
%             U_adrc_6 = U_adrc_new;
%         end
%         if mod(result,mod_fac) == 7
%             U_adrc_7 = U_adrc_new;
%         end
%         if mod(result,mod_fac) == 8
%             U_adrc_8 = U_adrc_new;
%         end
%         if mod(result,mod_fac) == 9
%             U_adrc_9 = U_adrc_new;
%         end
    else
        U_adrc = U_adrc_new;
    end
    Z = Z_new;
    a_R = a_R_new;
    eta = eta_new;
    d_omegaa = d_omegaa_new;
    
    alpha_y = alpha_y_new;
    alpha_z = alpha_z_new;
    d_alpha_y = d_alpha_y_new;
    d_alpha_z = d_alpha_z_new;
    dd_alpha_y = dd_alpha_y_new;
    dd_alpha_z = dd_alpha_z_new;
    
    gamma_1 = gamma_1_new;
    gamma_2 = gamma_2_new;
    gamma_3 = gamma_3_new;
    gamma_4 = gamma_4_new;
    d_gamma_1 = d_gamma_1_new;
    d_gamma_2 = d_gamma_2_new;
    d_gamma_3 = d_gamma_3_new;
    d_gamma_4 = d_gamma_4_new;
    dd_gamma_1 = dd_gamma_1_new;
    dd_gamma_2 = dd_gamma_2_new;
    dd_gamma_3 = dd_gamma_3_new;
    dd_gamma_4 = dd_gamma_4_new;
    
    U_adrc_actu = U_adrc_actu_new;
    
    alpha_y_actu = alpha_y_actu_new;
    alpha_z_actu = alpha_z_actu_new;
    d_alpha_y_actu = d_alpha_y_actu_new;
    d_alpha_z_actu = d_alpha_z_actu_new;
    dd_alpha_y_actu = dd_alpha_y_actu_new;
    dd_alpha_z_actu = dd_alpha_z_actu_new;
    
    dd_Mx = dd_Mx_new;
    d_Mx = d_Mx_new;
    
    U_adrc_filter = U_adrc_filter_new;
    
    f0 = f0_new;
    
    if mod(t, T_fig) == 0
        
        % 记录本次新出现状态的结果
        disturb_func_res = [disturb_func_res, f_func];
        F_E1_res = [F_E1_res, F_E1_cur];
        F_E_res = [F_E_res, F_E];
        T_E1_res = [T_E1_res, T_E1_cur];
        D_res = [D_res, f_D];
        f_gamma_res = [f_gamma_res, f_gamma];
        
    end
end
% dest_path = './pic_res_202501082/';  % 保存地址的目录，当无对应文件夹时，需要创建
%%

figure(111);
% set(gcf,'position',[0,0,1080,720]);
subplot(3,1,1),plot(t4fig, theta_res(1, :)*57.3, 'b', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('度数(°)','FontSize', 16,'FontWeight','bold');
title('输出 theta 在 X 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("theta x");
subplot(3,1,2),plot(t4fig, theta_res(2, :) * 57.3, 'b', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('度数(°)','FontSize', 16,'FontWeight','bold');
title('输出 theta 在 Y 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("theta y");
subplot(3,1,3),plot(t4fig, theta_res(3, :) * 57.3, 'b', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('度数(°)','FontSize', 16,'FontWeight','bold');
title('输出 theta 在 Z 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("theta z");
if FIG_SAV_LM == ON
    % 保存图形为 PNG 文件
    file_name111 = [dest_path, 'm_1=', num2str(m_1_mul_fac), ...
        'm_2=',num2str(m_2_mul_fac), 'm_3=',num2str(m_3_mul_fac), 'm_4=',num2str(m_4_mul_fac),...
        'pen_1= ',num2str(pendulum_length_rob_1),...
        'pen_2= ',num2str(pendulum_length_rob_2),...
        'pen_3= ',num2str(pendulum_length_rob_3),...
        'pen4= ',num2str(pendulum_length_rob_4),...
        'L12_1= ',num2str(L12_1),'L12_2= ',num2str(L12_2),...
        'L34_3= ',num2str(L34_3),'L34_4= ',num2str(L34_4),...
        'm_T1= ',num2str(m_T1_add_fac),...
        'm_T2= ',num2str(m_T2_add_fac),...
        'm_T3= ',num2str(m_T3_add_fac),...
        'm_T4= ',num2str(m_T4_add_fac),...
        'theta.png'];
    saveas(gcf, file_name111);  % 使用当前图形窗口保存
    pause(2);
end

figure(112);
% set(gcf,'position',[0,0,1080,720]);
subplot(3,1,1),plot(t4fig, omegaa_res(1, :) * 57.3 , 'r', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('omega(°/s)','FontSize', 16,'FontWeight','bold');
title('输出 omega 在 X 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("omega x");
subplot(3,1,2),plot(t4fig, omegaa_res(2, :) * 57.3, 'r', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('omega(°/s','FontSize', 16,'FontWeight','bold');
title('输出 omega 在 Y 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("omega y");
subplot(3,1,3),plot(t4fig, omegaa_res(3, :) * 57.3, 'r', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('omega(°/s','FontSize', 16,'FontWeight','bold');
title('输出 omega 在 Z 轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("omega z");
if FIG_SAV_LM == ON
    % 保存图形为 PNG 文件
    file_name112 = [dest_path, 'm_1=', num2str(m_1_mul_fac), ...
        'm_2=',num2str(m_2_mul_fac), 'm_3=',num2str(m_3_mul_fac), 'm_4=',num2str(m_4_mul_fac),...
        'pen_1= ',num2str(pendulum_length_rob_1),...
        'pen_2= ',num2str(pendulum_length_rob_2),...
        'pen_3= ',num2str(pendulum_length_rob_3),...
        'pen4= ',num2str(pendulum_length_rob_4),...
        'L12_1= ',num2str(L12_1),'L12_2= ',num2str(L12_2),...
        'L34_3= ',num2str(L34_3),'L34_4= ',num2str(L34_4),...
        'm_T1= ',num2str(m_T1_add_fac),...
        'm_T2= ',num2str(m_T2_add_fac),...
        'm_T3= ',num2str(m_T3_add_fac),...
        'm_T4= ',num2str(m_T4_add_fac),...
        'omega.png'];
    saveas(gcf, file_name112);  % 使用当前图形窗口保存
    pause(2);
end

% 绘制 观测器估计扰动和计算扰动的对比
figure(2);
% set(gcf,'position',[0,0,1080,720]);
subplot(3,1,1),plot(t4fig, EST_disturb_res(1, :), 'b', t4fig, disturb_func_res(1, :), 'g-.', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('扰动量','FontSize', 16,'FontWeight','bold');
title('扰动量在X轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend('est d','d func','FontSize', 6,'FontWeight','bold');
subplot(3,1,2),plot(t4fig, EST_disturb_res(2, :), 'b', t4fig, disturb_func_res(2, :), 'g-.', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('扰动量','FontSize', 16,'FontWeight','bold');
title('扰动量在Y轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend('est d','d func','FontSize', 6);
subplot(3,1,3),plot(t4fig, EST_disturb_res(3, :), 'b', t4fig, disturb_func_res(3, :), 'g-.', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('扰动量','FontSize', 16,'FontWeight','bold');
title('扰动量在Z轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend('est d','d func','FontSize', 6);
if FIG_SAV_LM == ON
    % 保存图形为 PNG 文件
    file_name2 = [dest_path, 'm_1=', num2str(m_1_mul_fac), ...
        'm_2=',num2str(m_2_mul_fac), 'm_3=',num2str(m_3_mul_fac), 'm_4=',num2str(m_4_mul_fac),...
        'pen_1= ',num2str(pendulum_length_rob_1),...
        'pen_2= ',num2str(pendulum_length_rob_2),...
        'pen_3= ',num2str(pendulum_length_rob_3),...
        'pen4= ',num2str(pendulum_length_rob_4),...
        'L12_1= ',num2str(L12_1),'L12_2= ',num2str(L12_2),...
        'L34_3= ',num2str(L34_3),'L34_4= ',num2str(L34_4),...
        'm_T1= ',num2str(m_T1_add_fac),...
        'm_T2= ',num2str(m_T2_add_fac),...
        'm_T3= ',num2str(m_T3_add_fac),...
        'm_T4= ',num2str(m_T4_add_fac),...
        'Z.png'];
    saveas(gcf, file_name2);  % 使用当前图形窗口保存
    pause(2);
end

figure(200)
subplot(3,1,1),plot(t4fig, U_adrc_res(1, :), 'r', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('输入M(N·m)','FontSize', 16,'FontWeight','bold');
title(' 限幅后输入在X轴的分量','FontSize', 16,'FontWeight','bold');
grid on;
legend("U x",'FontSize', 6);
subplot(3,1,2),plot(t4fig, alpha_y_actu_res(1, :) * 57.3, 'b', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel(' alpha y (°)','FontSize', 16,'FontWeight','bold');
title(' 限幅后输入-摇摆角在Y轴的量','FontSize', 16,'FontWeight','bold');
grid on;
legend(' al y ','FontSize', 6);
subplot(3,1,3),plot(t4fig, alpha_z_actu_res(1, :) * 57.3, 'r', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel(' alpha z   (°)','FontSize', 16,'FontWeight','bold');
title(' 限幅后输入-摇摆角在Z轴的量','FontSize', 16,'FontWeight','bold');
grid on;
legend('alpha z','FontSize', 6);
if FIG_SAV_LM == ON
    % 保存图形为 PNG 文件
    file_name200 = [dest_path, 'm_1=', num2str(m_1_mul_fac), ...
        'm_2=',num2str(m_2_mul_fac), 'm_3=',num2str(m_3_mul_fac), 'm_4=',num2str(m_4_mul_fac),...
        'pen_1= ',num2str(pendulum_length_rob_1),...
        'pen_2= ',num2str(pendulum_length_rob_2),...
        'pen_3= ',num2str(pendulum_length_rob_3),...
        'pen4= ',num2str(pendulum_length_rob_4),...
        'L12_1= ',num2str(L12_1),'L12_2= ',num2str(L12_2),...
        'L34_3= ',num2str(L34_3),'L34_4= ',num2str(L34_4),...
        'm_T1= ',num2str(m_T1_add_fac),...
        'm_T2= ',num2str(m_T2_add_fac),...
        'm_T3= ',num2str(m_T3_add_fac),...
        'm_T4= ',num2str(m_T4_add_fac),...
        'U.png'];
    saveas(gcf, file_name200);  % 使用当前图形窗口保存
    pause(2);
end

figure(21)
subplot(3,4,1),plot(t4fig, gamma_1_res(1, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('X轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱1','FontSize', 16,'FontWeight','bold');
grid on;
legend("g1 x");
subplot(3,4,5),plot(t4fig, gamma_1_res(2, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Y轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱1','FontSize', 16,'FontWeight','bold');
grid on;
legend("g1 y");
subplot(3,4,9),plot(t4fig, gamma_1_res(3, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Z轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱1','FontSize', 16,'FontWeight','bold');
grid on;
legend("g1 z");
subplot(3,4,2),plot(t4fig, gamma_2_res(1, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('X轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱2','FontSize', 16,'FontWeight','bold');
grid on;
legend("g2 x");
subplot(3,4,6),plot(t4fig, gamma_2_res(2, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Y轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱2','FontSize', 16,'FontWeight','bold');
grid on;
legend("g2 y");
subplot(3,4,10),plot(t4fig, gamma_2_res(3, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Z轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱2','FontSize', 16,'FontWeight','bold');
grid on;
legend("g2 z");
subplot(3,4,3),plot(t4fig, gamma_3_res(1, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('X轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱3','FontSize', 16,'FontWeight','bold');
grid on;
legend("g3 x");
subplot(3,4,7),plot(t4fig, gamma_3_res(2, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Y轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱3','FontSize', 16,'FontWeight','bold');
grid on;
legend("g3 y");
subplot(3,4,11),plot(t4fig, gamma_3_res(3, :) * 57.3, 'g--', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Z轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱3','FontSize', 16,'FontWeight','bold');
grid on;
legend("g3 z");
subplot(3,4,4),plot(t4fig, gamma_4_res(1, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('X轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱4','FontSize', 16,'FontWeight','bold');
grid on;
legend("g4 x");
subplot(3,4,8),plot(t4fig, gamma_4_res(2, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Y轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱4','FontSize', 16,'FontWeight','bold');
grid on;
legend("g4 y");
subplot(3,4,12),plot(t4fig, gamma_4_res(3, :) * 57.3, 'g', 'LineWidth', 3);
set(gca,'fontsize',16, 'FontWeight','bold');
xlabel('时间 (秒)','FontSize', 16,'FontWeight','bold');
ylabel('Z轴分量(°)','FontSize', 16,'FontWeight','bold');
title(' 晃动角-贮箱4','FontSize', 16,'FontWeight','bold');
grid on;
legend("g4 z");
if FIG_SAV_LM == ON
    % 保存图形为 PNG 文件
    file_name21 = [dest_path, 'm_1=', num2str(m_1_mul_fac), ...
        'm_2=',num2str(m_2_mul_fac), 'm_3=',num2str(m_3_mul_fac), 'm_4=',num2str(m_4_mul_fac),...
        'pen_1= ',num2str(pendulum_length_rob_1),...
        'pen_2= ',num2str(pendulum_length_rob_2),...
        'pen_3= ',num2str(pendulum_length_rob_3),...
        'pen4= ',num2str(pendulum_length_rob_4),...
        'L12_1= ',num2str(L12_1),'L12_2= ',num2str(L12_2),...
        'L34_3= ',num2str(L34_3),'L34_4= ',num2str(L34_4),...
        'm_T1= ',num2str(m_T1_add_fac),...
        'm_T2= ',num2str(m_T2_add_fac),...
        'm_T3= ',num2str(m_T3_add_fac),...
        'm_T4= ',num2str(m_T4_add_fac),...
        'gamma.png'];
    saveas(gcf, file_name21);  % 使用当前图形窗口保存
    pause(2);
end

COUNT = COUNT+1;
disp(['第', num2str(COUNT),'次拉偏循环']);
%%
end
end
end
toc;
disp(['运行时间: ',num2str(toc)]);
% 获取当前日期和时间
currentTime = now;
disp(['当前日期和时间: ', datestr(currentTime)]);

%%