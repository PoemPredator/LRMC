%% Call ALL Function
function F = utils
    S = struct();
    S.poly_func_fit = @poly_func_fit;
    S.poly_func_fit2 = @poly_func_fit2;
    S.poly_func_pre = @poly_func_pre;
    S.zero_func_pre = @zero_func_pre;    
    S.predeal_poly_func_fit = @predeal_poly_func_fit;
    S.predeal_poly_func_pre = @predeal_poly_func_pre;
    S.pendulum_1_func_fit = @pendulum_1_func_fit;    
    S.pendulum_1_func_fit2 = @pendulum_1_func_fit2;    
    
    S.segm_func_fit = @segm_func_fit;
    S.segm_func_pre = @segm_func_pre;
    S.parabola_func_fit = @parabola_func_fit;
    S.parabola_func_pre = @parabola_func_pre;
    S.pendulum_1_func_pre = @pendulum_1_func_pre;
    S.pendulum_1_func_pre2 = @pendulum_1_func_pre2;
    S.abnorm_data_dealing = @abnorm_data_dealing;
    
    F = S;
end


%%
% 第一个贮箱各个量的变化函数
function [params, MU] = poly_func_fit(xx,yy)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第一个贮箱 m01/IO1zz/IO1xy/IO1xz/I01yz
%                        第二个贮箱 m2
x = xx;
y = yy;
% 使用 polyfit 进行三次拟合
[params, ~, MU]= polyfit(x, y, 3); % ~处是个结构体，没啥用，具体用法见 help polyfit
end

% 第一个贮箱各个量的变化函数
function [p, MU] = pendulum_1_func_fit(xx,yy)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第一个贮箱 m01/IO1zz/IO1xy/IO1xz/I01yz
%                        第二个贮箱 m2
x = xx;
y = yy;
% 使用 polyfit 进行4次拟合
[p,~,MU] = polyfit(x, y, 4); 
end

function [xi,yi] = pendulum_1_func_fit2(xx,yy)
% 适用于第一个贮箱的摆长

% 定义插值的自变量范围
% xi = min(xx):0.01:max(xx);
% 需要插值的点（包含超出范围的值）
% xi = linspace(0, 1, 10);
% xi = 0:max(xx);

xi = 0:0.1:max(xx);

% 线性插值，并使用外推方法
yi = interp1(xx, yy, xi, 'linear','extrap');
end

function [yy_out] = pendulum_1_func_pre2(x, xi,yi)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第一个贮箱 m01/IO1zz/IO1xy/IO1xz/I01yz
%                        第二个贮箱 m2

% 获取特定自变量对应的因变量值
specific_y = interp1(xi, yi, x);
yy_out = specific_y;
end


% 第一个贮箱各个量的变化函数
function [pre_res] = pendulum_1_func_pre(xx,p,mu)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第一个贮箱 m01/IO1zz/IO1xy/IO1xz/I01yz
%                        第二个贮箱 m2
x = xx;
y_fit = polyval(p, x,[],mu); % 计算拟合值
pre_res = y_fit;
end

function [params, MU] = poly_func_fit2(xx, yy, threshold)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第一个贮箱 m01/IO1zz/IO1xy/IO1xz/I01yz
%                        第二个贮箱 m2
x = xx;
y = yy;
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) > threshold
        % y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
% 使用 polyfit 进行三次拟合
[params, ~, MU]= polyfit(x, y, 3); % ~处是个结构体，没啥用，具体用法见 help polyfit
end

function [pre_res] = poly_func_pre(xx,params, MU)
% 三次多项式--预测
x = xx;
% 生成拟合的 y 值
% pre_res = polyval(params, x);
pre_res = polyval(params, (x - MU(1)) / MU(2));  % Transform back using MU
end

function [pre_res] = zero_func_pre(xx,~, ~)
% 三次多项式--预测
x = xx;
% 生成拟合的 y 值
% pre_res = polyval(params, x);
% pre_res = polyval(params, (x - MU(1)) / MU(2));  % Transform back using MU
pre_res = 0;  % Transform back using MU
end

%%%% 用之前再次在 test 里确认其拟合效果 %%%%
function [params, MU] = predeal_poly_func_fit(xx, yy, fluc_amp)
% 三次多项式--拟合
% 适用于如下变量的变化-- 第二个贮箱 IO2zz  fluc_amp---2e-4
%                                   IO2xy  fluc_amp---50
%                                   IO2xz  fluc_amp---0.01
%                                   I02yz  fluc_amp---5e-3
x = xx;
y = yy;
% 设置默认参数
if nargin < 3
    % 默认凸点不处理
    fluc_amp = 1000;
end

for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) > fluc_amp
        % y_raw(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end

% 使用 polyfit 进行三次拟合
[params, ~, MU]= polyfit(x, y, 3); % ~处是个结构体，没啥用，具体用法见 help polyfit
end

%%%% 用之前再次在 test 里确认其拟合效果 %%%%
function [pre_res] = predeal_poly_func_pre(xx,params, MU)
% 三次多项式--预测
% 适用于如下变量的变化-- 第二个贮箱 IO2zz/IO2xy/IO2xz/I02yz                         
x = xx;
% 生成拟合的 y 值
% pre_res = polyval(params, x);
pre_res = polyval(params, (x - MU(1)) / MU(2));  % Transform back using MU
end

function [params1, params2] = segm_func_fit(variable, xx, yy, segm_point)
% 分段函数--拟合，需要给出分段点
% 适用于如下变量的变化-- 第一个贮箱 m1/I01xx/I01yy
% 各变量的分段点如下
% m1 segm_point = 1558;
% I01xx segm_point = 358;
% I01yy segm_point = 358;

x_raw = xx;
y_raw = yy;
% 如果variable == "I03xx" 或"I03yy" ，则对Y值进行预处理
if (variable == "I03xx") || (variable == "I03yy")
    fluc_amp = 500;    
    for i = 2:length(y_raw)-1
        if abs(y_raw(i) - y_raw(i-1)) > fluc_amp
            % y_raw(i) = (y_raw(i-1) + y_raw(i+1))/2;
            y_raw(i) = y_raw(i-1);
        end
    end
end

x_1_raw = [];
y_1_raw = [];
x_2_raw = [];
y_2_raw = [];
for i = 1:length(x_raw)
    if x_raw(i) > segm_point
       x_2_raw = [x_2_raw, x_raw(i)];
       y_2_raw = [y_2_raw, y_raw(i)];
    else
       x_1_raw = [x_1_raw, x_raw(i)];
       y_1_raw = [y_1_raw, y_raw(i)];        
    end
end

% 后半段 分段函数--拟合
if (variable == "m1") || (variable == "length1" )  % m1的后半段用三次多项式拟合
    x = x_2_raw';
    y = y_2_raw';
    % 使用 polyfit 进行三次拟合
    [P, S, MU]= polyfit(x, y, 3);
    % params2 = [P, S, MU];
    params2.P = P;
    params2.S = S;
    params2.MU = MU;
    
else
    x_data = x_2_raw';
    y_data = y_2_raw';
    % 初始参数估计，这里我们假设二次函数的形式为 ax^2 + bx + c
    initial_guess2 = [1, 1, 1];
    % 定义二次函数模型
    model2 = @(params, x) params(1) * x.^2 + params(2) * x + params(3);
    % 使用lsqcurvefit进行曲线拟合
    options2 = optimoptions('lsqcurvefit', 'Display', 'off', 'MaxFunEvals', 10000);
    params2 = lsqcurvefit(model2, initial_guess2, x_data, y_data, [], [], options2);
end

% 前半段 分段函数--拟合
% m的变化曲线是两个抛物线，当变量为m时，前半段也为抛物线；其它变量时，前半段默认是各个值的平均数
if (variable == "m1") || (variable == "length1")
    x_data = x_1_raw';
    y_data = y_1_raw';
    % 初始参数估计，这里我们假设二次函数的形式为 ax^2 + bx + c
    initial_guess = [1, 1, 1];
    % 定义二次函数模型
    model1 = @(params, x) params(1) * x.^2 + params(2) * x + params(3);
    % 使用lsqcurvefit进行曲线拟合
    options1 = optimoptions('lsqcurvefit', 'Display', 'off', 'MaxFunEvals', 10000);
    params1 = lsqcurvefit(model1, initial_guess, x_data, y_data, [], [], options1);
% elseif variable == "length1" % length1也用上面的二次拟合 %20250310
%     x_data = x_1_raw';
%     y_data = y_1_raw';
%     % 使用 polyfit 进行三次拟合
%     [P, S, MU]= polyfit(x, y, 3);
%     % params2 = [P, S, MU];
%     params1.P = P;
%     params1.S = S;
%     params1.MU = MU;
else
    x_data = x_1_raw'; %该段函数为全 0 
    y_data = y_1_raw';
    aver_amp = sum(y_data)/length(y_data);
%     aver_amp = sum(abs(y_data))/length(y_data);
%     params1 = [0, 0, aver_amp];
    params1 = [0, 0, 0];
end
end


function [pre_res] = segm_func_pre(xx, params1, params2, segm_point,variable)
% 分段函数--预测，预测函数也需要给出分段点
% 设置默认参数

x = xx;
if x > segm_point
    % 构建拟合后的二次函数表达式
    if (variable == "m1") || (variable == "length1" ) % 对m1与推进剂的关系，其后半段为三次函数的预测值
        % params2 = [P, S, MU];
        P = params2.P;
        S = params2.S;
        MU = params2.MU;
        % 生成拟合的 y 值
        % y_fit = polyval(p, x);
        y_fit = polyval(P, (x - MU(1)) / MU(2));  % Transform back using MU
        % Y_fit = polyval(P, (X_fit - MU(1)) / MU(2));  % Transform back using MU    
        pre_res = y_fit;
    else   % 对其它变量与推进剂的关系，其后半段为二次函数的预测值
        fitted_function2 = @(x) params2(1) * x.^2 + params2(2) * x + params2(3);
        pre_res = fitted_function2(x);
    end
    
else
%     if variable == "length1"
%     % params2 = [P, S, MU];
%         P = params1.P;
%         S = params1.S;
%         MU = params1.MU;
%         % 生成拟合的 y 值
%         % y_fit = polyval(p, x);
%         y_fit = polyval(P, (x - MU(1)) / MU(2));  % Transform back using MU
%         % Y_fit = polyval(P, (X_fit - MU(1)) / MU(2));  % Transform back using MU    
%         pre_res = y_fit;
%     else        
    % 对m1，拟合的参数直接带入；对其它变量，全0的参数直接带入
    % 构建拟合后的二次函数表达式
    fitted_function1 = @(x) params1(1) * x.^2 + params1(2) * x + params1(3);
    pre_res = fitted_function1(x);
%     end
end
end

%%%%
% 
% function [params1, params2] = segm_func_m0_fit(variable, xx, yy, segm_point)
% % 分段函数--拟合，需要给出分段点
% % 适用于如下变量的变化-- 第一个贮箱 m1/I01xx/I01yy
% % 各变量的分段点如下
% % m1 segm_point = 1558;
% % I01xx segm_point = 358;
% % I01yy segm_point = 358;
% 
% % 设置默认参数
% if nargin < 4
%     segm_point = 358;
% end
% 
% x_raw = xx;
% y_raw = yy;
% 
% x_1_raw = [];
% y_1_raw = [];
% x_2_raw = [];
% y_2_raw = [];
% 
% for i = 1:length(x_raw)
%     if x_raw(i) > segm_point
%        x_2_raw = [x_2_raw, x_raw(i)];
%        y_2_raw = [y_2_raw, y_raw(i)];
%     else
%        x_1_raw = [x_1_raw, x_raw(i)];
%        y_1_raw = [y_1_raw, y_raw(i)];        
%     end
% end
% 
% % 后半段 分段函数--拟合
% if variable == "m1"   % m1的后半段用三次多项式拟合
%     x = x_2_raw';
%     y = y_2_raw';
%     % 使用 polyfit 进行三次拟合
%     [P, S, MU]= polyfit(x, y, 3);
% end
% 
% % 前半段 分段函数--拟合
% % m的变化曲线是两个抛物线，当变量为m时，前半段也为抛物线；其它变量时，前半段默认是各个值的平均数
% if variable == "m1"  
%     x_data = x_1_raw';
%     y_data = y_1_raw';
%     % 初始参数估计，这里我们假设二次函数的形式为 ax^2 + bx + c
%     initial_guess = [1, 1, 1];
%     % 定义二次函数模型
%     model1 = @(params, x) params(1) * x.^2 + params(2) * x + params(3);
%     % 使用lsqcurvefit进行曲线拟合
%     options1 = optimoptions('lsqcurvefit', 'Display', 'off', 'MaxFunEvals', 10000);
%     params1 = lsqcurvefit(model1, initial_guess, x_data, y_data, [], [], options1);
% end
% 
% end
% 
% 
% function [pre_res] = segm_func_m0_pre(xx, params1, params2, segm_point,variable)
% % 分段函数--预测，预测函数也需要给出分段点
% % 设置默认参数
% if nargin < 4
%     segm_point = 358;
% end
% x = xx;
% if x > segm_point
%     % 构建拟合后的二次函数表达式
%     if variable == "m1"  % 对m1与推进剂的关系，其后半段为三次函数的预测值
%         % params2 = [P, S, MU];
%         P = params2(1);
%         S = params2(2);
%         MU = params2(3);
%         % 生成拟合的 y 值
%         % y_fit = polyval(p, x);
%         y_fit = polyval(P, (x - MU(1)) / MU(2));  % Transform back using MU
%         % Y_fit = polyval(P, (X_fit - MU(1)) / MU(2));  % Transform back using MU      
%     else   % 对其它变量与推进剂的关系，其后半段为二次函数的预测值
%         fitted_function2 = @(x) params2(1) * x.^2 + params2(2) * x + params2(3);
%         pre_res = fitted_function2(x);
%     end
%     
% else  % 对m1，拟合的参数直接带入；对其它变量，全0的参数直接带入
%     % 构建拟合后的二次函数表达式
%     fitted_function1 = @(x) params1(1) * x.^2 + params1(2) * x + params1(3);
%     pre_res = fitted_function1(x);
% end
% end


%%%%

% 未按照变量名写成固定值，将 fluc_amp 作为参数 
function [params] = parabola_func_fit(xx, yy, fluc_amp)
% 抛物线--拟合
% 适用于如下变量的变化-- 第一个贮箱 L01x/L1x/length1/ckesa1
% L01x fluc_amp = 0.2;
% L1x fluc_amp= 0.05; 
% length1 fluc_amp = 1000 
% ckesa1 fluc_amp = 0.0005

if nargin  < 3
    %平滑处理
    fluc_amp = 1000; %很大代表不处理
end
x_raw = xx;
y_raw = yy;
for i = 2:length(y_raw)-1
    if abs(y_raw(i) - y_raw(i-1)) > fluc_amp
        % y_raw(i) = (y_raw(i-1) + y_raw(i+1))/2;
        y_raw(i) = y_raw(i-1);
    end
end
x_data = x_raw';
y_data = y_raw';
% 初始参数估计，这里我们假设二次函数的形式为 ax^2 + bx + c
initial_guess = [1, 1, 1];
% 定义二次函数模型
model = @(params, x) params(1) * x.^2 + params(2) * x + params(3);
% 使用lsqcurvefit进行曲线拟合
options = optimoptions('lsqcurvefit', 'Display', 'off', 'MaxFunEvals', 10000);
params = lsqcurvefit(model, initial_guess, x_data, y_data, [], [], options);

end

function [pre_res] = parabola_func_pre(xx, params)
% 抛物线--预测
x = xx;
% 构建拟合后的二次函数表达式
fitted_function = @(x) params(1) * x.^2 + params(2) * x + params(3);
pre_res = fitted_function(x);
end



function [output] = abnorm_data_dealing(variable,input)
%DEALING_WITH_ABNORMAL_DATA(); 此处显示有关此函数的摘要
%   此处显示详细说明

y = input;

L01x_threshold = 0.35;
L1x_threshold  = 0.1;
ckeas1_threshold = 0.0048;

L2x_threshold  = 0.06;
I02xx_threshold = 80;
I02zz_threshold = 3e-4;

I03xx_threshold = 500;
I03yy_threshold = 500;
I03zz_threshold = 0.005;

if variable == "L01x"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= L01x_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "L1x"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= L1x_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "ckesa1"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= ckeas1_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "L2x"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= L2x_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "I02xx"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= I02xx_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "I02zz"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= I02zz_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "I03xx"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= I03xx_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "I03yy"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= I03yy_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

if variable == "I03zz"
for i = 2:length(y)-1
    if abs(y(i) - y(i-1)) >= I03zz_threshold
%         y(i) = (y(i-1) + y(i+1))/2;
        y(i) = y(i-1);
    end
end
end

output = y;
end


