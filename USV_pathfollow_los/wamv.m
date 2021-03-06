clc
clear
close all
%load  path_lineangle.mat
load path
%% 相关参数定义
RefPos = path;
targetSpeed = 5;      % m/s
dt = 0.5;              % 时间间隔，单位：s
num_dt = 0;
L = 1.2;%船长
Lg = 2.373776; %重心位置
%% 主程序

% 车辆初始状态定义
%pos = RefPos(1,:)+1;
%pos = RefPos(1,:)+[0 -1];
pos = RefPos(1,:);
u = 0;
v = 0;
r = 0;
y_e = 1;
heading = 0.04;
V = [0; 0; 0];   % u, v, r
%tao = [Fx_min; 0; Fx_min* lr];
tao = [0; 0; 0];
% 将初始状态纳入实际状态数组中
pos_actual = pos;
heading_actual = heading;
u_actual = u;
v_actual = v;
r_actual = r;
latError_PP = y_e;
idx = 1;
%latError_PP = [];

% 循环遍历轨迹点
while idx < size(RefPos,1)-10
    % 计算横向误差与LOS角
    [LOSAngle,y_e,idx] = CaculateTargetAngle(pos, v, RefPos);
 
    % 计算控制量
    [F, delt] = my_pid(u, targetSpeed, heading, LOSAngle);
    tao = [F*cos(delt); F*sin(delt); -F*sin(delt)*Lg];
    % 如果误差过大，退出循迹
    if abs(y_e) > 10
        disp('误差过大，退出程序!\n')
        break
    end
    % 更新状态量
    V = calculate_dynamic_mode(tao, V, dt);    % 计算动力学模型
    global_V = kinimatic_model(V, heading);   % 将速度转换到全局坐标系下
    posnew(1,1) = pos(1,1)+ global_V(1,1) * dt;
    posnew(1,2) = pos(1,2)+ global_V(2,1) * dt;
    headingnew = heading + global_V(3,1) * dt;
    if headingnew > pi
        headingnew = headingnew-2*pi;
    elseif headingnew < -pi
        headingnew = headingnew+2*pi;
    end
    pos = posnew;
    heading =  headingnew;
    u = V(1,1);
    v = V(2,1);
    r = V(3,1);
    pos_actual(end+1,:) = pos;
    heading_actual(end+1,:) = heading;
    u_actual(end+1,:) = u;
    v_actual(end+1,:) = v;
    r_actual(end+1,:) = r;
    %latError_PP(end+1,:) = [idx,y_e];
    latError_PP(end+1,:) = y_e;
    num_dt = num_dt + 1;
end

%画图
figure(1)
plot(RefPos(:,1), RefPos(:,2), 'b');
xlabel('X坐标 / m');
ylabel('Y坐标 / m');
axis equal
hold on 
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),10, '.r');
    pause(0.01)
end
legend('规划车辆轨迹', '实际行驶轨迹')

% 将u, v, r画在同一个fifure上
figure(2)
t =[0 : dt : num_dt*dt];
plot(t, u_actual, t, v_actual, t, r_actual);
xlabel('t  /s'), ylabel('u/v/r')
title('speed - t')
legend('u', 'v', 'r');
% 将误差画出来
figure(3)
t =[0 : dt : num_dt*dt];
plot(t, latError_PP);
xlabel('t  /s'), ylabel('latError')
title('latError - t')
legend('横向误差');
average_latError = mean(latError_PP)
% 保存
path_PP = pos_actual;
save path_PP.mat path_PP
save latError_PP.mat latError_PP
%%  计算LOS角度
function  [LOSAngle,y_e,idx] = CaculateTargetAngle(pos, v, RefPos)
% 首先在参考轨迹上搜索离当前车辆位置最近的点
 sizeOfRefPos = size(RefPos,1);
 for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
 end
[~,idx] = min(dist); 
% 计算LOS角度
x1 = RefPos(idx,1);
x2 = RefPos(idx+1,1);
y1 = RefPos(idx,2);
y2 = RefPos(idx+1,2);
if abs(y2-y1)< 0.00001||abs(x2-x1)<0.00001
    if abs(y2-y1)< 0.00001
        m_CrossPoint_x = pos(1,1);
        m_CrossPoint_y = ( y2+y1)/2;
        y_e = sqrt((m_CrossPoint_x - pos(1,1))^2+(m_CrossPoint_y - pos(1,2))^2);
        maxlength = 30;
        minlength = 10;
        Delta_Length = (maxlength - minlength)*power(2.718,-10*y_e*y_e) + minlength;%计算前视距离
        if x2>=x1
           increaseFlag = 1;
        else
           increaseFlag = -1;
        end
        m_LosPoint_x = m_CrossPoint_x + increaseFlag*Delta_Length;
        m_LosPoint_y = m_CrossPoint_y ; 
    else
        m_CrossPoint_x = ( x2+x1)/2;
        m_CrossPoint_y = pos(1,2);
        y_e = sqrt((m_CrossPoint_x - pos(1,1))^2+(m_CrossPoint_y - pos(1,2))^2);
        maxlength = 30;
        minlength = 10;
        Delta_Length = (maxlength - minlength)*power(2.718,-10*y_e*y_e) + minlength;%计算前视距离
        if x2>=x1
           increaseFlag = 1;
        else
           increaseFlag = -1;
        end
        m_LosPoint_x = m_CrossPoint_x + increaseFlag*Delta_Length;
        m_LosPoint_y = m_CrossPoint_y ; 
    end
else
a1 = (y2-y1)/(x2-x1);  %过最近点，且与路径相切的斜率 
b1 = y1 - a1*x1;
a2 = -1/a1;
b2 = pos(1,2) - a2*pos(1,1);
m_CrossPoint_x = (b1 - b2)/(a2 - a1);
m_CrossPoint_y = (a2*b1 - a1*b2)/(a2 - a1);
y_e = sqrt((m_CrossPoint_x - pos(1,1))^2+(m_CrossPoint_y - pos(1,2))^2);
        maxlength = 30;
        minlength = 10;
Delta_Length = (maxlength - minlength)*power(2.718,-10*y_e*y_e) + minlength;%计算前视距离
%标识局部路径的x坐标是增加还是减少，1表示增加，-1表示减少
if x2>=x1
   increaseFlag = 1;
else
   increaseFlag = -1;
end
Delta_x = sqrt((Delta_Length*Delta_Length)/(1+a1*a1));
m_LosPoint_x = m_CrossPoint_x + increaseFlag*Delta_x;
m_LosPoint_y = m_CrossPoint_y + a1*increaseFlag*Delta_x; 
end
m_vectorLos_x = m_LosPoint_x - pos(1,1);
m_vectorLos_y = m_LosPoint_y - pos(1,2);

LOSAngle = atan2(m_vectorLos_y,m_vectorLos_x);
end
%% 计算控制量（PID）输入量为期望的u, 期望的phi, 以及u,v,r
function [F, delt] = my_pid(u, targetSpeed, heading, LOSAngle)
    % 参数的定义
    kp_u = 20;
    %kp_t = 1.8; %对应path
    kp_t = -5; %对应path.circle
    % 计算力F_x, F_y
    F = kp_u*(targetSpeed - u);
    if heading > pi/2 && LOSAngle < -pi/2
        LOSAngle = LOSAngle + 2*pi;
    elseif LOSAngle > pi/2 && heading < -pi/2
        LOSAngle = LOSAngle - 2*pi;
    end   
    delt = kp_t*(LOSAngle - heading); 
end




%% 运动学模型:输入为[u,v,r]、yaw, 输出为全局速度
function global_V = kinimatic_model(V,yaw)
    rotate_m = [cos(yaw), -sin(yaw), 0;
                sin(yaw), cos(yaw),  0;
                0,        0,         1];
    global_V = rotate_m * V;
end


%% 动力学模型:输入为力和力矩，[u,v,r],dt
function V = calculate_dynamic_mode(tao, V, dt)
    % 船体参数
    m = 250.19;
    Iz = 499.75;
    Xg = 0;
    X_u_dot = 0;
    Y_v_dot = 0;
    Y_r_dot = 0;
    N_v_dot = 0;
    N_r_dot = 0;
    X_u = -51.3;
    X_uu = -72.4;
    X_uuu = 0;
    Y_v = -40;
    Y_vv = 0;
    N_v = 0;
    N_vv = 0;
    Y_rv = 0;
    Y_r = 0;
    Y_vr = 0;
    Y_rr = 0;
    N_rv = 0;
    N_r = -400;
    N_vr = 0;
    N_rr = 0;



    % 计算动力学矩阵
    u = V(1);
    v = V(2);
    r = V(3);

    M = [m-X_u_dot   0   0;  
         0   m-Y_v_dot   m*Xg-Y_r_dot;
         0   m*Xg-N_v_dot Iz-N_r_dot];
    M_inv = inv(M);
    
    C = [0,     -m*r,     0;
         -m*r,    0,      0;
         0,       0,     0];
    
    D_L = -[X_u, 0,    0;
          0,   Y_v,  Y_r;
          0,   N_v,  N_r];
    D_NL = -[X_uu*abs(u) + X_uuu*u*u,   0,   0;
             0,    Y_vv*abs(v) + Y_rv*abs(r),    Y_vr*abs(v)+Y_rr*abs(r);
             0,        N_vv*abs(v)+N_rv*abs(r),             N_rr*abs(r) + N_vr*abs(v)];
    D = D_L + D_NL;
    
    V = V + M_inv *(tao - C*V - D*V) * dt;
    % 将速度限制在范围之内

    u_max = 2.0;
    r_max = 1.5;
    if V(1) > u_max
        V(1) = u_max;
    end
    if V(1) < -u_max
        V(1) = -u_max;
    end

    if V(3) > r_max
        V(3) = r_max;
    end
    if V(3) < -r_max
        V(3) = -r_max;
    end

end
