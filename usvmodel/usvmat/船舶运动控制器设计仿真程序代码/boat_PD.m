% 主函数，文件名为boat_PD
t_f = 600;   % 仿真事件设定
h   = 0.1;   % 采样时间
Kp = 1;      % 控制器P增益
Td = 10;     % 控制器D增益
 
% 状态x = [ u v r x y psi delta ]' 赋初值
x = zeros(7,1);   
 
N = round(t_f/h);               % 采样量
xout = zeros(N+1,length(x)+2);    %  输出变量赋初值

% 分支结构流程控制
for i=1:N+1,
    time = (i-1)*h;                   
    r   = x(3);
    psi = x(6);
    
    psi_ref = 5*(pi/180);            % 控制目标角度
    delta = -Kp*((psi-psi_ref)+Td*r);  % PD控制器
 
    % 调用M函数文件
    [xdot,U] = mariner(x,delta);       % 船舶模型
    
    % 存储数据以便后续调用
    xout(i,:) = [time,x',U]; 
    
    % 数值积分，欧拉算法   
    x = x + h*xdot
end

% 从存储的数据中给变量赋值
t     = xout(:,1);
u     = xout(:,2); 
v     = xout(:,3);          
r     = xout(:,4)*180/pi;   %  pi为Matlab特殊常量，表示圆周率
x     = xout(:,5);
y     = xout(:,6);
psi  	 = xout(:,7)*180/pi;
delta	 = xout(:,8)*180/pi;
U     = xout(:,9);
 
% 作图
% 如果要作多个图，用figure(i)，i = 1，2，3，…来实现
figure(1)
% 作完图之后，利用axis，xlabel等来丰富和定制图形的信息
plot(y,x),grid,axis('equal'),xlabel('East'),ylabel('North'),title('Ship position')
 
figure(2)
% 如果要求在一个图中作多个小图，用subplot来完成
subplot(221),plot(t,r),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(222),plot(t,U),xlabel('time (s)'),title('speed U (m/s)'),grid
subplot(223),plot(t,psi),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(224),plot(t,delta),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid
