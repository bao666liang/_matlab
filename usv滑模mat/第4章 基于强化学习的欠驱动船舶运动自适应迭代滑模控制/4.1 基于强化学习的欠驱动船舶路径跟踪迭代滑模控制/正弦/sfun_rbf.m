%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%源自: 沈智鹏 著《船舶运动自适应滑模控制》 2019年科学出版社
%%下载地址www.shenbert.cn/book/shipmotionASMC.html
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts] = sfun_rbf(t,x,u,flag)

switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
     case 1,
    sys=mdlDerivatives(t,x,uss);
%   case 2,
%     sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case {2,4,9}
    sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
 

 function [sys,x0,str,ts]=mdlInitializeSizes
 
 sizes = simsizes;
 sizes.NumContStates  = 0;
 sizes.NumDiscStates  = 1;
 sizes.NumOutputs     = 2;
 sizes.NumInputs      = 3;  % 抖振，实际位置的x，y。;
 sizes.DirFeedthrough = 1;
 sizes.NumSampleTimes = 1;   
 x0=[15];
 str = [];
 ts  = [0 0];
 sys = simsizes(sizes);

 
function sys=mdlDerivatives(t,x,uss)

sys=[];

function sys=mdlOutputs(t,x,u)
global xt yt ye ymax c h b dw
xt=u(2); yt=u(3);
ymax=400;
e=yt-200*sin(3.14*xt/2500);
ye=(e)/ymax;
c=abs(ye);
h=[1 1 1 1 1 1 1 1 1 1];
w=[1 1 1 1 1 1 1 1 1 1];
cj=[-1 -0.75 -0.5  -0.3 0 0.2 0.3 0.5  0.75 1];
b=2;
for i=1:1:10;
    h(i)=exp(-norm(c-cj(i))^2/2*b^2);
end

dw=0.1*(u(1)-2)/2*h;
w=w+dw;
sys(1)=h*w';
sys(2)=ye*ymax;