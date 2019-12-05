clear all;
% Parameter


% MAX_SPEED =50.0 / 3.6;  % 最大速度 [m/s]
% MAX_ACCEL = 2.0;  % 最大加速度[m/s s]
% MAX_CURVATURE = 1.0 ; % 最大曲率 [1/m]
% MAX_ROAD_WIDTH  =7.0;  % 最大道路宽度 [m]
% D_ROAD_W = 1.0 ; % 道路宽度采样间隔 [m]
% DT = 0.2;  % Delta T [s]
% MAXT = 5.0 ; % 最大预测时间 [s]
% MINT = 4.0;  % 最小预测时间 [s]
% TARGET_SPEED = 30.0 / 3.6 ; % 目标速度（即纵向的速度保持） [m/s]
% D_T_S = 5.0 / 3.6;  % 目标速度采样间隔 [m/s]
% N_S_SAMPLE = 1;  % sampling number of target speed
% ROBOT_RADIUS = 2.0 ; % robot radius [m]
%D{1..}=d;
%D{2..}=d_d;
%D{3..}=d_dd;
%S{1..}=s;
%S{2..}=s_d; 
 N=600;
 T=0.1;

wx = [0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
wy = [0.0, -4.0, 1.0, 6.5, 8.0, 10.0, 6.0];
xx = 0:T:(N-1)*T;
[~,len] = size(xx);
yy = 2.*ones(1,len);

%yy = spline(wx,wy,xx); %generate spline
% xx = traject(:,2)';
% yy = traject(:,3)';

[csp,raw,RefrenceArray] = xy2sd(xx,yy); % coordinate tramsform ,xy to sd

%start
s0 =0 ; % 当前所在的位置
c_speed = 10.0 / 3.6 ; % 当前车速 [m/s]
c_d = -3;  % 当前的d方向位置 [m]
c_d_d= 1.0 ; % 当前横向速度 [m/s]
c_d_dd= 1.0;  % 当前横向加速度 [m/s2]

%end
endx=xx(end);

endy=yy(end);





% main loop
while true
    
    if (abs(s0-csp(end))<1)
        disp("Arrive!");
        break;
    end
    %clac path
    [csp,CF,D,S] = calc_frenet_path(csp,c_speed,c_d,c_d_d,c_d_dd,s0);   
    [~,row] = min(CF);

    %update coefficient
    c_d    = D{1,row}(2);
    c_d_d  = D{2,row}(2);
    c_d_dd = D{3,row}(2);
    s0     = S{1,row}(2);
    c_speed= S{2,row}(2);
    [nx,ny] = sd2xy(s0,c_d,RefrenceArray);

    %plot
    figure(1)
    plot(xx,yy);
    hold on;
    plot(nx,ny,'o');
    hold on;
end





 %% 
function [csp,CF,D,S] = calc_frenet_path(csp,c_speed,c_d,c_d_d,c_d_dd,s0)
% Parameter
    MAX_SPEED =30.0 / 3.6;  % 最大速度 [m/s]
    MAX_ACCEL = 5.0;  % 最大加速度[m/s2]
    MAX_CURVATURE = 1.0 ; % 最大曲率 [1/m]
    MAX_ROAD_WIDTH  =7.0;  % 最大道路宽度 [m]
    D_ROAD_W = 0.5 ; % 道路宽度采样间隔 [m]
    DT = 0.2;  % Delta T [s]
    MAXT = 5.0 ; % 最大预测时间 [s]
    MINT = 4.0;  % 最小预测时间 [s]
    TARGET_SPEED = 20.0 / 3.6 ; % 目标速度（即纵向的速度保持） [m/s]
    D_T_S = 5.0 / 3.6;  % 目标速度采样间隔 [m/s]
    N_S_SAMPLE = 1;  % sampling number of target speed
    ROBOT_RADIUS = 2.0 ; % robot radius [m]

    KJ = 0.1;
    KT = 0.1;
    KD = 1.0;
    KLAT = 1.0;
    KLON = 1.0;

    slat_qp=[];
    s=[];
    s_d=[];
    s_dd=[];
    s_ddd=[];
    d=[];
    d_d=[];
    d_dd=[];
    d_ddd=[];
    CF=[];
    D=cell(3,300);
    S=cell(1,300);
    csp=csp;
    i =1;
    for di = -MAX_ROAD_WIDTH:D_ROAD_W:MAX_ROAD_WIDTH   %道路采样
        for Ti = MINT:DT:MAXT      %时间采样
           lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti) ;
           slat_qp = [slat_qp;lat_qp];
              t = [0:DT:Ti];
                [d,d_d,d_dd,d_ddd] = calc_5diff(lat_qp,t);
                 for tv=TARGET_SPEED - D_T_S * N_S_SAMPLE : D_T_S :  TARGET_SPEED + D_T_S * N_S_SAMPLE
                   lon_qp =  quartic_polynomial(s0,c_speed,0,tv,0,Ti);
                    tt = [0 :DT : Ti];
                      [s,s_d,s_dd,s_ddd] = calc_4diff(lon_qp,tt);
                      D{1,i}=d;D{2,i}=d_d;D{3,i}=d_dd;S{1,i}=s; S{2,i}=s_d;
                      i=i+1;
                      Jp = sum(d_ddd.^2);
                      Js = sum(s_ddd.^2);
                      ds= (TARGET_SPEED - s(end)).^2;
                      cd = KJ * Jp + KT * Ti + KD * d(end).^2;
                      cv = KJ * Js + KT * Ti + KD * ds;
                      cf = KLAT * cd + KLON * cv;
                      CF=[CF;cf];
                 end  
                
        end
    end
    i=1;
    for i=1:length(CF)
        spdcheck = find(D{2,i}(1,:)>MAX_SPEED, 1);
        acclcheck = find(D{3,i}(1,:)>MAX_ACCEL, 1);
        if ( ~isempty(spdcheck)|| ~isempty(acclcheck))
            D{1,i}=[];D{2,i}=[];D{3,i}=[];S{1,i}=[];
            CF(i)=inf;
        end
    end
    %check path

end

%%

function a= quintic_polynomial(xs, vxs, axs, xe, vxe, axe, T)
% 计算五次多项式系数
    a0 = xs;
    a1 = vxs;
    a2 = axs / 2.0;
    A = [T^3, T^4, T^5;3 * T^2, 4 * T^3, 5 * T^4;6 * T, 12 * T^2, 20 * T^3];
    b = [xe - a0 - a1*T-a2*T^2;vxe - a1 - 2*a2*T;axe - 2 * a2];
    res=A\b;
    a3=res(1);
    a4=res(2);
    a5=res(3);
    a=[a0,a1,a2,res'];  
end


function c = quartic_polynomial(xs,vxs,axs,vxe,axe,T)
% 计算4次多项式系数
    a0 = xs;
    a1 = vxs;
    a2 = axs / 2.0;
    A = [3 * T ^2, 4 * T ^3;6 * T, 12*T^2];
    b = [vxe - a1 - 2 * a2 * T;axe - 2*a2];
    x = A\b;
    a3 = x(1);
    a4 = x(2);
    c=[a0,a1,a2,x'];  
end

function [x,xt,xtt,xttt] = calc_5diff(a,t)
     x = a(1) + a(2) .*t + a(3) .* t.^2 + a(4) .* t.^3 + a(5) .* t.^4 + a(6) .* t .^ 5 ;
     xt = a(2) + 2 .* a(3) .* t + 3 .* a(4) .* t .^ 2 + 4 .* a(5) .* t .^ 3 + 5 .* a(6) .* t .^ 4   ;    
     xtt = 2 .* a(3) + 6 .* a(4) .* t + 12 .* a(5) .* t .^ 2 + 20 .* a(6) .* t .^ 3   ;     
     xttt = 6 .* a(4) + 24 .* a(5) .* t + 60 .* a(6) .* t .^ 2 ;
end

function [y,yt,ytt,yttt] = calc_4diff(a,t)
    y = a(1) + a(2) .* t + a(3) .* t.^2 + a(4) .* t.^3 + a(5) .* t.^4;
    yt = a(2) + 2 .* a(3) .* t + 3 .* a(4) .* t .^ 2 + 4 .* a(5) .* t .^ 3;
    ytt = 2 .* a(3) + 6 .* a(4) .* t + 12 .* a(5) .* t .^ 2;
    yttt = 6 .* a(4) + 24 .* a(5) .* t;
end

%%
function [refs,raw,RefrenceArray] = xy2sd(x,y)
    %the input and output is array
    %x[.....................]
    %y[.....................]
    dx=[0,diff(x)];
    dy=[0,diff(y)];
    %draw = [0,atan2(dy,dx)];
    ds = [(dx.^2+dy.^2).^0.5];
    s= cumsum(ds);
    dr = atan2(dy,dx);
    raw=[0,atan2(cumsum(dy),cumsum(dx))];
    XYsr=[x;y;s;dr];

    refs = s;
    RefrenceArray = XYsr;
end

%%
function [x,y] = sd2xy(s,d,refarray)
    refx=refarray(1,:);
    refy=refarray(2,:);
    refs=refarray(3,:);
    refraw=refarray(4,:);
    for turns = 1:length(s)
        i=find(refs>s(turns));
        if isempty(i)
            NextRefPoint=length(refx);
            PreRefPoint=NextRefPoint;
            break;
        end
        NextRefPoint(turns)=i(1);
    end
    PreRefPoint =NextRefPoint-1;
    PreRefPoint(PreRefPoint<1)=1;

    Heading =refraw(NextRefPoint);
    fs = s - refs(PreRefPoint);
    fx = refx(PreRefPoint) + fs.*cos(Heading);
    fy = refy(PreRefPoint) + fs.*sin(Heading);
    PerpendicularHeading=Heading - pi/2;

    x= fx + d.*cos(PerpendicularHeading);
    y= fy - d.*sin(PerpendicularHeading);
end

