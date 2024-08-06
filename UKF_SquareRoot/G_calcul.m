%Computation of the system matrix (F) and the system noise distribution
%matrix (G)
%x_dot=f(x,u,w)~=Fx+Gw
%x=[L l z Vn Ve Vd roll pitch yaw]T
%u=[fx fy fz wx wy wz]T;
%w=[dfx dfy dfz dwx dwy dwz]T
%C=CBtoN
%Reference : My thesis page 88-89
%            Titterton page 344            
function [ G ] = G_calcul( x )
    %x=[Lat;lon;d;Vn;Ve;Vd;fn;fe;fd]
    %C=body to navigation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SPhi      = sin(x(7));
    CPhi      = cos(x(7));
%     STheta    = sin(x(8));
%     CTheta    = cos(x(8));
%     SPsi      = sin(x(9));
%     CPsi      = sin(x(9));
    tgTheta   = tan(x(8));
    secTheta  = sec(x(8));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C11 =  cos(x(8))*cos(x(9));
    C12 = -cos(x(7))*sin(x(9)) + sin(x(7))*sin(x(8))*cos(x(9));
    C13 =  sin(x(7))*sin(x(9)) + cos(x(7))*sin(x(8))*cos(x(9));
    C21 =  cos(x(8))*sin(x(9));
    C22 =  cos(x(7))*cos(x(9)) + sin(x(7))*sin(x(8))*sin(x(9));
    C23 = -sin(x(7))*cos(x(9)) + cos(x(7))*sin(x(8))*sin(x(9));
    C31 = -sin(x(8));
    C32 =  sin(x(7))*cos(x(8));
    C33 =  cos(x(7))*cos(x(8));  
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    G11 = 0; G12 = 0; G13 = 0; G14 = 0; G15 = 0; G16 = 0;
%    G21 = 0; G22 = 0; G23 = 0; G24 = 0; G25 = 0; G26 = 0;
%    G31 = 0; G32 = 0; G33 = 0; G34 = 0; G35 = 0; G36 = 0;
%    
%    G41 = C11; G42 = C12; G43 = C13; G44 = 0; G45 = 0; G46 = 0;
%    G51 = C21; G52 = C22; G53 = C23; G54 = 0; G55 = 0; G56 = 0;
%    G61 = C31; G62 = C32; G63 = C33; G64 = 0; G65 = 0; G66 = 0;
   
   G71 = 0; G72 = 0; G73 = 0; G74 = 1; G75 = SPhi * tgTheta; G76 = CPhi * tgTheta;
   G81 = 0; G82 = 0; G83 = 0; G84 = 0; G85 = CPhi;           G86 = -SPhi;
   G91 = 0; G92 = 0; G93 = 0; G94 = 0; G95 = SPhi * secTheta; G96 = CPhi * secTheta;
   
   C = [C11 C12 C13
        C21 C22 C23
        C31 C32 C33];
   G_1 =[G74 G75 G76
         G84 G85 G86
         G94 G95 G96];
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    G=[G11 G12 G13 G14 G15 G16
%       G21 G22 G23 G24 G25 G26
%       G31 G32 G33 G34 G35 G36
%       G41 G42 G43 G44 G45 G46
%       G51 G52 G53 G54 G55 G56
%       G61 G62 G63 G64 G65 G66
%       G71 G72 G73 G74 G75 G76
%       G81 G82 G83 G84 G85 G86
%       G91 G92 G93 G94 G95 G96];%9x6
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    G=[zeros(3,12)
       C          , zeros(3) , C , zeros(3)
       zeros(3)   , G_1         , zeros(3)  , G_1   
       zeros(3,6) , eye(3)    , zeros(3)
       zeros(3,9) , eye(3)];%15x12
   
   
end