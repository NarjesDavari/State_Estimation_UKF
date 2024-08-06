%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Computation of the transition matrix of the dynamic model(A) and the error
%covariance matrix(Q) in Error State Kalman Filter (ESKF) approach
%E:the system matrix(22x22)
%F:the system noise distribution matrix(22x6)
%x: corrected position and velocity vectors and the accel vector 
%C:corrected transform matrix
%x=[Lat_c,lon_c,h_c,Vn_c,Ve_c,Vh_c,fn,fe,fh]:15x1(N=15)
%R0:mean radius of the earth
%e:the major eccentricity of the ellipsoid
%Omega:Earth's rate
%Rm:the meridian radius of curvature
%Rt:the transverse radius of curvature
function [Simulation]=GQ_calcul(x,Simulation,C,dt)

        [G]=G_calcul(x);
%           [F,G]=FG_calcul(x,C);
            
        Simulation.Output.Kalman_mtx.G=0;
        Simulation.Output.Kalman_mtx.G=G;
        %
        q_ax=Simulation.Output.Kalman_mtx.Qc.q_ax;  
        q_ay=Simulation.Output.Kalman_mtx.Qc.q_ay; 
        q_az=Simulation.Output.Kalman_mtx.Qc.q_az; 
    
        q_wx=Simulation.Output.Kalman_mtx.Qc.q_wx;
        q_wy=Simulation.Output.Kalman_mtx.Qc.q_wy;
        q_wz=Simulation.Output.Kalman_mtx.Qc.q_wz;
        
        q_b_ax=Simulation.Output.Kalman_mtx.Qc.q_Bax;
        q_b_ay=Simulation.Output.Kalman_mtx.Qc.q_Bay;
        q_b_az=Simulation.Output.Kalman_mtx.Qc.q_Baz;
        
        q_b_gx=Simulation.Output.Kalman_mtx.Qc.q_Bwx;
        q_b_gy=Simulation.Output.Kalman_mtx.Qc.q_Bwy;
        q_b_gz=Simulation.Output.Kalman_mtx.Qc.q_Bwz;

        %power spectral density of white noise process(W(t))(diagnal spectral density)
%         Qc=[q_ax  ,0     ,0     ,0     ,0     ,0      ;%LxL(6X6)
%             0     ,q_ay  ,0     ,0     ,0     ,0      ;
%             0     ,0     ,q_az  ,0     ,0     ,0      ;
%             0     ,0     ,0     ,q_wx  ,0     ,0      ;
%             0     ,0     ,0     ,0     ,q_wy  ,0      ;
%             0     ,0     ,0     ,0     ,0     ,q_wz  ];
 Qc=diag([q_ax,q_ay,q_az,q_wx,q_wy,q_wz,q_b_ax,q_b_ay,q_b_az,q_b_gx,q_b_gy,q_b_gz]);%%LxL(12X12)

%  dt=1/fs;
%         [A,Q] = lti_disc(F,G,Qc,dt);
%         Simulation.Output.Kalman_mtx.A=0;
%         Simulation.Output.Kalman_mtx.Q=0;
%         Simulation.Output.Kalman_mtx.A=A;
%         Simulation.Output.Kalman_mtx.Q=Q;
 
 %Q:The covariance of the discrete process
        Simulation.Output.Kalman_mtx.Q=0;
        Simulation.Output.Kalman_mtx.Q= G * Qc * G' * dt;
%           Q=0.5*(A*G*Qc*G'*A'*dt+G*Qc*G'*dt);
end  