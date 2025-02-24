%Tunning of Qc(power spectral density) elements

function [ Simulation ] = Qc_setting( Simulation )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PSD_ax             = Simulation.Parameters_IMUNoisePSD.PSD_ax;
    PSD_ay             = Simulation.Parameters_IMUNoisePSD.PSD_ay;
    PSD_az             = Simulation.Parameters_IMUNoisePSD.PSD_az;
    TC_qax             = Simulation.Parameters_IMUNoisePSD.TC_qax;%TC:Tuning Coefficient
    TC_qay             = Simulation.Parameters_IMUNoisePSD.TC_qay;
    TC_qaz             = Simulation.Parameters_IMUNoisePSD.TC_qaz;
    
    PSD_wx             = Simulation.Parameters_IMUNoisePSD.PSD_wx;
    PSD_wy             = Simulation.Parameters_IMUNoisePSD.PSD_wy;
    PSD_wz             = Simulation.Parameters_IMUNoisePSD.PSD_wz;
    TC_qwx             = Simulation.Parameters_IMUNoisePSD.TC_qwx;
    TC_qwy             = Simulation.Parameters_IMUNoisePSD.TC_qwy;
    TC_qwz             = Simulation.Parameters_IMUNoisePSD.TC_qwz;
    
    PSD_Bax             = Simulation.Parameters_IMUNoisePSD.PSD_Bax;
    PSD_Bay             = Simulation.Parameters_IMUNoisePSD.PSD_Bay;
    PSD_Baz             = Simulation.Parameters_IMUNoisePSD.PSD_Baz;
    TC_qBax             = Simulation.Parameters_IMUNoisePSD.TC_qBax;%TC:Tuning Coefficient
    TC_qBay             = Simulation.Parameters_IMUNoisePSD.TC_qBay;
    TC_qBaz             = Simulation.Parameters_IMUNoisePSD.TC_qBaz;

    PSD_Bwx             = Simulation.Parameters_IMUNoisePSD.PSD_Bwx;
    PSD_Bwy             = Simulation.Parameters_IMUNoisePSD.PSD_Bwy;
    PSD_Bwz             = Simulation.Parameters_IMUNoisePSD.PSD_Bwz;
    TC_qBwx             = Simulation.Parameters_IMUNoisePSD.TC_qBwx;
    TC_qBwy             = Simulation.Parameters_IMUNoisePSD.TC_qBwy;
    TC_qBwz             = Simulation.Parameters_IMUNoisePSD.TC_qBwz;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a1= 1; a2=1;a3=1;a4=1;a5=1; %%%a1= 100; a2=100;a3=100;a4=10;a5=10;
    q_ax=(PSD_ax)*TC_qax*a1;
    q_ay=(PSD_ay)*TC_qay*a2;
    q_az=(PSD_az)*TC_qaz*a3;
            
    q_wx=(PSD_wx)*TC_qwx*a4;    
    q_wy=(PSD_wy)*TC_qwy*a4;    
    q_wz=(PSD_wz)*TC_qwz*a4;
    
    
     q_b_ax=(PSD_Bax)*TC_qBax*a1;
     q_b_ay=(PSD_Bay)*TC_qBay*a2;
     q_b_az=(PSD_Baz)*TC_qBaz*a3;
            
     q_b_wx=(PSD_Bwx)*TC_qBwx*a5;    
     q_b_wy=(PSD_Bwy)*TC_qBwy*a5;    
     q_b_wz=(PSD_Bwz)*TC_qBwz*a5;    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Simulation.Output.Kalman_mtx.Qc.q_ax=q_ax;  
    Simulation.Output.Kalman_mtx.Qc.q_ay=q_ay; 
    Simulation.Output.Kalman_mtx.Qc.q_az=q_az; 
    
    Simulation.Output.Kalman_mtx.Qc.q_wx=q_wx;
    Simulation.Output.Kalman_mtx.Qc.q_wy=q_wy;
    Simulation.Output.Kalman_mtx.Qc.q_wz=q_wz;
    
    Simulation.Output.Kalman_mtx.Qc.q_Bax=q_b_ax;  
    Simulation.Output.Kalman_mtx.Qc.q_Bay=q_b_ay; 
    Simulation.Output.Kalman_mtx.Qc.q_Baz=q_b_az; 
    
    Simulation.Output.Kalman_mtx.Qc.q_Bwx=q_b_wx;
    Simulation.Output.Kalman_mtx.Qc.q_Bwy=q_b_wy;
    Simulation.Output.Kalman_mtx.Qc.q_Bwz=q_b_wz;
    
end

