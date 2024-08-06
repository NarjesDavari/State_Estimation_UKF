% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan( )

%       load('Real_Measurement_simulateddata_LSTS_3', 'Real_Measurement')
        load('Real_Measurement_testReal_LSTS_rawdata', 'Real_Measurement')
        load('Parameter_stationary_8Jun', 'Simulation') 

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
        R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
        Qa=[1e-5,1e-6,1e-7,1e-8,1e-9];
        Qg=[1e-6,1e-7,1e-8,1e-9,1e-10,1e-11];
        Qba=[1e-6,1e-7,1e-8,1e-9,1e-10];
        Qbg=[1e-10,1e-11,1e-12,1e-13,1e-14,1e-15];
    
%     alpha=[1,10,100,1000];
%     rho =[0.5,0.7,0.9,1];
%         Dlen=+length(Qg)+length(Qba)+length(Qbg);
        k=1;
        for i=1:length(R_vd)
            for j=1:length(Qa)
                Simulation.Parameters_IMUNoisePSD.PSD_ax=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_ay=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_az=Qa(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_ax=Simulation.Parameters_IMUNoisePSD.PSD_ax;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_ay=Simulation.Parameters_IMUNoisePSD.PSD_ay;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_az=Simulation.Parameters_IMUNoisePSD.PSD_az;
                
                Simulation.Parameters_IMUNoisePSD.PSD_wx=Qg(j);
                Simulation.Parameters_IMUNoisePSD.PSD_wy=Qg(j);
                Simulation.Parameters_IMUNoisePSD.PSD_wz=Qg(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wx=Simulation.Parameters_IMUNoisePSD.PSD_wx;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wy=Simulation.Parameters_IMUNoisePSD.PSD_wy;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wz=Simulation.Parameters_IMUNoisePSD.PSD_wz;
                
                Simulation.Parameters_IMUNoisePSD.PSD_Bax=Qba(j);
                Simulation.Parameters_IMUNoisePSD.PSD_Bay=Qba(j);
                Simulation.Parameters_IMUNoisePSD.PSD_Baz=Qba(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bax=Simulation.Parameters_IMUNoisePSD.PSD_Bax;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bay=Simulation.Parameters_IMUNoisePSD.PSD_Bay;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Baz=Simulation.Parameters_IMUNoisePSD.PSD_Baz;
                
                Simulation.Parameters_IMUNoisePSD.PSD_Bwx=Qbg(j);
                Simulation.Parameters_IMUNoisePSD.PSD_Bwy=Qbg(j);
                Simulation.Parameters_IMUNoisePSD.PSD_Bwz=Qbg(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwx=Simulation.Parameters_IMUNoisePSD.PSD_Bwx;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwy=Simulation.Parameters_IMUNoisePSD.PSD_Bwy;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwz=Simulation.Parameters_IMUNoisePSD.PSD_Bwz;
                
                Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vx=R_vd(i);
                Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vy=R_vd(i);
                Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vz=R_vd(i);
                Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_alt=R_vd(i);
                
                Simulation.Parameters_AuxSnsrNoiseVar.var_vx=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vx;
                Simulation.Parameters_AuxSnsrNoiseVar.var_vy=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vy;
                Simulation.Parameters_AuxSnsrNoiseVar.var_vz=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vz; 
                
%                  Simulation.parameter_VB.alpha=alpha(i)*ones(9,1);
%                  Simulation.Output.Kalman_mtx.rho_VB=rho(i);
                
             Simulation = IINS(Simulation,coeff_MA,include_MA);
             Q=[Qa(j);Qg(j);Qba(j);Qbg(j)];
             Result.Q = Q; 
             Result.R = R_vd(i);
             Result.pos_m=Simulation.Output.ESKF.Pos_m;
             Result.error=Simulation.Output.ESKF.Pos_Error;
             save(['Result_' num2str(k) '.mat'],'Result')

%              filename= 'testdata.xlsx';
%              xlswrite(filename,A)

             k=k+1;
            end
        end
end
