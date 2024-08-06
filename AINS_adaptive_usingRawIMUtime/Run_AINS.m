function [ Simulation ] = Run_AINS( )

%       load('Real_Measuremet_test1_22th_2', 'Real_Measurement')
        load('Real_Measurement_IMU_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
        load('Parameter_stationary_8Jun', 'Simulation') 
%         load('Parameter_stationary_test1_22th', 'Simulation')

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
%         Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        Simulation = IINS_2(Simulation,coeff_MA,include_MA);
end
