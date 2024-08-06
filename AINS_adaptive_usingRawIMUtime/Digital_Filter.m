%First Order Digital Filter
function [ Simulation ] = Digital_Filter( Simulation , S , dt , fc , I2 , SigType , ave_sample )

    if strcmp(SigType,'RollPitch')
        Simulation.Output.INS.FilteredSignal.filtered_RP(I2-ave_sample + 1,1:3)   = ...
        Simulation.Output.INS.FilteredSignal.filtered_RP(I2-ave_sample,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.INS.FilteredSignal.filtered_RP(I2-ave_sample,1:3));
    end
    if strcmp(SigType,'Accel')
        Simulation.Output.INS.FilteredSignal.filtered_Accel(I2 - ave_sample + 1,1:3)   = ...
        Simulation.Output.INS.FilteredSignal.filtered_Accel(I2-ave_sample,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.INS.FilteredSignal.filtered_Accel(I2-ave_sample,1:3));        
    end
    if strcmp(SigType,'Gyro')
        Simulation.Output.INS.FilteredSignal.filtered_Gyro(I2 - ave_sample + 1,1:3)   = ...
        Simulation.Output.INS.FilteredSignal.filtered_Gyro(I2-ave_sample,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.INS.FilteredSignal.filtered_Gyro(I2-ave_sample,1:3));        
    end    
   
end




