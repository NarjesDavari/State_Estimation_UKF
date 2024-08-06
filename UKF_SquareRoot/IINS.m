%Integrated Navigation System using UKF
function [Simulation]=IINS(Simulation)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N                        = 1;%number of runs(simulations) in virtual mode
ave_sample               = Simulation.Init_Value.ave_sample;
calib_sample             = Simulation.Init_Value.calib_sample;
SF                       = Simulation.Init_Value.DVL_SF;
fs                       = Simulation.Init_Value.fs;
% gg                       = Simulation.Init_Value.gravity;
include_GPS              = Simulation.Auxiliary_Snsr.include_GPS;
include_depthmeter       = Simulation.Auxiliary_Snsr.include_depthmeter;
include_dvl              = Simulation.Auxiliary_Snsr.include_dvl;
include_heading          = Simulation.Auxiliary_Snsr.include_heading;
include_accelrollpitch   = Simulation.Auxiliary_Snsr.include_accelrollpitch;

fc_f_RP                  = Simulation.Parameters_Inclenometer.fc_f_RP;
fc_f_accel               = Simulation.Parameters_Accel.fc_f_accel;
fc_f_gyro                = Simulation.Parameters_Gyro.fc_f_gyro;
muu                      = Simulation.Parameters_Inclenometer.mu;

filtered_accel           = Simulation.Parameters_Accel.filtered_accel;
filtered_gyro            = Simulation.Parameters_Gyro.filtered_gyro;     

alpha = Simulation.Parameters_UKF.alpha;
beta = Simulation.Parameters_UKF.beta;
kappa = Simulation.Parameters_UKF.kappa;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt=1/fs;
             
C_DVL_IMU = eye(3);
 
%         C_DVL_IMU = eye(3);
        %DVL's Pitch bias:-4 deg
%         C_DVL_IMU                = [0.997541609568174,-0.048742414639543,-0.050347931389912;...
%                                     0.049293954121070,0.998736520358432,0.009770823375788;...
%                                     0.049808064279254,-0.012228651497115,0.998683942403860];           

       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       
           % Handles to dynamic model function,
           func_f  = @Navigation_f;
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                    [ Simulation ] = Qc_setting( Simulation );
                    [ Simulation ] = R_setting( Simulation );
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                tic 
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %Dlength:length of data(time step) 
                    Dlength=length(Simulation.Input.Measurements.IMU);      
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                    [ Simulation ] = Initialization( Simulation , fs , ave_sample ,alpha,beta,kappa );
                    WM=Simulation.Output.Kalman_mtx.UKF.WM;
                    WC=Simulation.Output.Kalman_mtx.UKF.WC;
                    W=Simulation.Output.Kalman_mtx.UKF.W;
                    c=Simulation.Output.Kalman_mtx.UKF.c;      
                    sigma = Simulation.Output.Kalman_mtx.UKF.newWeight;
                    W2 = Simulation.Output.Kalman_mtx.UKF.W2;
%                     WM2 = Simulation.Output.Kalman_mtx.UKF.WM2;
%                    WC2 = Simulation.Output.Kalman_mtx.UKF.WC2;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    Selection_Param1 ={include_GPS,include_depthmeter,include_dvl,include_heading,include_accelrollpitch ,muu };
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                     t_imu = zeros(Dlength,1);t_imu(1,1)=0;
                    
                    I=ave_sample+1;
                   while I <= Dlength
                       dt = Simulation.Input.Measurements.IMU(I,1)-Simulation.Input.Measurements.IMU(I-1,1);
                       I1=I;
                        while dt<=0
                            I=I+1;
                            dt = Simulation.Input.Measurements.IMU(I,1)-Simulation.Input.Measurements.IMU(I1,1);
%                            dt=0.05;   
                        end
                        I=I;
                            fs=1/dt;
                        IMU_Time   = num2str((round(Simulation.Input.Measurements.IMU(I,1)*1000))/1000); %%%%num2str(Simulation.Input.Measurements.IMU(I,1));
%                         IMU_Time   = Simulation.Input.Measurements.IMU(I,1);
                         if Simulation.Input.Measurements.GPS_Counter < length(Simulation.Input.Measurements.GPS)
                            GPS_Time   = num2str((round(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1)*1000))/1000);
%                             GPS_Time   = Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1);
                        end
                        if Simulation.Input.Measurements.Depth_Counter < length(Simulation.Input.Measurements.Depth)
                            depth_Time = num2str((round(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1)*1000))/1000);
%                             depth_Time = Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1);
                        end
                        if Simulation.Input.Measurements.DVL_Counter < length(Simulation.Input.Measurements.DVL)
                            DVL_Time   = num2str((round(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1)*1000))/1000);
%                             DVL_Time   = Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1);
                        end
%                         if Simulation.Input.Measurements.incln_Counter < length(Simulation.Input.Measurements.RollPitch)
%                             incln_Time = num2str(Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,1,1));
%                         end
                        if Simulation.Input.Measurements.hdng_Counter < length(Simulation.Input.Measurements.Heading)
                            hdng_Time  = num2str((round(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1)*1000))/1000);
%                             hdng_Time  = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1);
                        end 
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         Sensors_Time = {IMU_Time , GPS_Time , depth_Time , DVL_Time ,hdng_Time}; 
                        Selection_Param =[Sensors_Time,Selection_Param1 ];
                                     
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        [ Simulation ] = Unscented_KF( Simulation , I   , fc_f_RP , fc_f_accel , fc_f_gyro , dt , ...
                                                      filtered_accel , filtered_gyro , func_f , Selection_Param , C_DVL_IMU , WM , WC , W ,sigma,W2,alpha, c, SF, ave_sample );
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                          if I== calib_sample
% %                             [ Simulation ] = R_Moving( Simulation );
                             [  Simulation , include_GPS , include_depthmeter , include_dvl , include_heading , include_accelrollpitch ] = Moving_Selection(Simulation);
                             Selection_Param1 = {include_GPS , include_depthmeter , include_dvl , include_heading , include_accelrollpitch , muu };
                         end
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        if rem(I-ave_sample+1,10000)==0
                            I - ave_sample + 1
                        end    
                         t_imu(I,1)=str2num(IMU_Time);
                       
                        I=I+1;
                    end
                    Simulation.Output.UKF.X_i(:,:)=Simulation.Output.UKF.X(:,1:3);

                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                a=toc;
                a_min=fix(a)/60;
                a_sec=(a_min-fix(a_min))*60;
                fprintf('Elapsed time : %f min ',fix(a_min));
                fprintf('%f sec\n ',a_sec);      
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%                 if select_func.conversion
                    [Simulation]=conversion(Simulation,ave_sample );
%                 end         
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%                 if select_func.distance_cacul
                    [ Simulation ] = distance_cacul( Simulation );
%                 end
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%                 if select_func.Navigate_Error
                    [Simulation]=Navigate_Error(Simulation,ave_sample );
%                 end
                %%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                fprintf('Travelled time : %.2f min\n',Simulation.Output.UKF.Pos_Error.travelled_time/60);
                fprintf('Travelled distance : %.2f meter\n',Simulation.Output.UKF.Pos_Error.travelled_distance);
                fprintf('Travelled distance in x direction : %.2f meter\n',Simulation.Output.UKF.Pos_Error.travelled_distancex);
                fprintf('Travelled distance in y direction : %.2f meter\n',Simulation.Output.UKF.Pos_Error.travelled_distancey);
                fprintf('Travelled distance in z direction : %.2f meter\n',Simulation.Output.UKF.Pos_Error.travelled_distancez);
                
                fprintf('RMSE : %.3f meter\n',Simulation.Output.UKF.Pos_Error.RMSE);
                fprintf('Relative RMSE : %.3f %%\n',Simulation.Output.UKF.Pos_Error.Relative_RMSE);
                
                fprintf('RMSE in x direction : %.3f meter\n',Simulation.Output.UKF.Pos_Error.RMSEx);
                fprintf('RMSE in y direction : %.3f meter\n',Simulation.Output.UKF.Pos_Error.RMSEy);
                fprintf('RMSE in z direction : %.3f meter\n',Simulation.Output.UKF.Pos_Error.RMSEz);

                fprintf('Relative RMSE in x direction : %.3f %%\n',Simulation.Output.UKF.Pos_Error.Relative_RMSEx);
                fprintf('Relative RMSE in y direction : %.3f %%\n',Simulation.Output.UKF.Pos_Error.Relative_RMSEy);
                fprintf('Relative RMSE in z direction : %.3f %%\n',Simulation.Output.UKF.Pos_Error.Relative_RMSEz);
                
                fprintf('Absolute error in end of path : %.3f meter\n',Simulation.Output.UKF.Pos_Error.absolute_error_end);
                fprintf('Relative error in end of path : %.3f %%\n',Simulation.Output.UKF.Pos_Error.relative_error_end);           
                %%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&                
        
       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

end

   
