
function [flag_fusion,I_new,Sensors_Time,flag_fusion_param]=scheduling_fusion_2 (Simulation,I,Selection_Param)

include_GPS =Selection_Param{1};
include_depthmeter=Selection_Param{2};
include_dvl=Selection_Param{3};
include_heading=Selection_Param{4};
flag_fusion_GPS=0;flag_fusion_depth=0;flag_fusion_DVL=0;flag_fusion_hdng=0;

%  IMU_Time   = num2str(Simulation.Input.Measurements.IMU(I,1));
        IMU_Time_1   = Simulation.Input.Measurements.IMU(I,1);
        IMU_Time_2   = Simulation.Input.Measurements.IMU(I+1,1);
         if Simulation.Input.Measurements.GPS_Counter <= length(Simulation.Input.Measurements.GPS)
             %  GPS_Time   = num2str(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1));
             GPS_Time   = Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1);
             if include_GPS && (GPS_Time-IMU_Time_2)<0
%                  if abs(GPS_Time-IMU_Time_2)<abs(GPS_Time-IMU_Time_1)
%                      I_new=I+1;
%                  else
                     I_new=I;
%                  end
                 flag_fusion_GPS=1;
             end
         else
             GPS_Time   = Simulation.Input.Measurements.GPS(end,1);
        end
        if Simulation.Input.Measurements.Depth_Counter <= length(Simulation.Input.Measurements.Depth)
%                             depth_Time = num2str(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1));
            depth_Time = Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1);
            if include_depthmeter && (depth_Time-IMU_Time_2)<0
%                 if abs(depth_Time-IMU_Time_2)<abs(depth_Time-IMU_Time_1)
%                      I_new=I+1;
%                  else
                     I_new=I;
%                  end
                flag_fusion_depth=1;
            end
        else
             depth_Time = Simulation.Input.Measurements.Depth(end,1);
        end
        if Simulation.Input.Measurements.DVL_Counter <= length(Simulation.Input.Measurements.DVL)
%                             DVL_Time   = num2str(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1));
            DVL_Time   = Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1);
            if include_dvl && (DVL_Time-IMU_Time_2)<0
%                 if abs(DVL_Time-IMU_Time_2)<abs(DVL_Time-IMU_Time_1)
%                      I_new=I+1;
%                  else
                     I_new=I;
%                  end
                flag_fusion_DVL=1;
            end
        else
             DVL_Time   = Simulation.Input.Measurements.DVL(end,1);
        end
%                         if Simulation.Input.Measurements.incln_Counter < length(Simulation.Input.Measurements.RollPitch)
%                             incln_Time = num2str(Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,1,1));
%                         end
        if Simulation.Input.Measurements.hdng_Counter <= length(Simulation.Input.Measurements.Heading)
%                             hdng_Time  = num2str(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1));
            hdng_Time  = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1);
            if include_heading && (hdng_Time-IMU_Time_2)<0
%                 if abs(hdng_Time-IMU_Time_2)<abs(hdng_Time-IMU_Time_1)
%                     I_new=I+1;
%                  else
                     I_new=I;
%                  end
                flag_fusion_hdng=1;
            end
        else
            hdng_Time  = Simulation.Input.Measurements.Heading(end,1);
        end 
             if  flag_fusion_GPS==1  || flag_fusion_depth==1 ||  flag_fusion_DVL==1 ||  flag_fusion_hdng==1 
                IMU_Time=Simulation.Input.Measurements.IMU(I_new,1);
                flag_fusion=1;
             else
                 flag_fusion=0;
                   IMU_Time=Simulation.Input.Measurements.IMU(I,1);
                   I_new=I;
             end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                        
            Sensors_Time={IMU_Time,GPS_Time,depth_Time,DVL_Time,hdng_Time};
            flag_fusion_param={flag_fusion_GPS,flag_fusion_depth,flag_fusion_DVL,flag_fusion_hdng};
                                        
                                        
                                        