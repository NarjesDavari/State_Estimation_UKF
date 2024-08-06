function [ Simulation , gl, ave_fb , ave_W ] = Coarse_Alignment( Simulation , ave_sample )    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Dlength = length(Simulation.Input.Measurements.IMU);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fb= zeros(ave_sample,3);
    W = zeros(ave_sample,3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Initial Position
    Lat = zeros(ave_sample,1);
    lon = zeros(ave_sample,1);
%     Alt = zeros(ave_sample,1);
    Z   = zeros(ave_sample,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CA.Roll  = zeros(ave_sample,1);
    CA.Pitch = zeros(ave_sample,1);
    CA.Yaw   = zeros(ave_sample,1);
        %position,velocity,euler angles and accel computed by SDINS in navigation frame
    Simulation.Output.UKF.X      = zeros(Dlength-ave_sample+1 ,15);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for I=1:ave_sample% Five miutes 
I=1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         IMU_Time   = num2str(Simulation.Input.Measurements.IMU(I,1,1));
%         if Simulation.Input.Measurements.GPS_Counter < length(Simulation.Input.Measurements.GPS)
%             GPS_Time   = num2str(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1));
%         end
%         if Simulation.Input.Measurements.Depth_Counter < length(Simulation.Input.Measurements.Depth)
%             depth_Time = num2str(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1));
%         end  
%         if Simulation.Input.Measurements.DVL_Counter < length(Simulation.Input.Measurements.DVL)
%             DVL_Time   = num2str(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1));
%         end
%         if Simulation.Input.Measurements.hdng_Counter < length(Simulation.Input.Measurements.Heading)
%             hdng_Time  = num2str(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1));                            
%         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%         if strcmp(IMU_Time,GPS_Time)
            Lat(I,1) = Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,2)*(pi/180);
            lon(I,1) = Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,3)*(pi/180);

            Simulation.Input.Measurements.GPS_Counter   = Simulation.Input.Measurements.GPS_Counter + 1;
%         end
%          if strcmp(IMU_Time,depth_Time)
%          if strcmp(IMU_Time,DVL_Time)
            Z(I,1)   = Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,2);
%             Z(I,1)   = 0;
            
            Simulation.Input.Measurements.Depth_Counter = Simulation.Input.Measurements.Depth_Counter + 1;
%         end
%         if strcmp(IMU_Time,DVL_Time)
    %Initial velocity
            V1b = Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,2:4);
            Simulation.Input.Measurements.DVL_Counter   = Simulation.Input.Measurements.DVL_Counter + 1;
%         end
%         if strcmp(IMU_Time,hdng_Time)
            if Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180 > pi
                CA.Yaw(I) = (Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180) - 2*pi;
            elseif Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180 < -pi
                CA.Yaw(I) = (Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180) + 2*pi;
            else
                CA.Yaw(I) = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180;
            end
            Simulation.Input.Measurements.hdng_Counter  = Simulation.Input.Measurements.hdng_Counter + 1;
%         end
        %Stationary mode                
        fb(I,:) = Simulation.Input.Measurements.IMU(I,2:4);
        W(I,:) = Simulation.Input.Measurements.IMU(I,5:7);
%     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Initial Position (Latitude, longitude, altitude)
    Simulation.Output.UKF.X(1,1) = mean (nonzeros(Lat));
    Simulation.Output.UKF.X(1,2) = mean (nonzeros(lon));
    if ~isempty(nonzeros(Z))
        Simulation.Output.UKF.X(1,3) = mean (nonzeros(Z));
    else
        Simulation.Output.UKF.X(1,3) = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [gl,g0] = Gravity( [Simulation.Output.UKF.X(1,1), Simulation.Output.UKF.X(1,2),Simulation.Output.UKF.X(1,3)] );
    
    CA.Pitch = asin((fb(:,1))./gl(3));
    CA.Roll  = -asin((fb(:,2))./(gl(3).*cos(CA.Pitch)));    
    %Initial roll, pitch, heading
    if ~isempty(nonzeros(CA.Roll))
        Simulation.Output.UKF.X(1,7) = mean (CA.Roll);
    else
        Simulation.Output.UKF.X(1,7) = 0;
    end
    if ~isempty(nonzeros(CA.Pitch))
        Simulation.Output.UKF.X(1,8) = mean (CA.Pitch);
    else
        Simulation.Output.UKF.X(1,8) = 0;
    end
    if ~isempty(nonzeros(CA.Yaw))
        Simulation.Output.UKF.X(1,9) = mean (nonzeros(CA.Yaw)); %%+ Simulation.Parameters_Misalignment.IMU_phins(3)*pi/180;
    else
        Simulation.Output.UKF.X(1,9) = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Euler1=Simulation.Output.UKF.X(1,7:9);
Cbn   = InCBN(Euler1);
V1n=(Cbn*V1b')';
Simulation.Output.UKF.X(1,4:6) =V1n;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %IMU
    if ~isempty(nonzeros(fb(:,1)))
        ave_fx = mean (fb(:,1));
    else
        ave_fx = 0;
    end 
    if ~isempty(nonzeros(fb(:,2)))
        ave_fy = mean (fb(:,2));
    else
        ave_fy = 0;
    end
    if ~isempty(nonzeros(fb(:,3)))
        ave_fz = mean (fb(:,3));
    else
        ave_fz = 0;
    end 
    ave_fb =[ave_fx ave_fy ave_fz];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(nonzeros(W(:,1)))
        ave_Wx = mean (W(:,1));
    else
        ave_Wx = 0;
    end 
    if ~isempty(nonzeros(W(:,2)))
        ave_Wy = mean (W(:,2));
    else
        ave_Wy = 0;
    end
    if ~isempty(nonzeros(W(:,3)))
        ave_Wz = mean (W(:,3));
    else
        ave_Wz = 0;
    end    
    ave_W = [ave_Wx ave_Wy ave_Wz];  
    end

