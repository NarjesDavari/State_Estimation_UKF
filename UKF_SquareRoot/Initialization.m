%Initialization of the orientation, transformation matrix, vvelocity,
%position, acceleration, and so on.
function [ Simulation ] = Initialization( Simulation , fs ,ave_sample,alpha,beta,kappa)
            global Updt_Cntr;
            global Cbn_det;
            Updt_Cntr = 0;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters
    R = 6378137;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            Dlength = length(Simulation.Input.Measurements.IMU);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.Measurements.GPS_Counter   = 1; %GPS Counter
            Simulation.Input.Measurements.DVL_Counter   = 1; %DVL Counter%Path4:1658
            Simulation.Input.Measurements.Depth_Counter = 1; %Depth Counter
            Simulation.Input.Measurements.hdng_Counter  = 1; %Heading Counter
            Simulation.Input.Measurements.incln_Counter = 1; %Inclinometer Counter
            Simulation.Input.Measurements.accelrollpitch_Counter=1;
            
            Simulation.Input.Measurements.GPS_Counter_fusion=0;
            Simulation.Input.Measurements.Depth_Counter_fusion=0;
            Simulation.Input.Measurements.DVL_Counter_fusion =0;
            Simulation.Input.Measurements.hdng_Counter_fusion=0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %states                
            X1                            = (zeros(1,15))';%column vector
            Simulation.Output.UKF.X      = zeros(Dlength-ave_sample +1 ,size(X1,1));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Cbn_det   = zeros(Dlength,1);  
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
            %Initial acceleration and angular velocity
%             Bias1g =Simulation.Input.InitialParam.Initialbiasg;
            W1ib_b(1)=Simulation.Input.Measurements.IMU(1,5,1);    
            W1ib_b(2)=Simulation.Input.Measurements.IMU(1,6,1);    
            W1ib_b(3)=Simulation.Input.Measurements.IMU(1,7,1);
                   
%             Bias1a =Simulation.Input.InitialParam.Initialbiasa; 
            f1b(1)=Simulation.Input.Measurements.IMU(1,2,1);
            f1b(2)=Simulation.Input.Measurements.IMU(1,3,1);
            f1b(3)=Simulation.Input.Measurements.IMU(1,4,1);                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%Alignment:computation of initial transformation matrix
            [ Simulation , gl , ave_fb , ave_W ] = Coarse_Alignment( Simulation,ave_sample  );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Euler1                                      = Simulation.Output.UKF.X(1,7:9);
            Simulation.Input.InitialParam.InitialEuler  = Euler1;
            Cbn_INS                                     = InCBN(Euler1);
            Simulation.Output.UKF.Cbn                   = zeros(3,3,Dlength-ave_sample +1 );
            Simulation.Output.INS.Cbn(:,:,1)            = Cbn_INS;
            Simulation.Output.UKF.Cbn_corrected        = zeros(3,3,Dlength-ave_sample +1 );
            Simulation.Output.UKF.Cbn_corrected(:,:,1) = Cbn_INS;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.InitialParam.InitialVelocity            = Simulation.Output.UKF.X(1,4:6);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.UKF.FilteredSignal.filtered_RP         = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.UKF.FilteredSignal.filtered_RP(1,:)    = ave_fb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.UKF.FilteredSignal.filtered_Accel      = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.UKF.FilteredSignal.filtered_Accel(1,:) = ave_fb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.UKF.FilteredSignal.filtered_Gyro       = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.UKF.FilteredSignal.filtered_Gyro(1,:)  = ave_W;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Alignment.norm.Accel_norm    = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.norm.Gyro_norm     = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.norm.Accel_norm(1) = abs(norm(ave_fb) -  norm(gl));
            Simulation.Output.Alignment.norm.Gyro_norm (1) = norm(ave_W);      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Alignment.phi              = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.theta            = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.ave_phi          = zeros(fix(Dlength-ave_sample +1 /1),1);
            Simulation.Output.Alignment.ave_theta        = zeros(fix(Dlength-ave_sample +1 /1),1);            
%             Simulation.Output.Alignment.time         = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.RP_counter       = 1;
            Simulation.Output.Alignment.RP_counter2      = 0;
            Simulation.Output.Alignment.ave_RP_counter   = 1;
            Simulation.Output.Alignment.RP_active        = 0;
            
            Simulation.Output.Alignment.theta(1,1)       = Simulation.Output.UKF.X(1,8);
            Simulation.Output.Alignment.phi(1,1)         = Simulation.Output.UKF.X(1,7);
            Simulation.Output.Alignment.ave_theta(1,1)   = Simulation.Output.UKF.X(1,8);
            Simulation.Output.Alignment.ave_phi(1,1)     = Simulation.Output.UKF.X(1,7);           
%             Simulation.Output.Alignment.time         = Simulation.Input.Measurements.IMU(1,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            W_Coriolis = Coriolis_correction( Simulation.Output.UKF.X(1,1:6)  );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.fnn      = zeros(Dlength-ave_sample ,3);
            f1nn                           = (Cbn_INS*ave_fb')';%%convert initial Accelerometer frome  body to navigation
            %Simulation.Output.INS.fnn(1,:) = f1nn;
            f1n                            = f1nn - cross(W_Coriolis,Simulation.Output.UKF.X(1,4:6)) + gl;
            Simulation.Output.INS.fn       = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.INS.fn(1,:)  = f1n;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                   
            %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.Q_diag = zeros(Dlength-ave_sample ,size(X1,1));
            Simulation.Output.Kalman_mtx.F_diag = zeros(Dlength-ave_sample ,size(X1,1));
            Simulation.Output.Kalman_mtx.G_diag = zeros(Dlength-ave_sample ,24);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.adaptive_innovation=0;
            Simulation.Output.Kalman_mtx.adaptive_residual=0;
            Simulation.Output.Kalman_mtx.update_counter =0;
            Simulation.Output.Kalman_mtx.Number_adaptive=0;                                                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Simulation.Output.Kalman_mtx.Q_diag=zeros(Dlength-ave_sample,size(X1,1));
%             Simulation.Output.Kalman_mtx.norm_Q=zeros(Dlength-ave_sample,1);
%             Simulation.Output.Kalman_mtx.tun=zeros(Dlength-ave_sample,1);
            %corrected position,velocity and accel(in navigation frame) 
             x1=Simulation.Output.UKF.X(1,:);
             Cbn=Simulation.Output.INS.Cbn(:,:,1);
            [Simulation]=GQ_calcul(x1,Simulation,Cbn,1/fs);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Simulation.Output.User_Def_Sim.Kalman_mtx.P = diag([1,0.01,0.01,0.01,0.018773,0.018773,0.018773]);
            b1=Simulation.Init_Value.Init_Bias_P;
            b1=1e-15;b2=1e-15;b3=0.001;b4=1e-1;b5=1e-1;b6=1e-3;b7=1e-6;b8=1e-3;b9=1e-3;b10=1e-3;b11=1e-3;b12=1e-3;b13=1e-3;b14=1e-3;b15=1e-3;
            Simulation.Output.Kalman_mtx.P = zeros(15,15);
            Simulation.Output.Kalman_mtx.P(1,1) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(2,2) = 1e-6*b2;
            Simulation.Output.Kalman_mtx.P(3,3) = 1e-6*b3;
            Simulation.Output.Kalman_mtx.P(4,4) = 1e-6*b4;
            Simulation.Output.Kalman_mtx.P(5,5) = 1e-6*b4;
            Simulation.Output.Kalman_mtx.P(6,6) = 1e-6*b4;
            Simulation.Output.Kalman_mtx.P(7,7) =  1e-6*b5;
            Simulation.Output.Kalman_mtx.P(8,8) = 1e-6*b5;
            Simulation.Output.Kalman_mtx.P(9,9) =  1e-6*b5;
            Simulation.Output.Kalman_mtx.P(10,10) = 1e-6*b6;
            Simulation.Output.Kalman_mtx.P(11,11) = 1e-6*b6;
            Simulation.Output.Kalman_mtx.P(12,12) =  1e-6*b6;
            Simulation.Output.Kalman_mtx.P(13,13) =  1e-6*b7;
            Simulation.Output.Kalman_mtx.P(14,14) =  1e-6*b7;
            Simulation.Output.Kalman_mtx.P(15,15) =  1e-6*b7;
%             Simulation.Output.Kalman_mtx.P0_Bias=[Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
%                Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)];
            
            Simulation.Output.Kalman_mtx.P_diag=zeros(Dlength,size(X1,1));
            Simulation.Output.Kalman_mtx.P_diag(1,:)=[Simulation.Output.Kalman_mtx.P(1,1) Simulation.Output.Kalman_mtx.P(2,2) Simulation.Output.Kalman_mtx.P(3,3)...
                                                      Simulation.Output.Kalman_mtx.P(4,4) Simulation.Output.Kalman_mtx.P(5,5) Simulation.Output.Kalman_mtx.P(6,6)...
                                                      Simulation.Output.Kalman_mtx.P(7,7) Simulation.Output.Kalman_mtx.P(8,8) Simulation.Output.Kalman_mtx.P(9,9)...
                                                      Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
                                                      Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)];
                                                  
             Simulation.Output.Kalman_mtx.S = chol(Simulation.Output.Kalman_mtx.P);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% initilization of Bias
            Simulation.Output.UKF.X(1,10:12)=1e-5;
            Simulation.Output.UKF.X(1,13:15)=1e-6;
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%             Simulation.Output.Kalman_mtx.Q_adaptive=Simulation.Output.Kalman_mtx.Q;
%             Simulation.Output.Kalman_mtx.V=zeros(Dlength-ave_sample +1,15);
            Simulation.Output.INS.Wnb_b=zeros(Dlength,3);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [WM,WC,W,c] = ut_mweights(length(x1),alpha,beta,kappa);
            Simulation.Output.Kalman_mtx.UKF.WM=WM;
            Simulation.Output.Kalman_mtx.UKF.WC=WC;
            Simulation.Output.Kalman_mtx.UKF.W=W;
            Simulation.Output.Kalman_mtx.UKF.c=c;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            W0 = 0.9;
            [sigma,W2] = Weight_cal (W0,length(x1),alpha,beta);
            Simulation.Output.Kalman_mtx.UKF.newWeight = sigma;
            Simulation.Output.Kalman_mtx.UKF.W2 = W2;
%             Simulation.Output.Kalman_mtx.UKF.WM2=WM2;
%             Simulation.Output.Kalman_mtx.UKF.WC2=WC2;
  
end




