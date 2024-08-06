function [Simulation,dX,P,flag_Qadapt,alpha,beta]=eskf_update_select(Simulation,dX,P,dz,H,H2,M,R,I2,V,Selection_Param,flag_accelrollpitch,Include_R_adaptive,Include_Q_adaptive,Include_VB_adaptive, Include_IF_VB_adaptive,alpha,beta,alphaN,betaN)
        
           windowSize_adaptive= Simulation.Output.Kalman_mtx.WindowSize_adaptive;
           Simulation.Output.Kalman_mtx.update_counter = Simulation.Output.Kalman_mtx.update_counter+1;
           update_counter=Simulation.Output.Kalman_mtx.update_counter;
            Simulation.Output.Kalman_mtx.X_hat_minus(update_counter,:)=dX;%%%?????????
%            Simulation.Output.Kalman_mtx.V(Simulation.Output.Kalman_mtx.update_counter,:) = V';

        if Include_VB_adaptive
              [Simulation,dX ,P,alpha,beta] = eskf_update_VB(Simulation,dX , P , dz , H, H2,M,I2,alpha,beta,flag_accelrollpitch,windowSize_adaptive);
%             [Simulation]=R_adaptive(Simulation,R,windowSize_adaptive,update_counter,I,Selection_Param,flag_accelrollpitch);            
              Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%             Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
              Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
%               Simulation.Output.Kalman_mtx.alpha_VB=alpha;
%               Simulation.Output.Kalman_mtx.beta_VB=beta;
        elseif Include_IF_VB_adaptive
              [Simulation,dX,P,alpha,beta] = update_IF_VB(Simulation,dX,dz,H,alpha,beta,alphaN,betaN,I2);
              
        elseif  Simulation.Output.Kalman_mtx.update_counter >windowSize_adaptive && Include_R_adaptive
            Simulation.Output.Kalman_mtx.adaptive_innovation=1;
           [Simulation,C]= Covariance_innovation(Simulation,windowSize_adaptive);
           [Simulation,dX,P] = eskf_update_adaptive_innovation(Simulation,dX,P,dz,H,I2,windowSize_adaptive,C,Selection_Param,flag_accelrollpitch);
%          [X,P,landa_P] = ekf_update1_adaptive2(Simulation,X,P,Y,H,I,windowSize_adaptive,C);

           Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
           Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
           Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
                     
        else
           [dX , P] = eskf_update(dX , P , dz , H, H2, M , R,I2 );
%            [Simulation]=R_adaptive(Simulation,R,windowSize_adaptive,update_counter,I,Selection_Param,flag_accelrollpitch);

%            Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%          Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
%            Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
        end
            
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q_adaptive
           
           if  Simulation.Output.Kalman_mtx.update_counter >windowSize_adaptive && Include_Q_adaptive
               [Simulation] = Estimate_Q(Simulation);
               flag_Qadapt=1;
           else
               flag_Qadapt=0;
           end
               
           
           
           
           