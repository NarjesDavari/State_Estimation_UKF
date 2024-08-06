%UKF_UPDATE1  1st order Extended Kalman Filter update step
%
% Syntax:
%   [M,P,K,MU,S,LH] = UKF_UPDATE1(M,P,Y,H,R,[h,V,param])
%
% In:
%   M  - Nx1 mean state estimate after prediction step
%   P  - NxN state covariance after prediction step
%   Y  - Dx1 measurement vector.
%   H  - Derivative of h() with respect to state as matrix,
%        inline function, function handle or name
%        of function in form H(x,param)
%   R  - Measurement noise covariance.
%   h  - Mean prediction (innovation) as vector,
%        inline function, function handle or name
%        of function in form h(x,param).               (optional, default H(x)*X)
%   V  - Derivative of h() with respect to noise as matrix,
%        inline function, function handle or name
%        of function in form V(x,param).               (optional, default identity)
%   param - Parameters of h                            (optional, default empty)
%
% Out:
%   M  - Updated state mean
%   P  - Updated state covariance
%   K  - Computed Kalman gain
%   MU - Predictive mean of Y
%   S  - Predictive covariance of Y
%   LH - Predictive probability (likelihood) of measurement.
%   
% Description:
%   Unscentended Kalman Filter measurement update step.
%   UKF model is
%
%     y[k] = h(x[k],r),   r ~ N(0,R)

function [Simulation, X,P] = UKF_update2(Simulation,X_hat_minus , P_minus ,S_minus,Q , Y , H , R , X_i2 , WM , WC , W , c,W2,WM2,WC2, sigma,alpha,beta, I)

%% first method
[X_i,X_aug,X_aug2] = ut_sigmas(X_hat_minus,P_minus,Q,c);
X_i (10:15,:)=  X_aug2 (10:15,:);
Y_i = H*X_i;
mu = Y_i*WM;
S = Y_i*W*Y_i';
C = X_i *W* Y_i';
K = C/(S+R);
X = X_hat_minus + K*(Y - mu);
P = P_minus - K*S*K'; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [X_i,X_aug] = ut_sigmas(X_hat_minus,P_minus,Q,c);
% N =length(X_hat_minus);
% % a = .01;
% W0 = 1-N/3;%%1-1/a^2; 
% Wj = (1-W0)/N; %%1/2/N/a^2; %%%
% % gama= sqrt(N/(1-W0));
% % X_i = gama*X_i;
% Rc = chol(R); %%chol(kron(eye(length(Y)),R));
% S_aug = blkdiag(S_minus,Rc);
% N_aug =length(S_aug);
% % X_i = sqrt(N_aug/(1-W0))*[zeros(N_aug) S_aug' -S_aug];
% Y_i = H*X_i;
% 
% 
% mu = W0*Y_i(:,1) + Wj*sum(Y_i(:,2:end),2);%%WM(1,1)*Y_i(:,1) + WM(2,1)*sum(Y_i(:,2:end),2); %%% Y*WM;
% Y_i(:,1) = sqrt(abs(W0+(1-alpha^2+beta)))* (Y_i(:,1)-mu); %%sqrt(abs(WC(1,1)))* (Y_i(:,1)-mu); %%%
% YY = sqrt(Wj)*(Y_i(:,2:2*N+1)-mu*ones(1,2*N)); %%%sqrt(abs(WC(2,1)))*(Y_i(:,2:2*N+1)-mu*ones(1,2*N));%%%
% [~,Rs] = qr(YY');
% Ss = Rs(1:length(Y),1:length(Y));
% Sy = cholupdate(Ss,Y_i(:,1));
% 
% Pxy = 0;
% for j =2:2*N+1
%     Pxy = Pxy + Wj*(X_i(:,j)-X_hat_minus)*(Y_i(:,j)-mu)';
% end
% K = Pxy/(Sy*Sy'+R);
% % A = K*Sy;
% % for n=1:length(Y)
% %     S = cholupdate(chol(P_minus),A(:,n),'-');
% % end
% % P = S*S';
% P = P_minus - K*Sy*K';
% X = X_hat_minus + K*(Y - mu);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% second method
% [X_i,X_aug,X_aug2] = ut_sigmas(X_hat_minus,P_minus,Q,c);
% X_i (10:15,:)=  X_aug2 (10:15,:);
% Y_i = H*X_i;
% mu = Y_i*WM;
% T2=0; C2=0;
%     for j =1:length(WM)
%       T2 = T2+(Y_i(:,j)-mu)*WC(j,1)*(Y_i(:,j)-mu)';
%       C2 = C2+(X_i(:,j)-X_hat_minus)*WC(j,1)*(Y_i(:,j)-mu)';
%     end
% T = T2 + R;
%  [u,d,v1]=svd(T);
%    T=u*sqrt(d)*u';
%  T = Y_i * W * Y_i' + R;
%   C = X_i2 * W * Y_i';
% [u,d,v1]=svd(T);
%         T=u*sqrt(d)*u';
%  K = C2/T; %%diag(diag(T));
%  X = X_hat_minus + (K * (Y - mu));
%  P = P_minus - K * T * K';
%  P = (P+P')/2;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %% third method
%  [X_i,X_aug2] = ut_sigmas_2(length(X_hat_minus),X_hat_minus,P_minus,Q,sigma,alpha);
%  X_i (10:15,:)=  X_aug2 (10:15,:);
% Y_i = H*X_i;
%  mu = Y_i*W2;
%  for j=1:length(W2)
%    Y_ii(:,j) = Y_i(:,j)-mu;
%    X_ii(:,j) =X_i(:,j)-X_hat_minus;
%  end
%  C2=0;
%  for i=1:size(Y_ii,1)
%      Y_y(i,:)=Y_ii(i,:).*W2';
%  end
%  T=Y_y*Y_ii';
%  for i=1:size(X_ii,1) 
%      X_x(i,:)=X_ii(i,:).*W2';
%  end
%       C = X_x*Y_ii';
% 
% %       [u,d,v1]=svd(T);
% %        T=u*sqrt(d)*u';
% %   Simulation.Output.Kalman_mtx.UKF.T(:,I)=T(:,1);
% %   Simulation.Output.Kalman_mtx.UKF.C(:,I)=C(:,1);
%  K = C/(T+R);
%  X = X_hat_minus + (K * (Y - mu));
%  P = P_minus - K * T * K';
%  P = (P+P')/2;
 %% 
 

 
    end

