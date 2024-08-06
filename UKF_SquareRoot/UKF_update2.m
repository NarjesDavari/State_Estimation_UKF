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

function [X,S] = UKF_update2(X_hat_minus , S_minus , Y , H , R , X_i2, WM , WC , W , c,sigma,alpha, Q, I)

 X_i = ut_sigmas(X_hat_minus,Q,c);
%% or 
% X_i = ut_sigmas_2(length(X_hat_minus),X_hat_minus,P_minus,sigma,alpha);

 Y_i = H*X_i;
 mu = Y_i*WM;
%%%%%%%%%%%%%%%%  
%  S_hat = qr([sqrt(WC(2,1)*(Y_i-mu)) sqrt(R)]);
% S_hat = cholupdate(S_hat, Y_i(:,1)-mu, WC(1,1));
% 
% P_xy =0;
% for j =1:length(WC)
%       P_xy = P_xy+(X_i(:,j)-X_hat_minus)*WC(j,1)*(Y_i(:,j)-mu)';
% end
% 
%  K = (P_xy /S_hat')/S_hat; %%diag(diag(T));
%  X = X_hat_minus + (K * (Y - mu));
%  U = K *S_hat;
%  S = cholupdate (S_minus, U ,-1);
 %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 [q,r] = qr([sqrt(WC(2,1))*(Y_i(:,2:end) - mu)]'); 
S_ = r(1:length(r)/2,:);
 S_hat = cholupdate(S_, Y_i(:,1)- mu, sign(WC(1,1)));
% P = S_'*S_ + (sign(WC(1,1))*(Y_i(:,1)- mu))*(sign(WC(1,1))*(Y_i(:,1)- mu))'; %%% if WC(1,1)=WC0<0

 P_xy =0;
for j =1:length(WC)
      P_xy = P_xy+(X_i(:,j)-X_hat_minus)*WC(j,1)*(Y_i(:,j)-mu)';
end

 K = (P_xy /S_hat')/S_hat; 
  X = X_hat_minus + (K * (Y - mu));
S = cholupdate(S_minus, K*S_hat,-1);

