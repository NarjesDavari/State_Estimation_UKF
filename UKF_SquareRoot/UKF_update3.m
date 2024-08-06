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

function [X,P] = UKF_update3(X_hat_minus , P_minus , Y, Y_i, H , R ,sigma,W2,WM2,WC2,alpha, I)

% X_i = ut_sigmas_2(length(X_hat_minus),X_hat_minus,P_minus,sigma,alpha);
%%%%%%%%%%%%%%%
 Y_o = H*Y_i;
mu = Y_o*WM2;

% T_ = zeros(length(Y));
% C = zeros(length(X_hat_minus),1);
% for i = 1: size(Y_i,2)
%     T_  = T_ + (Y_i(:,i)-mu)*WC2(i,1)*(Y_i(:,i)-mu)';
%     C  = C + (X_i(:,i) - X_hat_minus)*WC2(i,1)*(Y_i(:,i)-mu)';
% end

T_  =  Y_o*W2*Y_o';
C  = Y_i*W2*Y_o';

 T = T_ + R;

%         [u,d,v1]=svd(T);
%         T=u*sqrt(d)*u';
        
     K = C/diag(diag(T));
 X = X_hat_minus + (K * (Y - mu));
 P = P_minus - K * T * K';
 


