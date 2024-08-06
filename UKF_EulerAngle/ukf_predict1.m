d%UKF_PREDICT2  Augmented (state and process noise) UKF prediction step
%
% Syntax:
%   [M,P] = UKF_PREDICT2(M,P,a,Q,[param,alpha,beta,kappa])
%
% In:
%   M - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   f - Dynamic model function as inline function,
%       function handle or name of function in
%       form a([x;w],param)
%   Q - Non-singular covariance of process noise w
%   f_param - Parameters of f               (optional, default empty)
%   alpha - Transformation parameter      (optional)
%   beta  - Transformation parameter      (optional)
%   kappa - Transformation parameter      (optional)
%   mat   - If 1 uses matrix form         (optional, default 0)
%
% Out:
%   M - Updated state mean
%   P - Updated state covariance
%
% Description:
%   Perform augmented form Unscented Kalman Filter prediction step
%   for model
%
%    x[k+1] = a(x[k],w[k],param)
%
%   Function a should be such that it can be given
%   DxN matrix of N sigma Dx1 points and it returns 
%   the corresponding predictions for each sigma
%   point. 

function [M,P,Y] = UKF_predict1(M , P , f , G , Q  ,Qc, C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt , WM ,WC , W ,sigma,W2,WM2,WC2,alpha, c,I)

  %
  % Do transform
  % and add process noise
  %
%  P = P + Q;
  [M,P,Y] = ut_transform(M , P , f , G , Q , Qc , C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt , WM ,WC , W ,sigma,W2,WM2,WC2,alpha, c,I);
   P = P + Q;
 
end