%UT_SIGMAS - Generate Sigma Points for Unscented Transformation
%
% Syntax:
%   X = ut_sigmas(M,P,c);
%
% In:
%   M - Initial state mean (Nx1 column vector)
%   P - Initial state covariance
%   c - Parameter returned by UT_WEIGHTS
%
% Out:
%   X - Matrix where 2N+1 sigma points are as columns
%
% Description:
%   Generates sigma points and associated weights for Gaussian
%   initial distribution N(M,P). For default values of parameters
%   alpha, beta and kappa see UT_WEIGHTS.
%
% See also UT_WEIGHTS UT_TRANSFORM UT_SIGMAS  Simo Särkkä


function [X,X_aug,X_aug2] = ut_sigmas(M,P,Q,c)

  A = utchol(P); %%%schol(P);
%   A = chol(P);
B = utchol(Q);
S_aug = blkdiag(A,B);
N_aug = length(S_aug);
w0 = 1-N_aug/3;
cc = sqrt(N_aug/(1-w0));
 X_aug = cc*[zeros(size(S_aug,1)) S_aug' -S_aug'];
   X_aug2 = [zeros(size(M)) B -B];
   X_aug2 = sqrt(c)*X_aug2 + repmat(M,1,size(X_aug2,2));
   
  X = [zeros(size(M)) A -A];
  X = sqrt(c)*X + repmat(M,1,size(X,2));
