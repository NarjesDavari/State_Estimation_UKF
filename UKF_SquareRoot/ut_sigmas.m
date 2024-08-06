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


function X = ut_sigmas(M,S,c)

%  A = utchol(P); %%%schol(P);
%   A = chol(P)';
  X = [zeros(size(M)) S -S];
  X = sqrt(c)*X + repmat(M,1,size(X,2));
