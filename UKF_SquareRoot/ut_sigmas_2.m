%UT_SIGMAS - Generate Sigma Points for Unscented Transformation
%
function X_point = ut_sigmas_2(n,X,P,sigma,alpha)

 A = utchol(P); %%%schol(P);
 for i=1:n+2
     X_point(:,i) = X + alpha*A* sigma (:,i);
 end