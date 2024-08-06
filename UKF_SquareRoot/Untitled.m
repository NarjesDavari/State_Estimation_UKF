function [] = Weight_cal (W0,n)
W(1) = 0;
    W(2) = (1 - W(1)) / (n + 1);
    W(3) = W(2);
    W(4) = W(3);
    W(5) = W(4);
sigma = zeros(n,n+2);
sigma(1,1) = 0;
sigma(1,2) = -1 / sqrt(2 * W(2));
sigma(1,3) = 1 / sqrt(2 * W(2));
for j=2:n
    for i=1:n+2
%         if (i-1)>(j+1)
%            sigma (j,i)=0; 
        if (i-1)==(j+1)
            sigma (1:j,i) = [zeros(j-1,1);j/sqrt(j*(j+1)*W(2))];
        elseif (i-1)==0
            sigma (1:j,i) = [sigma(1:j-1,1) ; 0];
        elseif (i-1)<=j
            sigma (1:j,i) = [sigma(1:j-1,i); -1/sqrt(j*(j+1)*W(2))];
        end
    end
end