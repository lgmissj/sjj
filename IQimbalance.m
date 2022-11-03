function [outputI, outputQ]=IQimbalance(inputI,inputQ)

% I=real(input);
% Q=imag(input);
I = inputI;
Q = inputQ;
I=I-mean(I);
Q=Q-mean(Q);

% a = mean(I.*Q)/mean(abs(I).*abs(Q));
a = mean(I.*Q)/sqrt(mean(I.^2.*Q.^2));
k1 = 1/sqrt(1+a);
k2 = 1/sqrt(1-a);
outputI = (k1+k2)*I+(k1-k2)*Q;
outputQ = (k1-k2)*I+(k1+k2)*Q;
% outputI = outputI/2;
% outputQ = outputQ/2;
% output = ((k1+k2)*I+(k1-k2)*Q) + 1i*((k1-k2)*I+(k1+k2)*Q);