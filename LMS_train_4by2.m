function [X, Y] = LMS_train_4by2(XI,XQ,YI,YQ,TrainSeq1,TrainSeq2,delay,step,Up2,TrainLenSingle)

iter = 10;

TrainLen = length(TrainSeq1);
Len = length(XI);
taps = 2*delay + 1;
w1 = zeros(1,taps*4);
w2 = zeros(1,taps*4);
w3 = zeros(1,taps*4);
w4 = zeros(1,taps*4);
w1(delay+1) = 1; 
w2(delay+1) = 1; 
w3(delay+1) = 1; 
w4(delay+1) = 1; 
% w2(3*delay+1) = 1; 
input1 = [zeros(1,delay) XI zeros(1,delay)];
input2 = [zeros(1,delay) XQ zeros(1,delay)];
input3 = [zeros(1,delay) YI zeros(1,delay)];
input4 = [zeros(1,delay) YQ zeros(1,delay)];
error1 = zeros(1,iter*TrainLen);
error2 = zeros(1,iter*TrainLen);
error3 = zeros(1,iter*TrainLen);
error4 = zeros(1,iter*TrainLen);
% Delta1 = 1*eye(taps*4,taps*4);
% Delta2 = 1*eye(taps*4,taps*4);
% Delta3 = 1*eye(taps*4,taps*4);
% Delta4 = 1*eye(taps*4,taps*4);
% lambda = 0.99;
TrainSeqXI = real(TrainSeq1);
TrainSeqXQ = imag(TrainSeq1);
TrainSeqYI = real(TrainSeq2);
TrainSeqYQ = imag(TrainSeq2);

g = 1*2e-3;
% PLL_Error_X = zeros(1, iter*TrainLen);
% PLL_Phase_X = PLL_Error_X;
% PLL_Error_Y = zeros(1, iter*TrainLen);
% PLL_Phase_Y = PLL_Error_Y;

for jj = 1:iter
for ii = 1:TrainLen
    x1 = [input1( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input2( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input3( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input4( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    x2 = [input2( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input1( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input4( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input3( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    x3 = [input3( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input4( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input1( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input2( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    x4 = [input4( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input3( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input2( (ii-1)*Up2+1:(ii-1)*Up2+taps ) input1( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    x1out = conj(w1)*(x1).';
    x2out = conj(w2)*(x2).';
    x3out = conj(w3)*(x3).';
    x4out = conj(w4)*(x4).';
    XoutPLL = (x1out+1j*x2out);
    YoutPLL = (x3out+1j*x4out);  
    Target1 = TrainSeqXI(ii);
    Target2 = TrainSeqXQ(ii);
    Target3 = TrainSeqYI(ii);
    Target4 = TrainSeqYQ(ii);
%     PLL_Error_X( (jj-1)*TrainLen+ii ) = imag(XoutPLL.*conj(Target1+1j*Target2));
%     PLL_Error_Y( (jj-1)*TrainLen+ii ) = imag(YoutPLL.*conj(Target3+1j*Target4));
    err1 = Target1 -  real(XoutPLL);
    err2 = Target2 -  imag(XoutPLL);
    err3 = Target3 -  real(YoutPLL);
    err4 = Target4 -  imag(YoutPLL);
%     PLL_Phase_X( (jj-1)*TrainLen+ii+1 ) = g*PLL_Error_X( (jj-1)*TrainLen+ii )+PLL_Phase_X( (jj-1)*TrainLen+ii );
%     PLL_Phase_Y( (jj-1)*TrainLen+ii+1 ) = g*PLL_Error_Y( (jj-1)*TrainLen+ii )+PLL_Phase_Y( (jj-1)*TrainLen+ii);
    
%     x1 = [x1(1:2*taps).*exp(-1j*PLL_Phase_X( ii )) x1(2*taps+1:end).*exp(-1j*PLL_Phase_Y( ii ))];
%     x2 = [x2(1:2*taps).*exp(-1j*PLL_Phase_X( ii )) x2(2*taps+1:end).*exp(-1j*PLL_Phase_Y( ii ))];
%     x3 = [x3(1:2*taps).*exp(-1j*PLL_Phase_Y( ii )) x3(2*taps+1:end).*exp(-1j*PLL_Phase_X( ii ))];
%     x4 = [x4(1:2*taps).*exp(-1j*PLL_Phase_Y( ii )) x4(2*taps+1:end).*exp(-1j*PLL_Phase_X( ii ))];
%     G1 = Delta1 * x1.' / (lambda + conj(x1)*Delta1*x1.');
%     G2 = Delta2 * x2.' / (lambda + conj(x2)*Delta2*x2.');
%     G3 = Delta3 * x3.' / (lambda + conj(x3)*Delta3*x3.');
%     G4 = Delta4 * x4.' / (lambda + conj(x4)*Delta4*x4.');

%     Delta1 = 1/lambda * (Delta1 - G1*conj(x1)*Delta1);
%     Delta2 = 1/lambda * (Delta2 - G2*conj(x2)*Delta2);
%     Delta3 = 1/lambda * (Delta3 - G3*conj(x3)*Delta3);
%     Delta4 = 1/lambda * (Delta4 - G4*conj(x4)*Delta4);

%     w1 = w1 + G1.'.*conj(err1);
%     w2 = w2 + G2.'.*conj(err2); 
%     w3 = w3 + G3.'.*conj(err3);
%     w4 = w4 + G4.'.*conj(err4);  

    w1 = w1 + step.*conj(err1).*(x1);
    w2 = w2 + step.*conj(err2).*(x2); 
    w3 = w3 + step.*conj(err3).*(x3);
    w4 = w4 + step.*conj(err4).*(x4);  

    error1( (jj-1)*TrainLen+ii ) = err1;
    error2( (jj-1)*TrainLen+ii ) = err2;
    error3( (jj-1)*TrainLen+ii ) = err3;
    error4( (jj-1)*TrainLen+ii ) = err4;
end
end
plot(abs(error1)); 
hold on;plot(abs(error2));plot(abs(error3));plot(abs(error4));hold off;
legend('XI','XQ','YI','YQ');
output1 = zeros(1,Len);
output2 = zeros(1,Len);
for ii = 1:Len
    in1 = [input1(ii:ii+taps-1) input2(ii:ii+taps-1) input3(ii:ii+taps-1) input4(ii:ii+taps-1)];
    in2 = [input2(ii:ii+taps-1) input1(ii:ii+taps-1) input4(ii:ii+taps-1) input3(ii:ii+taps-1)];
    in3 = [input3(ii:ii+taps-1) input4(ii:ii+taps-1) input1(ii:ii+taps-1) input2(ii:ii+taps-1)];
    in4 = [input4(ii:ii+taps-1) input3(ii:ii+taps-1) input2(ii:ii+taps-1) input1(ii:ii+taps-1)];
    output1(ii) = conj(w1)*in1.';
    output2(ii) = conj(w2)*in2.';
    output3(ii) = conj(w3)*in3.';
    output4(ii) = conj(w4)*in4.';
end
   X = output1 + 1i*output2;
   Y = output3 + 1i*output4;
end

