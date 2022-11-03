function [output1,output2] = LMS_train_PLL(input1,input2,TrainSeq1,TrainSeq2,delay,step,Up)

TrainLen = length(TrainSeq1);
Len = length(input1);
taps = 2*delay + 1;
w1 = zeros(1,taps*2);
w2 = zeros(1,taps*2);
w1(delay+1) = 1;
w2(delay+1) = 1;
iter = 10;
g = 1e-2;
input1 = [zeros(1,delay) input1 zeros(1,delay)];
input2 = [zeros(1,delay) input2 zeros(1,delay)];
error1 = zeros(1,iter*TrainLen);
error2 = zeros(1,iter*TrainLen);

PLL_Error_X = zeros(1, iter*TrainLen);
PLL_Phase_X = PLL_Error_X;
PLL_Error_Y = zeros(1, iter*TrainLen);
PLL_Phase_Y = PLL_Error_Y;

for jj = 1:iter
  for ii = 1:TrainLen
       x1 = [input1( (ii-1)*Up+1:(ii-1)*Up+taps ) input2( (ii-1)*Up+1:(ii-1)*Up+taps )];
       x2 = [input2( (ii-1)*Up+1:(ii-1)*Up+taps ) input1( (ii-1)*Up+1:(ii-1)*Up+taps )];
       x1out = conj(w1)*(x1).';
       x2out = conj(w2)*(x2).';

       x1outPLL = x1out.*exp(-1j*PLL_Phase_X( (jj-1)*TrainLen+ii ));
       x2outPLL = x2out.*exp(-1j*PLL_Phase_Y( (jj-1)*TrainLen+ii ));  
       
       Target1 = TrainSeq1(ii);
       Target2 = TrainSeq2(ii);
       
       PLL_Error_X( (jj-1)*TrainLen+ii ) = imag(x1outPLL.*conj(Target1));
       PLL_Error_Y( (jj-1)*TrainLen+ii ) = imag(x2outPLL.*conj(Target2));
       err1 = Target1 -  x1outPLL;
       err2 = Target2 -  x2outPLL;
       PLL_Phase_X( (jj-1)*TrainLen+ii+1 ) = g*PLL_Error_X( (jj-1)*TrainLen+ii )+PLL_Phase_X( (jj-1)*TrainLen+ii );
       PLL_Phase_Y( (jj-1)*TrainLen+ii+1 ) = g*PLL_Error_Y( (jj-1)*TrainLen+ii )+PLL_Phase_Y( (jj-1)*TrainLen+ii);
       w1 = w1 + step.*conj(err1).*( [x1(1:taps).*exp(-1j*PLL_Phase_X( (jj-1)*TrainLen+ii )) x1(taps+1:2*taps).*exp(-1j*PLL_Phase_Y( (jj-1)*TrainLen+ii ))] );     
       w2 = w2 + step.*conj(err2).*( [x2(1:taps).*exp(-1j*PLL_Phase_Y( (jj-1)*TrainLen+ii )) x2(taps+1:2*taps).*exp(-1j*PLL_Phase_X( (jj-1)*TrainLen+ii ))]); 
       error1((jj-1)*TrainLen+ii) = err1;
       error2((jj-1)*TrainLen+ii) = err2;
  end
end
plot(abs(error1)); hold on;plot(abs(error2),'r');hold off;

output1 = zeros(1,Len);
output2 = zeros(1,Len);
for ii = 1:Len
    in1 = [input1(ii:ii+taps-1) input2(ii:ii+taps-1)];
    in2 = [input2(ii:ii+taps-1) input1(ii:ii+taps-1)];
    output1(ii) = conj(w1)*(in1).';
    output2(ii) = conj(w2)*(in2).';
end

end

