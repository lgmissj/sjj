function [output1,output2] = LMS_train(input1,input2,TrainSeq1,TrainSeq2,delay,step,Up)

TrainLen = length(TrainSeq1);
Len = length(input1);
taps = 2*delay + 1;
w1 = zeros(1,taps*2);
w2 = zeros(1,taps*2);
w1(delay+1) = 1;
w2(delay+1) = 1;
iter = 30;
input1 = [zeros(1,delay) input1 zeros(1,delay)];
input2 = [zeros(1,delay) input2 zeros(1,delay)];
error1 = zeros(1,iter*TrainLen);
error2 = zeros(1,iter*TrainLen);

for jj = 1:iter
  for ii = 1:TrainLen
       x1 = [input1( (ii-1)*Up+1:(ii-1)*Up+taps ) input2( (ii-1)*Up+1:(ii-1)*Up+taps )];
       x2 = [input2( (ii-1)*Up+1:(ii-1)*Up+taps ) input1( (ii-1)*Up+1:(ii-1)*Up+taps )];
       err1 = TrainSeq1(ii) -  conj(w1)*(x1).';
       err2 = TrainSeq2(ii) -  conj(w2)*(x2).';
       w1 = w1 + step.*conj(err1).*(x1);     
       w2 = w2 + step.*conj(err2).*(x2); 
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

