function [output1,output2] = LMS_train_DD(input1,input2,TrainSeq1,TrainSeq2,delay,step,Up2,BitPerSym)

TrainLen = length(TrainSeq1);
Len = length(input1);
taps = 2*delay + 1;
w1 = zeros(1,taps*1);
w2 = zeros(1,taps*1);
w1(delay+1) = 1; 
w2(delay+1) = 1; 
input1 = [zeros(1,delay) input1 zeros(1,delay)];
input2 = [zeros(1,delay) input2 zeros(1,delay)];
error1 = zeros(1,TrainLen);

output1 = zeros(1,Len);
output2 = zeros(1,Len);

for ii = 1:Len
    x1 = [input1( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    x2 = [input2( (ii-1)*Up2+1:(ii-1)*Up2+taps )];
    output1(ii) = conj(w1)*x1.';
    output2(ii) = conj(w2)*x2.';
    TrainSeq1(ii) = QAMdecision(output1(ii),BitPerSym);
    TrainSeq2(ii) = QAMdecision(output2(ii),BitPerSym);
    err1 = TrainSeq1(ii) -  output1(ii);
    err2 = TrainSeq2(ii) -  output2(ii);

    w1 = w1 + step.*conj(err1).*(x1);     
    w2 = w2 + step.*conj(err2).*(x2); 
    error1(ii) = err1;
    error2(ii) = err2;
end

figure;plot(abs(error1)); hold on;plot(abs(error2),'r');hold off;


end

