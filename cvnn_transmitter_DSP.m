% function [output1,output2] = cvnn_transmitter_DSP(input1)
function [output1,output2] = cvnn_transmitter_DSP(input1,BitRateDefault,SampleRateDefault,BitPerSym,UpRate,FrameLen,FrameNum,TrainLen,TrainNum)
addpath f/
addpath ../f/
addpath ../apd/
addpath ../mpam/
% 
% save('C:\Users\84398\Desktop\time_skew_DDMZM\coherent.vtmu_pack\Inputs\send.mat');

%% ======================== Simulation parameters =========================
UpRate = 4;    % Oversampling ratio to simulate continuous time.
BitPerSym=4;
M=16;                                            %modulation order
Npol = 2;                                                              % number of polarization
BaudRate = 30e9;
BitRateDefault = BaudRate*BitPerSym;
SampleRateDefault = BitRateDefault*UpRate;
% BitRateDefault = 2*112e9; % Bit rate
RanSeed = 15;
Nzeros=512;
FrameLen = 1024;
FrameNum = 40;
TrainLen = 256;
TrainNum = 4;
PilotLen=4;
SynLen = 128;
% Nsymb = FrameLen * FrameNum; % 单路偏振符号数
% NTsymb = TrainLen*TrainNum;

ros.rxDSP = 2;
% sim.BERtarget = 1.8e-4; 
% sim.Ndiscard = 1024; % number of symbols to be discarded from the begining and end of the sequence 
% N = Mct*Nsymb; % number points in 'continuous-time' simulation
AWGSamplingRate = 60e9*1;
DSOSamplingRate = 100e9*1+80e9*0;
BER_all = [];

% sim.ModFormat = QAM(4, sim.Rb/sim.Npol, sim.pulse_shape);                  % M-QAM modulation format
% sim.ModFormat = DPSK(4, sim.Rb/sim.Npol, sim.pulse_shape);                     % M-DPSK modulation format
% sim.ModFormat = PAM(4, sim.Rb, 'equally-spaced', sim.pulse_shape);


%% Date Generation And Mapping
rand('seed', 1);
TxBit1 = randi([0 1],BitPerSym,(FrameLen-PilotLen)*FrameNum);    %M=2^3
TxNum1 = 8*TxBit1(1,:)+4*TxBit1(2,:)+2*TxBit1(3,:)+TxBit1(4,:);       %调制用数值代替信号运算
% const=[0.924+0.383i,0.383+0.924i,-0.383+0.924i,-0.924+0.383i,-0.924-0.383i,-0.383-0.924i,0.383-0.924i,0.924-0.383i];
% TxSym1 = genqammod(TxNum1,const);            %8PSK映射

TxSym1=qammod(TxNum1,M);
% scatterplot(TxSym1);

% abd=genqamdemod(TxSym1,const);
% sim.TxModData = pammod(sim.TxBit,4);             %8PSK的另一种映射方式
% sim.TxModData = pskmod(sim.TxNum,sim.M);
% scatterplot(sim.TxModDate);
rand('seed', 2);
TxBit2 = randi([0 1],BitPerSym, (FrameLen-PilotLen)*FrameNum);    %M=2^3
TxNum2 = 8*TxBit2(1,:)+4*TxBit2(2,:)+2*TxBit2(3,:)+TxBit2(4,:);       %调制用数值代替信号运算
TxSym2=qammod(TxNum2,M);
% sim.TxModData = pammod(sim.TxBit,4);             %8PSK的另一种映射方式
% sim.TxModData = pskmod(sim.TxNum,sim.M);
% scatterplot(sim.TxModDate);
rand('seed', 3);
PilotBit1 = randi([0 1],BitPerSym, PilotLen*FrameNum);    %M=2^3
PilotNum1 = 8*PilotBit1(1,:)+4*PilotBit1(2,:)+2*PilotBit1(3,:)+PilotBit1(4,:);       %调制用数值代替信号运算
PilotSym1 = qammod(PilotNum1,M);            %映射rand('seed', 2);

rand('seed', 4);
PilotBit2 = randi([0 1],BitPerSym, PilotLen*FrameNum);    %M=2^3
PilotNum2 = 8*PilotBit2(1,:)+4*PilotBit2(2,:)+2*PilotBit2(3,:)+PilotBit2(4,:);       %调制用数值代替信号运算
PilotSym2 = qammod(PilotNum2,M);            %映射rand('seed', 2);

rand('seed', 5);
TrainBit1 = randi([0 1],BitPerSym, TrainLen);    %M=2^3
TrainNum1 =  8*TrainBit1(1,:)+4*TrainBit1(2,:)+2*TrainBit1(3,:)+TrainBit1(4,:);       %调制用数值代替信号运算
TrainSym1 = qammod(TrainNum1,M);            %映射

rand('seed', 6);
TrainBit2 = randi([0 1],BitPerSym,TrainLen);    %M=2^3
TrainNum2 = 8*TrainBit2(1,:)+4*TrainBit2(2,:)+2*TrainBit2(3,:)+TrainBit2(4,:);       %调制用数值代替信号运算
TrainSym2 = qammod(TrainNum2,M);            %映射

rand('seed', 0);
SynBit1 = randi([0 1],BitPerSym,SynLen);    %M=2^3
SynNum1 = 8*SynBit1(1,:)+4*SynBit1(2,:)+2*SynBit1(3,:)+SynBit1(4,:);       %调制用数值代替信号运算
SynSym1 = qammod(SynNum1,M);            %映射
%% Framing
TxSymMatrix1 = reshape(TxSym1,FrameLen-PilotLen,[]).';
TxSymMatrix2 = reshape(TxSym2,FrameLen-PilotLen,[]).';
Plen = floor(FrameLen/(PilotLen+1));
FrameSym1 = zeros(1,FrameLen*FrameNum);
FrameSym2 = zeros(1,FrameLen*FrameNum);
for aa = 1:FrameNum
    temp1 = TxSymMatrix1(aa,:);
    temp2 = TxSymMatrix2(aa,:);
    tempHead1 = temp1(1:Plen*PilotLen);
    tempHead2 = temp2(1:Plen*PilotLen);
    tempTail1 = temp1(Plen*PilotLen+1:end);
    tempTail2 = temp2(Plen*PilotLen+1:end);
    tempHead1 = reshape(tempHead1,Plen,[]);
    tempHead2 = reshape(tempHead2,Plen,[]);
    tempHead1 = [tempHead1; PilotSym1( (aa-1)*PilotLen+1:aa*PilotLen)];
    tempHead2 = [tempHead2; PilotSym2( (aa-1)*PilotLen+1:aa*PilotLen)];
    tempHead1 = reshape(tempHead1,1,[]);
    tempHead2 = reshape(tempHead2,1,[]);
    temp1 = [tempHead1 tempTail1];
    temp2 = [tempHead2 tempTail2];
    FrameSym1((aa-1)*FrameLen+1:aa*FrameLen) = temp1;
    FrameSym2((aa-1)*FrameLen+1:aa*FrameLen) = temp2;
end
TrainPattern1 = repmat([TrainSym1 zeros(1,TrainLen)],1,TrainNum);
TrainPattern2 = repmat([zeros(1,TrainLen) TrainSym2],1,TrainNum);
FrameSym1 = [TrainPattern1 FrameSym1];
FrameSym2 = [TrainPattern2 FrameSym2];
%% upsample
% Up = AWGSamplingRate/1e9;实验上采样大小 与AWG相关
Up = 2;
% UpsampleSym_test = upsample(FrameSym,Up);
UpsampleSym1 = reshape([FrameSym1;zeros((Up-1),length(FrameSym1))],1,[]);
UpsampleSym2 = reshape([FrameSym2;zeros((Up-1),length(FrameSym2))],1,[]);
%% Transmitter
% rrcFilter = rcosdesign(sim.Rolloff,sim.Span,sim.BitPerSym,'sqrt');

 
Rolloff=0.01;
Span=128;
pulse_shape = select_pulse_shape('rrc',UpRate,Rolloff,Span);                     % pulse shape
% TxModFData1=[TxModDate1 TxModTData1];                              %信号和训练符号拼接
TxWfm1=conv(UpsampleSym1,pulse_shape.h);                  %生成发送脉冲波
TxWfmNorm1=TxWfm1/sqrt(mean(abs(TxWfm1).^2));   
% TxModFData2=[TxModDate2 TxModTData2];
TxWfm2=conv(UpsampleSym2,pulse_shape.h);   
%  sim.TxWS=conv(sim.TxSampleDate,rrcFilter,'same');
TxWfmNorm2=TxWfm2/sqrt(mean(abs(TxWfm2).^2));                       %normalization


%% Output
output1 = input1;
output1.band.E = TxWfmNorm1;
output2 = input1;
output2.band.E = TxWfmNorm2;

end



