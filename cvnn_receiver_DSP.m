function [BER] = cvnn_receiver_DSP(Xi,Xq,Yi,Yq,BitRateDefault,SampleRateDefault,BitPerSym,UpRate,FrameLen,FrameNum,TrainLen,TrainNum)
addpath DSP/
addpath f/
addpath ../f/
addpath ../apd/
addpath ../mpam/

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
% TrainPattern1 = repmat([TrainSym1],1,TrainNum);
% TrainPattern2 = repmat([TrainSym2],1,TrainNum);
% FrameSym1 = [SynSym1 TrainPattern1 FrameSym1];
% FrameSym2 = [SynSym1*0 TrainPattern2 FrameSym2];另一种组帧方式
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
% fvtool(sim.pulse_shape.h);
% figure;
% plot(10*log10(abs(fftshift(fft(sim.TxWfmNorm)))));
% title('TxWfmNorm');

% %% 噪声部分人为添加代码 VPI联合仿真可忽略
% %% Fiber dispersion emulation
% 
% C_speed = 3e8;
% FiberLen = 10*80e3;
% SampleRateDefault = BaudRate*Up;
% lamda = C_speed/193.1e12;
% CD_value = 17e-6 * FiberLen;
% N = length(TxWfm1);
% TimeW = N/SampleRateDefault;
% beta2 = -lamda^2*CD_value/2/pi/C_speed;
% w = 2*pi*[(0:N/2-1),(-N/2:-1)]/TimeW;
% TxWfmFFT1 = fft(TxWfm1);
% TxWfmFFT1 = TxWfmFFT1.*exp(1i*-beta2*(w.^2)/2);
% RxWfm1 = ifft(TxWfmFFT1);
% TxWfmFFT2 = fft(TxWfm2);
% TxWfmFFT2 = TxWfmFFT2.*exp(1i*-beta2*( w.^2)/2);
% RxWfm2 = ifft(TxWfmFFT2);
% %% polarization rotation emulation
% phi = 45/180*pi;
% temp1 = RxWfm1;
% temp2 = RxWfm2;
% RxWfm1 = cos(phi)*temp1 + sin(phi)*temp2;
% RxWfm2 = -sin(phi)*temp1 + cos(phi)*temp2;
% 
% PMD = 10e-12;
% RxWfm1 = ifft( fft(RxWfm1).*exp(-1j*2*pi*[0:floor((length(RxWfm1)-1)/2) -ceil((length(RxWfm1)-1)/2):-1]/length(RxWfm1)*BaudRate*Up*PMD) );
% 
% phi = 45/180*pi;
% temp1 = RxWfm1;
% temp2 = RxWfm2;
% RxWfm1 = cos(phi)*temp1 + sin(phi)*temp2;
% RxWfm2 = -sin(phi)*temp1 + cos(phi)*temp2;
% 
% %% AWGN
%     RxWfm1_Orin = RxWfm1;
%     RxWfm2_Orin = RxWfm2;
% for OSNR_dB = 25
%     close all;
% AWGNSeed = 20;
% SNR_dB = OSNR_dB +10*log10(12.5e9/AWGSamplingRate);
% SNR = 10^(SNR_dB/10);
% len = length(RxWfm1_Orin);
% sigma1 = sqrt(mean(abs(RxWfm1_Orin).^2+abs(RxWfm2_Orin).^2)/2/2/SNR*1);
% rand('seed', AWGNSeed);
% RxWfm1 = RxWfm1_Orin + sigma1*( randn(1,len) + 1i*randn(1,len) );
% rand('seed', AWGNSeed+1);
% RxWfm2 = RxWfm2_Orin + sigma1*( randn(1,len) + 1i*randn(1,len) );
% %% Frequency offset emulation
% FreOffReal = 1*1e8;    % unknown to the receiver
% RxWfm1 = RxWfm1.*exp(1i*2*pi*FreOffReal*[1:length(RxWfm1)]/BaudRate/Up);
% RxWfm2 = RxWfm2.*exp(1i*2*pi*FreOffReal*[1:length(RxWfm2)]/BaudRate/Up);
% %% Phase noise emulation
% linewidth = 1*2e5;
% PNSeed = 30;
% sigma2 = sqrt(2*pi*linewidth*1/BaudRate/Up);
% randn('seed',PNSeed);
% deltaphase = sigma2*randn(1,length(RxWfm1));
% phasenoise = cumsum(deltaphase);
% RxWfm1 = RxWfm1.*exp(1i*phasenoise);
% RxWfm2 = RxWfm2.*exp(1i*phasenoise);
%% reciever
RxWfmXi = Xi.band.E;
RxWfmXq = Xq.band.E;
RxWfmYi = Yi.band.E;
RxWfmYq = Yq.band.E;
%plot
% figure;
% plot(linspace(-SampleRateDefault/2,SampleRateDefault/2,length(RxWfm)),10*log10(fftshift(abs(fft(RxWfm)))));
% title('RxWfm');

RxWfmXi = RxWfmXi - mean(RxWfmXi);
RxWfmXi = RxWfmXi./sqrt(mean(abs(RxWfmXi).^2));

RxWfmXq = RxWfmXq - mean(RxWfmXq);
RxWfmXq = RxWfmXq./sqrt(mean(abs(RxWfmXq).^2));

RxWfmYi = RxWfmYi - mean(RxWfmYi);
RxWfmYi = RxWfmYi./sqrt(mean(abs(RxWfmYi).^2));

RxWfmYq = RxWfmYq - mean(RxWfmYq);
RxWfmYq = RxWfmYq./sqrt(mean(abs(RxWfmYq).^2));


%% DSO 3&4  ????????
% pwelch(RxWfmXi);
% N = length(RxWfmXq);
% XQ = real(ifft( fft(RxWfmXq).*exp(-1j*2*pi*[0:floor((N-1)/2) -ceil((N-1)/2):-1]/N*DSOSamplingRate*8e-12) ));
% skew1 = 0;
% RxWfmYi = real(ifft( fft(RxWfmYi).*exp(-1j*2*pi*[0:floor((N-1)/2) -ceil((N-1)/2):-1]/N*DSOSamplingRate*(skew1)*1e-12) ));
% RxWfmYq = real(ifft( fft(RxWfmYq).*exp(-1j*2*pi*[0:floor((N-1)/2) -ceil((N-1)/2):-1]/N*DSOSamplingRate*(skew1-8)*1e-12) ));

%% IQ imbalance
[RxWfmXi,RxWfmXq] = IQimbalance(RxWfmXi,RxWfmXq);
[RxWfmYi,RxWfmYq] = IQimbalance(RxWfmYi,RxWfmYq);
RxWfmX = RxWfmXi + 1i*RxWfmXq;
RxWfmY = RxWfmYi + 1i*RxWfmYq;
%         XI = resample(RxWfmXi, Up2*BaudRate/1e9 ,Up);
%         XQ = resample(RxWfmXq, Up2*BaudRate/1e9 ,Up);
%         YI = resample(RxWfmYi, Up2*BaudRate/1e9 ,Up);
%         YQ = resample(RxWfmYq, Up2*BaudRate/1e9 ,Up);
%% resample
% 
% RxWfmX=TxWfmNorm1;
% RxWfmY=TxWfmNorm2;


% Up = DSOSamplingRate/1e9;实验的Up，与示波器采样速率相关
Up2 = 2;
RxWfm1 = resample(RxWfmX, Up2 ,Up);
RxWfm2 = resample(RxWfmY, Up2 ,Up);
% RxWfm1 = resample(RxWfmX, Up2*BaudRate/1e9 ,Up);
% RxWfm2 = resample(RxWfmY, Up2*BaudRate/1e9 ,Up);
RxWfm10 = RxWfmX;
RxWfm20 = RxWfmY;

%% dispersion compensation 
CD_Est_Mode = 2;
switch CD_Est_Mode
    case 1
% chromatic dispersion estimation
NNN = 4000;
temp0 = RxWfm2(1:Up2:NNN*Up2);
b = ifft(abs(fft(temp0(NNN/2+1:NNN/2+NNN/4))).^2);
b(1)=0;
[~,cd] = max(b(1:NNN/8));
EstDistance = cd*C_speed/lamda.^2/BaudRate^2*1000/17/80
c=abs(b).';
plot(c);
    case 2
% chromatic dispersion estimation Sweep 
NNN = 2048;
FiberLen_Test = [0:5:1000]*1e3;   %
SampleRateDefault = BaudRate*Up2;
[ EstDistance ] = DispersionEstPAPR( RxWfm1, RxWfm2, NNN, FiberLen_Test, SampleRateDefault )
end
C_speed = 3e8;
FiberLen = EstDistance*80e3;
SampleRateDefault = BaudRate*Up2;
lamda = C_speed/193.1e12;
CD_value = 17e-6 * FiberLen;
N = length(RxWfm1);
TimeW = N/SampleRateDefault;
beta2 = -lamda^2*CD_value/2/pi/C_speed;
w = 2*pi*[(0:N/2-1),(-N/2:-1)]/TimeW;
RxWfmFFT1 = fft(RxWfm1);
RxWfmFFT1 = RxWfmFFT1.*exp(1i*beta2*(w.^2)/2);
RxWfm1 = ifft(RxWfmFFT1);
RxWfmFFT2 = fft(RxWfm2);
RxWfmFFT2 = RxWfmFFT2.*exp(1i*beta2*( w.^2)/2);
RxWfm2 = ifft(RxWfmFFT2);

 %% matches RRC filter
pulse_shape = select_pulse_shape('rrc',UpRate,Rolloff,Span); 
RxWfm1=conv(RxWfm1,pulse_shape.h);  
RxWfm2=conv(RxWfm2,pulse_shape.h);  

%% FOE
RxWfm1temp = RxWfm1(1:Up2:end);
RxWfm2temp = RxWfm2(1:Up2:end);
a1 = abs(fftshift(fft(RxWfm1temp.^4)));
a2 = abs(fftshift(fft(RxWfm2temp.^4)));
[~,FO1] = max(a1);
[~,FO2] = max(a2);
% plot(a2);
FreOffEst1 = (FO1 - round(length(RxWfm1temp)/2))/4/length(RxWfm1temp)*BaudRate;
FreOffEst2 = (FO2 - round(length(RxWfm2temp)/2))/4/length(RxWfm2temp)*BaudRate;
if abs(FreOffEst1)>abs(FreOffEst2)
    FreOffEst = FreOffEst1;
else
    FreOffEst = FreOffEst2;
end
FreOffEst 
RxWfm1 = RxWfm10.*exp(-1i*2*pi*FreOffEst*[1:length(RxWfm10)]/BaudRate/Up2);
RxWfm2 = RxWfm20.*exp(-1i*2*pi*FreOffEst*[1:length(RxWfm20)]/BaudRate/Up2);
RxWfm1 = conv(RxWfm1,pulse_shape.h);
RxWfm2 = conv(RxWfm2,pulse_shape.h);
% 
%         XI = XI0.*exp(-1i*2*pi*FreOffEst*[1:length(XI0)]/BaudRate/Up2);
%         XQ = XQ0.*exp(-1i*2*pi*FreOffEst*[1:length(XQ0)]/BaudRate/Up2);
%         YI = YI0.*exp(-1i*2*pi*FreOffEst*[1:length(YI0)]/BaudRate/Up2);
%         YQ = YQ0.*exp(-1i*2*pi*FreOffEst*[1:length(YQ0)]/BaudRate/Up2);
%         XI = conv(XI,RRCFilt);
%         XQ = conv(XQ,RRCFilt);
%         YI = conv(YI,RRCFilt);
%         YQ = conv(YQ,RRCFilt);
        %% synchronization
%         RxWfm1 = XI+1i*XQ;
%         RxWfm2 = YI+1i*YQ;
        % RxWfm1 = XI;
        % RxWfm2 = XQ;
RxWfm1 = RxWfm1/sqrt(mean(abs(RxWfm1).^2));
RxWfm2 = RxWfm2/sqrt(mean(abs(RxWfm2).^2));
SynLen = 128;
corrLen = 10000;
thres = 120;
SynSeq = TrainSym1(1:SynLen);
corr1 = zeros(1,corrLen);
corr2 = zeros(1,corrLen);
for pp = 1:corrLen
    corr1(pp) = abs(RxWfm1( pp : Up2 : pp+(SynLen-1)*Up2 )*SynSeq')';
    corr2(pp) = abs(RxWfm2( pp : Up2 : pp+(SynLen-1)*Up2 )*SynSeq')';
end
% plot(corr1);hold on;plot(corr2,'r');hold off;
for pp = 1:corrLen
    if (corr1(pp) >= thres) || (corr2(pp) >= thres)
        posMax = pp + SynLen*Up2;    
        break;
    end
end
[~,pos1] = max(corr1(1:posMax));
[~,pos2] = max(corr2(1:posMax));
if corr1(pos1) >= corr2(pos2)
    pos = pos1;
else pos =pos2;
end
RxSeq1 = RxWfm1(pos:pos+(TrainLen*TrainNum*2+FrameLen*FrameNum)*Up2-1);
RxSeq2 = RxWfm2(pos:pos+(TrainLen*TrainNum*2+FrameLen*FrameNum)*Up2-1);
% scatterplot(RxSeq1);

%% Equalization stage 1
TrainSeq1 = TrainPattern1;
TrainSeq2 = TrainPattern2;
EqMode = 1;
switch EqMode
    case 0
        RxEqu1 = RxSeq1;
        RxEqu2 = RxSeq2;
    case 1
        delay = 24;
        step = 0.001;
        [RxEqu1,RxEqu2] = LMS_train(RxSeq1,RxSeq2,TrainSeq1,TrainSeq2,delay,step,Up2);
    case 2
        delay = 24;
        step = 0.001;
        [RxEqu1,RxEqu2] = LMS_train_4by2(real(RxSeq1),imag(RxSeq1),real(RxSeq2),imag(RxSeq2),TrainSeq1,TrainSeq2,delay,step,Up2);
    case 3
        delay = 24;
        step = 0.001;
        [RxEqu1,RxEqu2] = LMS_train_PLL(RxSeq1,RxSeq2,TrainSeq1,TrainSeq2,delay,step,Up2);
    case 4 
        delay = 24;
        step = 0.001;
        [RxEqu1,RxEqu2] = LMS_train_4by2_PLL(real(RxSeq1),imag(RxSeq1),real(RxSeq2),imag(RxSeq2),TrainSeq1,TrainSeq2,delay,step,Up2);
    case 5
        delay = 24;
        step = 0.0001;
        N=20;L=20;
        [RxEqu1, RxEqu2] = FSBA_MIMO_LMS(RxSeq1,RxSeq2,TrainSeq1,TrainSeq2,N,L,step);Up2=1;
end

%% downsample
RxDown1 = RxEqu1(1:Up2:end);
RxDown2 = RxEqu2(1:Up2:end);
RxDown1 = RxDown1(TrainLen*TrainNum*2+1:end);
RxDown2 = RxDown2(TrainLen*TrainNum*2+1:end);

        %% Pilot
        PilotBlock = FrameLen;
        Plen = floor(PilotBlock/(PilotLen+1));
        pilottx1 = reshape(PilotSym1,PilotLen,[]);
        pilottx2 = reshape(PilotSym2,PilotLen,[]);
        temprx1 = reshape(RxDown1,PilotBlock,[]);
        temprx2 = reshape(RxDown2,PilotBlock,[]);
        pilotrx1 = temprx1(Plen+1:Plen+1:end,:);
        pilotrx2 = temprx2(Plen+1:Plen+1:end,:);
        
        rotate1 = pilotrx1./pilottx1;
        rotate2 = pilotrx2./pilottx2;
        rotate1 = mean(rotate1,1);
        rotate2 = mean(rotate2,1);
        rotate1 = rotate1./abs(rotate1);
        rotate2 = rotate2./abs(rotate2);
        rotate1 = repmat(rotate1,PilotBlock,1);
        rotate2 = repmat(rotate2,PilotBlock,1);
        RxDown1 = temprx1./rotate1;
        RxDown2 = temprx2./rotate2;
        RxDown1 = reshape(RxDown1,1,[]);
        RxDown2 = reshape(RxDown2,1,[]);
        % scatterplot(RxDown1);
        % scatterplot(RxDown2);
%         RxDown1 = RxDown1./sqrt(mean(abs(RxDown1).^2))*P(BitPerSym);
%         RxDown2 = RxDown2./sqrt(mean(abs(RxDown2).^2))*P(BitPerSym);
%% BPS
BlockLen = 32;
PhaseNum = 50;
Phase = pi/180*20;
Len = length(RxDown1);
BlockNum = Len/BlockLen;
RxSymAft1 = zeros(1,Len);
RxSymAft2 = zeros(1,Len);
RxSymBef1 = reshape(RxDown1,BlockLen,[]);
RxSymBef2 = reshape(RxDown2,BlockLen,[]);
for ii = 1:BlockNum
    temp1 = RxSymBef1(:,ii);
    temp2 = RxSymBef2(:,ii);
    temp1 = temp1*exp(1j*linspace(-Phase,Phase,PhaseNum));
    temp2 = temp2*exp(1j*linspace(-Phase,Phase,PhaseNum));
%     tempD1 = qamdemod(temp1.',2^BitPerSym,'OutputType','Bit').';
%     tempD1 = qammod(tempD1.',2^BitPerSym,'InputType','Bit').';
%     tempD2 = qamdemod(temp2.',2^BitPerSym,'OutputType','Bit').';
%     tempD2 = qammod(tempD2.',2^BitPerSym,'InputType','Bit').';
    tempD1 = QAMdecision(temp1, BitPerSym);
    tempD2 = QAMdecision(temp2, BitPerSym);
    Distance1 = sum(abs(temp1-tempD1).^2,1);
    Distance2 = sum(abs(temp2-tempD2).^2,1);
    [~,ind1] = min(Distance1);
    [~,ind2] = min(Distance2);
    RxSymAft1( (ii-1)*BlockLen+1:ii*BlockLen ) = temp1(:,ind1).';
    RxSymAft2( (ii-1)*BlockLen+1:ii*BlockLen ) = temp2(:,ind2).';
end
RxSym1 = RxSymAft1;
RxSym2 = RxSymAft2;
        %% remove pilot
        PilotBlock = FrameLen;
        Plen = floor(PilotBlock/(PilotLen+1));
        RxSym1 = reshape(RxSym1,PilotBlock,[]);
        RxSym2 = reshape(RxSym2,PilotBlock,[]);
        RxSym1Head = RxSym1(1:(Plen+1)*PilotLen,:);
        RxSym2Head = RxSym2(1:(Plen+1)*PilotLen,:);
        RxSym1Tail = RxSym1((Plen+1)*PilotLen+1:end,:);
        RxSym2Tail = RxSym2((Plen+1)*PilotLen+1:end,:);
        RxSym1Head = reshape(RxSym1Head,Plen+1,[]);
        RxSym2Head = reshape(RxSym2Head,Plen+1,[]);
        RxSym1Head = RxSym1Head(1:Plen,:);
        RxSym2Head = RxSym2Head(1:Plen,:);
        RxSym1Head = reshape(RxSym1Head,Plen*PilotLen,[]);
        RxSym2Head = reshape(RxSym2Head,Plen*PilotLen,[]);
        RxSym1 = [RxSym1Head;RxSym1Tail];
        RxSym2 = [RxSym2Head;RxSym2Tail];
        RxSym1 = reshape(RxSym1,1,[]);
        RxSym2 = reshape(RxSym2,1,[]);
        % scatterplot(RxSym1);
        % scatterplot(RxSym2);



%% BER cal.
% RxBit1 = qamdemod(RxSym1.',2^BitPerSym,'OutputType','Bit').';
% RxBit2 = qamdemod(RxSym2.',2^BitPerSym,'OutputType','Bit').';
RxNum1 = qamdemod(RxSym1,M);
RxNum2 = qamdemod(RxSym2,M);
RxBit1= Num2Bit(RxNum1,BitPerSym);
RxBit2= Num2Bit(RxNum2,BitPerSym);
% abd=genqamdemod(TxSym1,const);
% [~,BER1,individual] = biterr(TxBit1,RxBit1);
% [~,BER2,individual] = biterr(TxBit2,RxBit2);
TotalBits = (FrameLen-PilotLen)*FrameNum*BitPerSym;
TxBits1 = reshape(TxBit1,1,[]);
TxBits2 = reshape(TxBit2,1,[]);
BER1 = sum(abs(TxBits1-RxBit1))/TotalBits;
BER2 = sum(abs(TxBits2-RxBit2))/TotalBits;
BER = (BER1+BER2)/2
SNR1 = 10*log10( mean(abs(TxSym1).^2)./mean(abs(TxSym1-RxSym1).^2) );
SNR1
SNR2 = 10*log10( mean(abs(TxSym2).^2)./mean(abs(TxSym2-RxSym2).^2) );
SNR2
BER_all = [BER_all BER];

 end



















