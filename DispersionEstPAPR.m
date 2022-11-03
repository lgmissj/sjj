function [ EstDistance ] = DispersionEstPAPR( RxWfm1, RxWfm2, NNN, FiberLen_Test, SampleRateDefault )

RxWfmTemp1 = RxWfm1(1:NNN).';
RxWfmTemp2 = RxWfm2(1:NNN).';

C_speed = 3e8;
% FiberLen_Test = [0:5:1000]*1e3;   %
% SampleRateDefault = BaudRate*Up2;
lamda = C_speed/193.1e12;
CD_value = 17e-6 * FiberLen_Test;
N = length(RxWfmTemp1);
TimeW = N/SampleRateDefault;
beta2 = -lamda^2*CD_value/2/pi/C_speed;
w = 2*pi*[(0:N/2-1),(-N/2:-1)].'/TimeW;
TxWfmFFT1 = fft(RxWfmTemp1);
TxWfmFFT1 = TxWfmFFT1.*exp(1i.*beta2.*(w.^2)/2);
RxWfmTemp1_Comp = ifft(TxWfmFFT1);
TxWfmFFT2 = fft(RxWfmTemp2);
TxWfmFFT2 = TxWfmFFT2.*exp(1i.*beta2.*( w.^2)/2);
RxWfmTemp2_Comp = ifft(TxWfmFFT2);
RxWfm_Comp = abs(RxWfmTemp1_Comp).^2+abs(RxWfmTemp2_Comp).^2;
PAPR_all = 10*log10(max(RxWfm_Comp)./mean(RxWfm_Comp));
plot(FiberLen_Test, PAPR_all);
[~, MinPos] = min(PAPR_all);
EstDistance = FiberLen_Test(MinPos)/80e3;

end

