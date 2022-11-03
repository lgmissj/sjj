function [ Bits ] = Num2Bit(Symbol,BitPerSym)
Bits = [];
N = length(Symbol);
if BitPerSym == 2
    for ii = 1:N
       a1=mod(Symbol(ii) , 2);
       a0=floor(Symbol(ii)/2);
       b1=mod(a0 , 2);
       b0=floor(a0/2);
       Bits = [Bits b1 a1];
    end
end


if BitPerSym == 3
    for ii = 1:N
       a1=mod(Symbol(ii) , 2);
       a0=floor(Symbol(ii)/2);
       b1=mod(a0 , 2);
       b0=floor(a0/2);
       c1=mod(b0 , 2);
       c0=floor(b0/2);
       Bits = [Bits c1 b1 a1];
    end
end

if BitPerSym == 4
    for ii = 1:N
       a1=mod(Symbol(ii) , 2);
       a0=floor(Symbol(ii)/2);
       b1=mod(a0 , 2);
       b0=floor(a0/2);
       c1=mod(b0 , 2);
       c0=floor(b0/2);
       d1=mod(c0 , 2);
       d0=floor(c0/2);
       Bits = [Bits d1 c1 b1 a1];
    end
end

