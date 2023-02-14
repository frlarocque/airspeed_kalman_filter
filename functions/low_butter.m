function [arr] = low_butter(arr,filter_freq,sf,debug,order)
%HIGH_BUTTER Apply a Low pass butter filter to eliminate high frequency
%noise
%
% Inputs:
%           -arr: signal
%           -filter_freq: filter low pass frequency [Hz]
%           -sf: sampling frequency [Hz]
%           -order: order of filter

[b,a] = butter(order,filter_freq/(sf/2),'low');

arr = filter(b,a,arr);
if debug==1
    figure (200)
    clf
    %freqz(b,a)
    transfer=tf(b,a,1/sf);
    h=bodeplot(transfer);
    setoptions(h,'FreqScale','linear','FreqUnits','Hz','Grid','on','PhaseVisible','off');
    %setoptions(h,'FreqScale','linear','Grid','on','PhaseVisible','off');
    %grid on
    %grid minor
    %transfer2=c2d(transfer,1/sf)
    figure(201)
    clf
    transfer1=tf(b1,a1,1/sf);
    transfer2=tf(b2 ,a2,1/sf);
    transfer3=tf(b3,a3,1/sf);
    transfer4=tf(b4,a4,1/sf);
    transfer5=tf(b5,a5,1/sf);
    transfer6=tf(b6,a6,1/sf);
    h=bodeplot(transfer1,transfer2,transfer3,transfer4,transfer5,transfer6);
    setoptions(h,'FreqScale','linear','FreqUnits','Hz','Grid','on','PhaseVisible','off')
    legend("0.1","0.5","1","2","3","4")

end
end

