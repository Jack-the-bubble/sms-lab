delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;

s = serial('COM11'); % COM11 to jest port utworzony przez mikrokontroler
set(s,'BaudRate',115200);
set(s,'StopBits',1);
set(s,'Parity','none');
set(s,'DataBits',8);
set(s,'Timeout',1);
set(s,'InputBufferSize',1000);
set(s,'Terminator',13);
fopen(s); % otwarcie kanalu komunikacyjnego

Tp = 0.05; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu

while length(y)~=200 % zbieramy 100 pomiarow
    txt = fread(s,22); % odczytanie z portu szeregowego
    % txt powinien zawieraŽc Y=%4d;U=%4d;
    % czyli np. Y=1234;U=3232;
    disp(char(txt'));
    eval(char(txt')); % wykonajmy to co otrzymalismy
    if U==0.00
        flag = 1;
    end
    if flag == 1
        y=[y;Y]; % powiekszamy wektor y o element Y
        u=[u;U]; % powiekszamy wektor u o element U
    end
end
figure; plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
figure; plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie
