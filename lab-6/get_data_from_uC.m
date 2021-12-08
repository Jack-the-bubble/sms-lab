delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
vector_len=300;
s = serial('COM12'); % COM9 to jest port utworzony przez mikrokontroler
set(s,'BaudRate',115200);
set(s,'StopBits',1);
set(s,'Parity','none');
set(s,'DataBits',8);
set(s,'Timeout',1);
set(s,'InputBufferSize',1000);
%set(s,'Terminator',13);
fopen(s); % otwarcie kanalu komunikacyjnego
Tp = 0.01; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu
yzad = ones(vector_len,1)*1000;
for i=1:250
    yzad(i) = 200;
end
while length(y)~=vector_len % zbieramy 100 pomiarow
    txt = fgetl(s); % odczytanie z portu szeregowego
% txt powinien zawiera´c Y=%4d;U=%4d;
% czyli np. Y=1234;U=3232;
    disp(char(txt));
    eval(char(txt)); % wykonajmy to co otrzymalismy//
y=[y;Y]; % powiekszamy wektor y o element Y
u=[u;U]; % powiekszamy wektor u o element U
end
figure; plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
hold on 
plot((0:(length(y)-1))*Tp,yzad);
figure; plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie
fid = fopen('step_response_u_y_yzad.dat', 'w');
for i=1:vector_len
    fprintf(fid, "%6.2f %6.2f %6.2f\n", u(i), y(i), yzad(i));
end
%fprintf(fid, "%6.2f %6.2f %6.2f", u, y, yzad);
fclose(fid);