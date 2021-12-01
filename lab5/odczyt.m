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

y(1:100) = -1; % mikrokontroler zwraca tylko dodatnie wartosci, wiec kazde ?1
% oznacza, ze nie otrzymalismy jeszcze wartosci o tym indeksie
while true
    txt = fread(s,22); % odczytanie z portu szeregowego
    disp(char(txt'));
    eval(char(txt')); % wykonajmy to co otrzymalismy
    if(y(1) ~= -1)
        break;
    end
end

y(2:100) = -1; % ignorujemy wszystko co odczytalismy poza pierwszym elementem
i = 2;

while true
    txt = fread(s,22); % odczytanie z portu szeregowego
    eval(char(txt')); % wykonajmy to co otrzymalismy
    y(i) = Y;
    if(i == 100)
        break; 
    end
    i = i+1;
end

figure;
plot(0:length(y),y); % w tym momencie mozna juz wyrysowac dane
