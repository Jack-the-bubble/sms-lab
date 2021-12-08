#include <stdio.h>

float multipleVectors(float Ku[], float Upast[], int length) {
  uint16_t i = 0;
  float suma = 0.0f;
  for (i = 0; i < length; ++i) {
    suma = suma + Ku[i] * Upast[i];
  }
  return suma;
}

int main(void) {
  // -------------- PARAMETRY ---------------
  uint16_t time = 1000;  // czas pomiaru
  uint16_t D = 60;       // horyzont dynamiki
  float ke = 0.0068f;    // Ke
  float Ku[] = {
      0.0011f, 0.0014f, 0.0015f, 0.0016f, 0.0016f, 0.0016f, 0.0016f, 0.0015f,
      0.0014f, 0.0013f, 0.0013f, 0.0012f, 0.0011f, 0.0010f, 0.0009f, 0.0009f,
      0.0008f, 0.0007f, 0.0007f, 0.0006f, 0.0006f, 0.0005f, 0.0004f, 0.0004f,
      0.0004f, 0.0003f, 0.0003f, 0.0003f, 0.0002f, 0.0002f, 0.0002f, 0.0002f,
      0.0002f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f,
      0.0001f, 0.0000f, 0.0001f, 0.0001f, 0.0000f, 0.0001f, 0.0000f, 0.0000f,
      0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
      0.0000f, 0.0000f, 0.0000f};

  // -------------- DO REGULACJI ---------------

  float y = 0.0f;
  float u = 0.0f;

  float yzad = 0.0f;   // najpierw -500, po 1s 500
  float ypast = 0.0f;  // poprzednia wartosc wyjscia
  float upast = 0.0f;  // poprzednia wartosc sterowania
  float e = 0.0f;      // uchyb

  // zmienne obliczeniowe
  float ue = 0.0f;
  float uu = 0.0f;

  float a = 3.026e-05;  // parametry rownania roznicowego
  float b = 3.009e-05;
  float c = 1.983f;
  float d = 0.9831f;

  float U[time];
  float Y[time];
  float Upast[D - 1];  // wektor przeszlych przyrostow sterowan

  int i;
  for (i = 0; i < time - 1; ++i) {
    U[i] = 0;
    Y[i] = 0;

    if (i < D) Upast[i] = 0;
  }

  int licznik = 2;

  while (licznik < time) {
    if (licznik < 500.0f) {
      yzad = -500.0f;
      ++licznik;
    }

    else {
      yzad = 500.0f;
      ++licznik;
    }

    ypast = y;
    upast = u;

    y = a * U[licznik - 1] + b * U[licznik - 2] + c * Y[licznik - 1] -
        d * Y[licznik - 2];
    Y[licznik] = y;

    e = yzad - y;

    ue = ke * e;
    uu = multipleVectors(Ku, Upast, D - 1);

    u = upast + ue - uu;

    if (u > 2047.0f) u = 2047.0f;

    if (u < -2048.0f) u = -2048.0f;

    U[licznik] = u;

    // Przesowanie o 1 w prawo
    for (i = 0; i < time - 1; ++i) U[i + 1] = U[i];

    Upast[0] = u - upast;

    licznik = licznik + 1;
  }

  return 0;
}

