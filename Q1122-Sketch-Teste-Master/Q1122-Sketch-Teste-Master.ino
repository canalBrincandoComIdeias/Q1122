// Código para os Arduino Master (Teste do acionamento de motor de passo)
#include <Wire.h>

#define RODA_SCRIPT true

char comando = 0;
char retorno = 0;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.begin();  //Modo Master
  Serial.begin(9600);
  while (!Serial);

#if RODA_SCRIPT
  uint8_t comandos[] = {
    B11000000,  //Grava posicao 0 //Reset
    B11000000,  //Grava posicao 0 //Reset
    B01011110,  //Sobe
    B00011110,  //Para
    B11100001,  //Grava posicao 1
    B01011110,  //Sobe
    B00011110,  //Para
    B11100010,  //Grava posicao 2
    B01011110,  //Sobe
    B00011110,  //Para
    B11100011,  //Grava posicao 3
    B01111100,  //Desc
    B00011110,  //Para
    B11000010,  //Grava posicao 2
    B01111110,  //Desc
    B00011110,  //Para
    B11000001,  //Grava posicao 1
    B10000011,  //Vai para posicao 3
    B00011110,  //Para
    B10000001,  //Vai para posicao 1
    B00011110,  //Para
    B10000010,  //Vai para posicao 2
    B00011110,  //Para
    B10000000   //Vai para posicao 0
  };
 
  delay(1000);
  digitalWrite(13, LOW);

  for (int nL = 0; nL < 24; nL++) {
    Wire.beginTransmission(0x10);
    Wire.write(comandos[nL]);
    Wire.endTransmission();
    delay(500);
  }

#endif
}

void loop() {

  if (Serial.available()) {
    String strComando = Serial.readString();

    //acao=
    //estado=
    //sentido=
    //valor=
    //enter=
    //comando=  11000000 (reset)
    int igualPos = strComando.indexOf('=');
    String cmd = strComando.substring(0, igualPos);
    String val = strComando.substring(igualPos + 1);

    Serial.print("cmd:");
    Serial.print(cmd);
    Serial.print(":");
    Serial.println(val);

    if (cmd == "acao") {
      bitWrite(comando, 7, val.toInt());
    }

    if (cmd == "estado") {
      bitWrite(comando, 6, val.toInt());
    }

    if (cmd == "sentido") {
      bitWrite(comando, 5, val.toInt());
    }

    if (cmd == "valor") {
      int vComando = val.toInt();
      for (int nL = 0; nL < 5; nL++) {
        bitWrite(comando, nL, bitRead(vComando, nL));
      }
    }

    if (cmd == "enter") {
      Wire.beginTransmission(0x10);
      Wire.write(comando);
      Wire.endTransmission();
    }

    if (cmd == "comando") {
      for (int nL = 0; nL < 8; nL++) {
        String charBit = val.substring(7 - nL, 8 - nL);
        bitWrite(comando, nL, charBit.toInt());
      }

      Wire.beginTransmission(0x10);
      Wire.write(comando);
      Wire.endTransmission();
    }
  }

  Wire.requestFrom(0x10, 1);
  if (Wire.available()) {
    char command = Wire.read();
    if ((command != 0) && (command != retorno)) {
      retorno = command;
      Serial.print("ret:");
      Serial.println(retorno);
    }
  }
}