// Código para os Arduinos Pro Mini (Controlador de Motor de Passo)
#include <Wire.h>

#define DEBUG true
#define CALIBRA_AUTO true  //Se verdadeiro, a calibração das posições será feita automaticamente com a média entre a posição subindo e a descendo

#define pinMotor1 9
#define pinMotor2 8
#define pinMotor3 7
#define pinMotor4 6

// Defina o endereço I2C deste Pro Mini
#define I2C_ADDRESS 0x10  // Controlador do Motor de Passo: endereço 0x10

int portasMotor[4] = { pinMotor1, pinMotor2, pinMotor3, pinMotor4 };

int sequencia[8][4] = {
  { HIGH, LOW, LOW, HIGH },  // Passo 1
  { HIGH, LOW, LOW, LOW },   // Passo 2
  { HIGH, HIGH, LOW, LOW },  // Passo 3
  { LOW, HIGH, LOW, LOW },   // Passo 4
  { LOW, HIGH, HIGH, LOW },  // Passo 5
  { LOW, LOW, HIGH, LOW },   // Passo 6
  { LOW, LOW, HIGH, HIGH },  // Passo 7
  { LOW, LOW, LOW, HIGH }    // Passo 8
};

int passo = 0;
unsigned long delayPasso;

//Variáveis de controle da movimentação do motor
int estado = 0;        //0=parado  1=girando
int sentido = 0;       //0=sobe    1=desce
int velocidade = 240;  //(de 0-parado até 255-velocidade máxima)

//Variáveis de controle das posições previamente gravadas
int iniciado = false;
int funcao = 0;  //0=direto 1=automatico (com posicoes previamente gravadas)
unsigned long posicao = 0;
unsigned long posicoes[3][32];  //{subindo, descendo, calibrado} por andar
unsigned long posicaoDestino = 0;

//Bytes de comando:
//Inicialização = 0xC0 (B11000000)

//Estrutura do byte de comando:
//
//  Posição = 76543210
//           B00000000
//
//  Byte 7 = funcao  -> 0=direto 1=automatico
//
//  para funcao=0:
//  Byte 6 = estado  -> 0=parado 1=girando
//  Byte 5 = sentido -> 0=sobe   1=desce
//  Bytes 43210 = velocidade  -> de 0=parado até 31=rápido
//
//  para funcao=1:
//  Bytes 65    = acao    -> 00=movePara 01=calibra 10=gravaDescendo 11=gravaSubindo
//  Bytes 43210 = memoria -> de 0 até 31

//Variáveis de controle da comunicação I2C
char erro = 0;

void setup() {
  pinMode(pinMotor1, OUTPUT);
  pinMode(pinMotor2, OUTPUT);
  pinMode(pinMotor3, OUTPUT);
  pinMode(pinMotor4, OUTPUT);

  Wire.begin(I2C_ADDRESS);  // Inicializa o I2C com o endereço específico
  Wire.onReceive(recebeDados);
  Wire.onRequest(enviarDados);

#if DEBUG
  Serial.begin(9600);
  Serial.println("DEBUG Iniciado");
#endif
}

void loop() {
  if (((micros() - delayPasso) > (750 + ((255 - velocidade) * 36.27451))) && (velocidade > 0) && (estado != 0)) {
    delayPasso = micros();

    if (funcao == 1) {
      sentido = (posicao > posicaoDestino);
      estado = (posicao != posicaoDestino);
    }

    if (!sentido) {
      posicao++;
      passo++;
      if (passo > 7) passo = 0;
    } else {
      posicao--;
      passo--;
      if (passo < 0) passo = 7;
    }

    for (int nL = 0; nL < 4; nL++) {
      digitalWrite(portasMotor[nL], sequencia[passo][nL]);
    }
  }
}

void recebeDados(int num_bytes) {
  int command = Wire.read();

  if (command != -1) {
    bool acaoCmd = bitRead(command, 7);
    bool estadoCmd = bitRead(command, 6);
    bool sentidoCmd = bitRead(command, 5);
    int valorCmd = (command & 0x1F);

#if DEBUG
    Serial.print(F(" acao:"));
    Serial.print(acaoCmd);
    Serial.print(F(" estado:"));
    Serial.print(estadoCmd);
    Serial.print(F(" sentido:"));
    Serial.print(sentidoCmd);
    Serial.print(F(" valor:"));
    Serial.println(valorCmd);
#endif

    if (!acaoCmd) {
      //Comando de acionamento direto do motor
      estado = estadoCmd;
      sentido = sentidoCmd;
      velocidade = (valorCmd * 8.22);  //Converte a velocidade de 0 a 31 para a faixa de 0 a 255
      funcao = 0;

#if DEBUG
      Serial.print(F("funcao:direto"));
      Serial.print(F(" posicao:"));
      Serial.println(posicao);
#endif

    } else {
      //Comando de controle por posições pré-gravadas
      if (!estadoCmd && !sentidoCmd) {
        //Comando para mover para uma posição pré-definida

        if (iniciado) {
          if ((valorCmd == 0) || ((valorCmd != 0) && (posicoes[2][valorCmd] != 0))) {
            posicaoDestino = posicoes[2][valorCmd];

            sentido = (posicao > posicaoDestino);
            estado = (posicao != posicaoDestino);
            funcao = 1;  //movimento automatico com posição previamente gravada

#if DEBUG
            Serial.print(F("funcao:auto"));
            Serial.print(F(" posicaoDestino:"));
            Serial.print(posicaoDestino);
            Serial.print(F(" sentido:"));
            Serial.print(sentido);
            Serial.print(F(" estado:"));
            Serial.println(estado);
#endif
          }
        } else {
          erro = erro & 0x08;  //comando para mover para uma posicao, sem o reset prévio

#if DEBUG          
          Serial.println(F(" erro:0x08"));
#endif
        }
      }

      if (!estadoCmd && sentidoCmd) {
        //Comando para calibrar uma posição pré-definida
        if (iniciado) {
          posicoes[2][valorCmd] = posicao;

#if DEBUG   
          Serial.print(F("funcao:calibra"));
          Serial.print(F(" posicao:"));
          Serial.println(posicao);
#endif
        } else {
          erro = erro & 0x04;  //comando para calibrar uma posicao, sem o reset prévio

#if DEBUG             
          Serial.println(F(" erro:0x04"));
#endif
        }
      }

      if (estadoCmd) {
        //Comando para gravar uma posição
        if (!sentidoCmd) {
          //Gravar posicao descendo
          if (valorCmd == 0) {  //Função de inicialização (ao definir a posicao 0) = byte 0xC0
            posicao = 0;
            iniciado = true;
            erro = 0;

#if DEBUG   
            Serial.println(F("funcao:iniciado"));
#endif
          }

          if (iniciado) {
            posicoes[1][valorCmd] = posicao;

#if CALIBRA_AUTO
            if (posicoes[0][valorCmd] != 0) {
              posicoes[2][valorCmd] = posicoes[0][valorCmd] + ((posicoes[1][valorCmd] - posicoes[0][valorCmd]) / 2);
            } else {
              posicoes[2][valorCmd] = posicoes[1][valorCmd];
            }
#endif

#if DEBUG   
            Serial.print(F("funcao:grava"));
            Serial.print(F(" memoria:"));
            Serial.print(valorCmd);
            Serial.print(F(" posicao:"));
            Serial.println(posicao);
            Serial.print(F(" posicoes[0][memoria]:"));
            Serial.print(posicoes[0][valorCmd]);
            Serial.print(F(" posicoes[1][memoria]:"));
            Serial.print(posicoes[1][valorCmd]);
            Serial.print(F(" posicoes[2][memoria]:"));
            Serial.println(posicoes[2][valorCmd]);
#endif
          } else {
            erro = erro & 0x01;  //comando para gravar uma posicao descendo, sem o reset prévio
#if DEBUG   
            Serial.println(F(" erro:0x01"));
#endif
          }
        } else {
          //Gravar posicao subindo
          if (iniciado) {
            posicoes[0][valorCmd] = posicao;

#if CALIBRA_AUTO
            if (posicoes[1][valorCmd] != 0) {
              posicoes[2][valorCmd] = posicoes[0][valorCmd] + ((posicoes[1][valorCmd] - posicoes[0][valorCmd]) / 2);
            } else {
              posicoes[2][valorCmd] = posicoes[0][valorCmd];
            }
#endif

#if DEBUG   
            Serial.print(F("funcao:grava"));
            Serial.print(F(" memoria:"));
            Serial.print(valorCmd);
            Serial.print(F(" posicao:"));
            Serial.println(posicao);
            Serial.print(F(" posicoes[0][memoria]:"));
            Serial.print(posicoes[0][valorCmd]);
            Serial.print(F(" posicoes[1][memoria]:"));
            Serial.print(posicoes[1][valorCmd]);
            Serial.print(F(" posicoes[2][memoria]:"));
            Serial.println(posicoes[2][valorCmd]);
#endif
          } else {
            erro = erro & 0x02;  //comando para gravar uma posicao subindo, sem o reset prévio

#if DEBUG   
            Serial.println(F(" erro:0x02"));
#endif
          }
        }
      }
    }
  }
}

void enviarDados() {
  Wire.write(erro);  //nada definido para retornar
}