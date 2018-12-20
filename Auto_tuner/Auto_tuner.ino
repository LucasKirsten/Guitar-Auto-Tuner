#include <ADC.h>
#include <stdlib.h>
#include <Fuzzy.h>
#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>
#include <AccelStepper.h>

Fuzzy *fuzzy = new Fuzzy();       // objeto do fuzzy
ADC *adc = new ADC();             // objeto do adc
IntervalTimer amostrador;         // objeto do timer
AccelStepper stepper(1, 3, 2);    // 3 - pinStep; 4 - pinDirection.



const float Fs = 50000;
const float Ts = 1000000.0 / Fs; //uS

const int TAM = 10000;
const int PEAKS = 1;

const float afinacao[6] = {329.627556913, 246.941650628, 195.997717991, 146.832383959, 110, 82.4068892282};

//****************FREQUÊNCIAS****************************************************
// START: lag mínimo para detectar frequência
// LAG: lag máximo para detectar frequência
void getFreqIdx(float* x, int START, int LAG, int* idx) {
  float r[LAG - START] = {0}, der[LAG - START - 1] = {0}, pks[PEAKS] = {0};

  for (int i = START; i < LAG; i++) {
    //autocorrelação
    r[i - START] = 0.0;
    for (int j = 0; j < TAM; j++) {
      if (j > i) r[i - START] += x[j] * x[j - i];
    }
    //Serial.print("r["); Serial.print(i); Serial.print("]="); Serial.println(r[i]);
    //derivada
    if (i - START > 0) der[i - START - 1] = r[i - START] - r[i - START - 1];
    //índice dos picos
    if (i - START > 2) {
      //Serial.print("der["); Serial.print(i-1); Serial.print("]="); Serial.println(der[i-1]);
      if (r[i - START - 2] > pks[0] && (der[i - START - 3] > 0 && der[i - START - 1] < 0)) { //verifica se há pico
        for (int k = PEAKS - 1; k >= 0; k--) { //varre do maior ao menor pico
          if (r[i - START - 2] > pks[k]) { //encontrou posição do pico
            for (int l = 1; l <= k; l++) { //realoca a posição dos picos
              pks[l - 1] = pks[l];
              idx[l - 1] = idx[l];
            }
            pks[k] = r[i - START - 2];
            idx[k] = i - 2;
            break;
          }
        }
      }
    }
  }
}
// ***************************************************************************

//**************CÁLCULOS ESTATÍSTICOS****************************************
// média
float mean(float* x, int SIZE) {
  float total = 0.0;
  for (int i = 0; i < SIZE; i++) total += x[i];
  return total / SIZE;
}

// desvio padrão
float deviation(float M, float* x, int SIZE) {
  float total = 0.0;
  for (int i = 0; i < SIZE; i++) total += (x[i] - M) * (x[i] - M);
  return sqrt(total / (SIZE - 1));
}

// critério de Chauvenet’s
float chauvenets(float tau, float* x, int SIZE) {
  float M = mean(x, SIZE);
  float S = deviation(M, x, SIZE);

  float t = 0.0; int n = 0;
  if (S <= 0.0) return x[0]; //std=0 : todas as amostras são iguais
  for (int i = 0; i < SIZE; i++) {
    if ( ( abs(x[i] - M) / S ) < tau ) {
      t += x[i];
      n++;
    }
  }
  return t / n;
}
//***************************************************************************

// ****************AMOSTRAGEM*****************************
volatile uint16_t sample;
float amostras[TAM] = {0.0};
volatile float total = 0.0;
volatile int it = 0;
volatile int amostrou = 0, janelaValida = 0;

float thresh = 3000;
void getSample() {
  sample = (uint16_t)adc->analogRead(A18, ADC_1);
  if (sample > thresh) janelaValida = 1;
  //sample=(uint16_t)(sine(2*pi*it*147/Fs)+1)*2048;
  if (it < TAM && janelaValida) {
    amostras[it] = (float)(sample - 2048) / 4096; //*hann(it);
    it++;
  }
  if (it >= TAM - 1) {
    amostrou = 1;
    janelaValida = 0; total = 0; it = 0;
  }
}
// **********************************************************

// ****************CONFIGURAÇÃO ADC,DAC e TIMER**************
void configADC() {
  // ADC_1
  pinMode(A18, INPUT);
  adc->setReference(ADC_REFERENCE::REF_EXT, ADC_1); //REF_EXT
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1);
  adc->setResolution(12, ADC_1);
  adc->setAveraging(8, ADC_1);
}
// inicia o timer para fazer as amostragens
void initTimer() {
  amostrador.begin(getSample, Ts);
  amostrador.priority(0);
}
// **********************************************************

/*****************************CONFIGURACAO DO FUZZY***************************/
void configFuzzy(void)
{
  FuzzyInput *erro = new FuzzyInput(1); // Entrada do sistema

  //Fuzzy Sets (funcoes de pertinencia) da entrada.
  FuzzySet *negativo = new FuzzySet(-300, -300, -100, -10);    //fc saturacao esquerda.
  erro->addFuzzySet(negativo);                            // Adiciona a entrada.
  FuzzySet *zeroIn = new FuzzySet(-100, -10, 10, 100);         //fc triangular centrada em zero.
  erro->addFuzzySet(zeroIn);                              // Adiciona a entrada.
  FuzzySet *positivo = new FuzzySet(10, 100, 300, 300);       // fc saturacao direita.
  erro->addFuzzySet(positivo);                            // Adiciona a entrada.

  fuzzy->addFuzzyInput(erro);                             // Adiciona a entrada ao objeto.

  FuzzyOutput *rpm = new FuzzyOutput(1);                  // saida do sistema.

  //Fuzzy Sets da saida
  FuzzySet *rapAntHorario = new FuzzySet(-100, -50, -50, 0);     // fc triangular centrada em -1.
  rpm->addFuzzySet(rapAntHorario);                           // adiciona a saida
  FuzzySet *zeroOut = new FuzzySet(-50, 0, 0, 50);             // fc triangular centrada em zero
  rpm->addFuzzySet(zeroOut);                                 // adiciona a saida
  FuzzySet *rapHorario = new FuzzySet(0, 50, 50, 100);           // fc triangular centrada em 1
  rpm->addFuzzySet(rapHorario);                              // adiciona a saida

  fuzzy->addFuzzyOutput(rpm);                                // Adiciona a saida ao objeto

  //Criando as regras fuzzy.
  // 1. Se Erro = negativo entao rpm = rapanthorario
  FuzzyRuleAntecedent *ifErroNeg = new FuzzyRuleAntecedent(); //Adicionando a premissa
  ifErroNeg->joinSingle(negativo);                             // Adicionando o set correspondente
  FuzzyRuleConsequent *thenRPMRapHorario = new FuzzyRuleConsequent(); // Adicionando a consequencia
  thenRPMRapHorario->addOutput(rapHorario);                           // Adicionando o set correspondente

  // Instanciar objeto FuzzyRule
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifErroNeg, thenRPMRapHorario);

  fuzzy->addFuzzyRule(fuzzyRule01);                           // Adicionando o FuzzyRule ao objeto

  // 2. Se Erro = Positivo ENTAO rpm=rapHorario
  FuzzyRuleAntecedent *ifErroPos = new FuzzyRuleAntecedent(); // Adicionando a premissa
  ifErroPos->joinSingle(positivo);                            // Adicionando o set correspondente
  FuzzyRuleConsequent *thenRPMRapAntHorario = new FuzzyRuleConsequent(); //Adicionando a Consequencia
  thenRPMRapAntHorario->addOutput(rapAntHorario);                        // Adicionando o set correspondente

  //Instanciar o objeto FuzzyRule da segunda regra
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifErroPos, thenRPMRapAntHorario);

  fuzzy->addFuzzyRule(fuzzyRule02);                                    // Adicionando ao objeto Fuzzy.

  // 3. Se Erro = zeroIn entao RPM = zeroOut
  FuzzyRuleAntecedent *ifErroZeroIn = new FuzzyRuleAntecedent();      // Adicionando A premissa.
  ifErroZeroIn->joinSingle(zeroIn);                                   // Adicionando o Set correspondente
  FuzzyRuleConsequent *thenRPMZeroOut = new FuzzyRuleConsequent();    // Adicionando a consequencia
  thenRPMZeroOut->addOutput(zeroOut);                                 // Adicionando o set conrrespondente

  // Instanciar o objeto FuzzyRule da terceira regra
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifErroZeroIn, thenRPMZeroOut);

  fuzzy->addFuzzyRule(fuzzyRule03);                                 // adicionando a ultima regra ao objeto fuzzy
}
/*----------------------------------------------------------------------------*/

void setup() {
  Serial.begin(2000000);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(100.0);
  //sinaliza inicio das amostragens (pisca o led 3 vezes) e matém aceso
  pinMode(LED_BUILTIN, OUTPUT);
  for (int j = 0; j < 8; j++) {
    if (j % 2 == 0) digitalWrite(LED_BUILTIN, LOW);
    else digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }

  configADC();
  Serial.println("ADC configurado...");
  initTimer();
  Serial.println("TIMER configurado...");
  configFuzzy();
  Serial.println("FUZZY configurado...");

}

int idx[PEAKS] = {0};
float fund[5] = {0.0}; int f = 0;
int corda = 6; float minFreq = 48, maxFreq = 150;
long iTime;
long fTime;
float fAmostra, cent;
int girarMotor=0;
int afinado = 0;
float freq;
void loop() {
  if (Serial.available()) {
    noInterrupts();
    corda = Serial.parseInt() - 1;
    if (corda == 0 || corda == 1 || corda == 2 || corda == 3 || corda == 4 || corda == 5) {
      if (corda==5 || corda == 4 || corda == 3) thresh = 3000;
      else thresh = 2800;
      Serial.print("\nCorda: "); Serial.println(corda + 1);
      freq = afinacao[corda];
      minFreq = freq/1.189207115; maxFreq = freq*1.189207115;
      // reinicia cálculos
      afinado=0; amostrou = 0; f = 0; it = 0;
    }
    interrupts();
  }
  if (amostrou) {
    //Serial.print("amostrado..."); Serial.print(f+1); Serial.print("/5");
    noInterrupts();

    iTime = micros();
    getFreqIdx(amostras, (int)Fs / maxFreq, (int)Fs / minFreq, idx);
    fTime =  micros();
    
    //Serial.print("TEMPO: "); Serial.print((fTime - iTime) / 1000.0); Serial.println("ms");

    //fAmostra = (float)Fs / idx[0];
    
    amostrou = 0;

    fund[f++] = (float)Fs/idx[0];
    //Serial.print("f="); Serial.println((float)Fs/idx[0]);
    if (f>=5) {
      fAmostra = chauvenets(1.65, fund, 5); Serial.println(fAmostra);
      cent = 1200*log(fAmostra/freq)/log(2);
      girarMotor=1;
      // curva de calibracao
      //fAmostra = 1.01*fAmostra-0.652;
      Serial.print("FREQ "); Serial.print(fAmostra); Serial.println("Hz");
      Serial.print("CENTS "); Serial.println(cent);
      f = 0; it = 0;
     }
     interrupts();
  }
  if (girarMotor && afinado<3) {
    girarMotor=0;
    noInterrupts();
    float err = cent;
    //Serial.print("Erro:");
    //Serial.println(err);
    fuzzy->setInput(1, err);          // seta a entrada que tem ID 1 (erro)
    fuzzy->fuzzify();                 // executa a fuzzificacao
    float sps = fuzzy->defuzzify(1);     // retorna a saida em stepsPerSeconds/100.
    
    // verifica há 3 valores seguidos de sps=0 para corda afinada
    if ((int)sps==0) afinado++;
    else afinado=0;
    if (afinado==3) Serial.println("Corda afinada!");
    
    //Serial.print("SPS");
    //Serial.println(sps);
    stepper.move(round(sps));       // passa a variavel pra entrada do motor. Type casting pra int.
    //Serial.print("INTSPS");
    //Serial.println((int)sps);
    stepper.runToPosition();
    interrupts();
    Serial.println("----------------\n");
  }
}
