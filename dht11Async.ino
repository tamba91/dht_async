#define DHT_INT_PIN 3

#define temp() STRUCT_dhtData.i8DhtTempHigh + (float)STRUCT_dhtData.i8DhtTempLow / 10
#define humi() STRUCT_dhtData.i8DhtHumiHigh + (float)STRUCT_dhtData.i8DhtHumiLow / 10

/*variabili globali per il controllo e la lettura del sensore DHT11*/

volatile uint8_t dhtBuffer[5];

volatile struct{
  int8_t i8DhtTempLow;
  int8_t i8DhtTempHigh;
  int8_t i8DhtHumiLow;
  int8_t i8DhtHumiHigh;
}STRUCT_dhtData;

volatile int8_t bitNumber = 0;
volatile uint8_t pulseCounter = 0;
volatile bool dhtBusy = false;
volatile uint8_t dhtAck = 0;
volatile bool dhtWait = false;

void(*dhtSuccessCallback)() = NULL;
void(*dhtFailureCallback)() = NULL;

/*variabili globali utilizzate "dall'utente"*/
volatile uint32_t millisInterval;
volatile boolean newData = false;

/*funzioni per il controllo e la lettura del sensore DHT11*/

//funzione che setta i registri del timer2 per ottenere un overflow del timer e quindi un interrupt dopo 16.3 ms circa (il massimo con prescaler a 1024)
void startMaxDelayTimer2() {    
  TCCR2B = 0x00; //&= ~(1 << WGM22); //timer funzionamento normale, no PWM o CTC + timer fermo
  TIMSK2 = 0x00; //disabilita tutti gli interrupt sul timer 2
  TCCR2A = 0x00; //&= ~((1 << WGM21) | (1 << WGM20)); //timer funzionamento normale, no PWM o CTC
  TCNT2 = 0x00;  //azzera il timer2
  TIFR2 |= (1 << TOV2); //clear interrupt flag (pending overflow)
  TIMSK2 |= (1 << TOIE2); //abilita l'interrupt per il solo overflow del timer2 (Timer Overflow Interrupt Enable)
  TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20)); // avvia il timer con prescaler a 1024
}

//funzione di interrupt dell'overflow del timer2 
ISR(TIMER2_OVF_vect) {
    if(dhtBusy && !dhtWait && !dhtAck){  //overflow del timer2 al termine del "pulldown" di circa 16.3 millisecondi del pin utilizzato come trigger
      TCCR2B = 0x00; //&= ~((1 << CS22)  | (1 << CS21) | (1 << CS20)); // ferma il timer
      TCNT2 = 0x00;  //azzera il timer2
      TCCR2B |= ((1 << CS21) | (1 << CS20)); // avvia il timer con prescaler a 32
      dhtWait = true;
      pinMode(DHT_INT_PIN, INPUT_PULLUP);  
      attachInterrupt(digitalPinToInterrupt(DHT_INT_PIN), dhtSerialInputIsr, FALLING);  // interrupt generato dal sensore durante la trasmissione dati
    }
    else {           //si è verificato un timeout e il timer2 è andato in overflw, il sensore non risponde o ha smesso di rispondere o un bit è andato perduto 
      TIMSK2 = 0x00; //disabilita tutti gli interrupt sul timer 2
      detachInterrupt(digitalPinToInterrupt(DHT_INT_PIN)); // ritorno alla condizione di partenza
      pinMode(DHT_INT_PIN, OUTPUT);
      digitalWrite(DHT_INT_PIN, HIGH);
      dhtBusy = false; 
      if(dhtFailureCallback)    //se l'utente definisce una funzione di callback in caso di fallimento viene chiamata
        dhtFailureCallback();
   }
}

//funzione di interrupt per leggere i bit seriali trasmessi dal sensore, è un po' lunga ma solo una piccola parte del codice viene eseguita ad ogni chiamata
void dhtSerialInputIsr() {
  TCCR2B = 0x00; //&= ~((1 << CS22)  | (1 << CS21) | (1 << CS20)); // ferma il timer  
  uint8_t timerValue = TCNT2; //leggi il timer
  TCNT2 = 0x00; //azzera il timer 2
  TCCR2B |= ((1 << CS21) | (1 << CS20)); // (ri)avvia il timer con prescaler a 32
  
  switch(timerValue) {
    case 35 ... 43:                                       //bit 0, non fare nulla, il buffer di destinazione viene già settato a 0 con memset quando viene chiesta una misurazione
      --bitNumber;
      ++pulseCounter;
      break;   
    case 58 ... 64:
      dhtBuffer[bitNumber >> 3] |= 1 << (bitNumber & 7);  //bit 1, setta a 1 il bit corrente nel buffer equivale a dhtBuffer[bitNumber / 8] |= 1 << (bitNumber % 8)
      --bitNumber;
      ++pulseCounter;
      break;
    case 65 ... 86:                                       //acknowledgment ricevuto, dalla prossima chiamata arriveranno i 40 bit di dati (dal 39 allo 0, è "big endian")
      ++dhtAck;
      bitNumber = 39;
      ++pulseCounter;
      break;  
    case 0 ... 22:              //garbage? non specificato nel datasheet, è il primo impulso ricevuto ancora prima dell'acknowledgment, sarebbe da vedere con oscilloscopio cos'è 
      dhtWait = false;
      ++pulseCounter;
      break;
    default:                      //segnale non riconosciuto, non identificabile come 0 o 1 o ack o "garbage", lettura del sensore fallita prima del termine della trasmissione 
      bitNumber = -1;
      dhtAck = 0;
      break;
  }
  
  if(bitNumber == -1 || pulseCounter >= 45){              //se ho ricevuto tutti i 40 bit o più di 45 impulsi generici smetto di ascoltare gli interrupt 
      TIMSK2 = 0x00; //disabilita tutti gli interrupt sul timer 2
      detachInterrupt(digitalPinToInterrupt(DHT_INT_PIN)); 
      pinMode(DHT_INT_PIN, OUTPUT);
      digitalWrite(DHT_INT_PIN, HIGH);
      dhtBusy = false;
      if(bitNumber == -1 && dhtAck == 1 && (dhtBuffer[0] == (int8_t)(dhtBuffer[1] + dhtBuffer[2] + dhtBuffer[3] + dhtBuffer[4]))){ //ho ricevuto l'ack, 40 bit e la checksum è corretta
        STRUCT_dhtData.i8DhtTempLow = dhtBuffer[1];      //scrivo i valori dal buffer di ricezione alla struttura dati dedicata in questo modo il buffer può ricevere un'atra misurazione senza sovrascirvere dati validi
        STRUCT_dhtData.i8DhtTempHigh = dhtBuffer[2];
        STRUCT_dhtData.i8DhtHumiLow = dhtBuffer[3];
        STRUCT_dhtData.i8DhtHumiHigh = dhtBuffer[4];
        if(dhtSuccessCallback)          //se l'utente definisce una funzione di callback in caso di ricezione positiva viene chiamata
          dhtSuccessCallback();
      } 
      else                             //qualcosa è andato storto
        if(dhtFailureCallback)         //se l'utente definisce una funzione di callback in caso di fallimento viene chiamata
          dhtFailureCallback();
  }
} 

//funzione che avvia la misurazione, prende in input il valore di millisecondi tra una misurazione e l'altra
void dhtMeasure(uint32_t dhtInterval) {
  static uint32_t prev;
  dhtInterval = dhtInterval < 1000 ? 1000 : dhtInterval;      //il sensore dht11 richiede almeno 1 secondo tra una misurazione e l'altra
  if(!dhtBusy && ((uint32_t)(millis() - prev) >= dhtInterval)){
    pinMode(DHT_INT_PIN, OUTPUT);
    digitalWrite(DHT_INT_PIN, LOW);
    prev = millis();
    dhtBusy = true;
    dhtAck = 0;
    dhtWait = false;
    bitNumber = 0;
    pulseCounter = 0; 
    memset(dhtBuffer, 0, sizeof(dhtBuffer));  //azzeramento del buffer di lettura del sensore, per scrivere solo gli "uni" ricevuti
    startMaxDelayTimer2();
  }
}

//funzione che assegna l'indirizzo della funzione custom definita dall'utente in caso di misurazione corretta al puntatore a funzione
void dhtSetSuccessCallback(void (*callback)()){
  dhtSuccessCallback = callback;
}

//funzione che assegna l'indirizzo della funzione custom definita dall'utente in caso di misurazione errata o mancata al puntatore a funzione
void dhtSetFailureCallback(void (*callback)()){
  dhtFailureCallback = callback;
}

/*funzioni di callback di prova definite "dall'utente" */
void sensorRead_OK(){              //funzione di callback personalizzata eseguita in caso di lettura positiva dei dati dal sensore
  newData = true;
  millisInterval = 10000;          //se ricevo correttamente la risposta dal sensore imposto l'intervallo di misurazione a 10000 ms
  if(temp() >= 22.50)              //se la temperatura è superiore a 22.5 accendo il led builtin altrimenti lo spengo
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
}

void sensorRead_NO_OK(){          //funzione di callback personalizzata eseguita in caso di lettura negativa o mancata dei dati dal sensore
  millisInterval = 1500;           //riduco l'intervallo di lettura a 1.5 secondi e faccio lampeggiare il led builtin 
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   //se ad esempio scollego il sensore il led builtin lampeggia
}



void setup() {
    Serial.begin(230400);
    dhtSetSuccessCallback(sensorRead_OK);       //nella funzione di setup assegno l'indirizzo delle funzioni di cb ai puntatori
    dhtSetFailureCallback(sensorRead_NO_OK);    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DHT_INT_PIN, OUTPUT);
    digitalWrite(DHT_INT_PIN, HIGH);
    millisInterval = 2200;                      //primo intervallo di misurazione a 2.2 s
}

void loop() { 
    dhtMeasure(millisInterval);
    if(newData){               //la variabile booleana newData è settata a true nella funzione di callback di misurazione corretta e posta a false dopo che i dati sono stati utilizzati
      Serial.print("TEMP: ");
      Serial.print(temp());
      Serial.print(" HUMI: ");
      Serial.println(humi());
      newData = false;
    }
}
