// KODUN BOZULMASI MUHTEMEL. ŞİFRELEME ÇALIIŞMALARI DEVAM EDİYOR.


// SON EDIT 16.03.2025
// MUSTAFA ALPER KAYA TARAFINDAN OLUŞTURULDU
// nrf24 modüllerini kullanan dijital trimli 8 kanallı RC kumanda verici kodu
// UÇTAN UCA ŞİFRELEME İÇİN BÜTÜN VERİLER SHORT TİPİNDEDİR.
// TRİM SERVONUN MENZİLİNİ AZALTACAĞI İÇİN SERVOLARIN TAKILMASINA VE TRİM AYARLARINDA DİKKATLİCE YAPIN

/*
####################################
YAPILACAKLAR/YAPIMI DEVAM EDENLER (önem sırasına göre)
%0          başlangıçta yapılan gimbal ve switch kontrollerinin yapılandırılması
%0          Servo sinyal karıştırma                                             
%70         ChaCha20-Poly1305 Şifreleme Koruması                                
####################################

#####################################################         KANAL SIRALAMASI         #####################################################
  Keyfinize göre açın, kapatın, değiştirin.
  1 = aileron             düz 
  2 = elevator            düz
  3 = gaz                 düz
  4 = rudder              düz
  5 = SWA(SPDT)           düz
  6 = SWB(SPDT)           düz
  7 = SWC(SPDT_MSM)       düz
  8 = SWD(SPDT)           düz
######################         DOĞRU ÇALIŞMASI İÇİN LÜTFEN HEM VERİCİDE HEM DE ALICIDA AYNI SIRALAMAYI YAPIN         #######################
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>


#define aileronPinTX A2
#define elevatorPinTX A3
#define gazPinTX A0
#define rudderPinTX A1

#define pinSWA 7
#define pinSWB 8
#define pinSWC A5
#define pinSWD 2

#define trimAileronArti A7  
#define trimAileronEksi A6
#define trimElevatorArti 4
#define trimElevatorEksi  3           
#define trimRudderArti 6
#define trimRudderEksi 7

#define buzzerPin A4

// kanal sayısı bu sayıya bağlı. (ŞİFRELİ MAKS. 8)
int  kanal[8];


short ailTrimAdresleri[2] = {100,101};// kanallarda 1 = aileron, 2 = elevator ve 4 = rudder olarak kullanıldığı için adresleri böyle seçtim. 
short eleTrimAdresleri[2] = {102,103};// çoklu eeprom okuma ve yazma fonksiyonları düzeltilince işe yarayacaklar.s
short rudTrimAdresleri[2] = {104,105};



unsigned long sonBasma = 0;
int aralik = 200; // tam olarak ne olmalı bilmiyorum, şimdilik 200 ama. 

const byte nrf24kod[5] = {'s','i','f','r','e'}; 
RF24 radio(9,10);

//// FONKSİYONLAR

// SERVOLARA GİDEN DEĞERLERİ BELİRLEYEN BÜTÜN FONKSİYONLARDA HATA DURUMUNDA 1500 (servo.writeMicroseconds() kullanıldığı takdirde tam orta konum) DEĞERİNİ VERMESİ SAĞLANMIŞTIR.

short cokluEEPROMoku(int, int); // EEPROM'dan short veri okuma

void cokluEEPROMyaz(short, int, int); // EEPROM'a short veri yazma

short swcFonksiyon(short, short); /*
Fonksiyonun doğru çalışması için sinyal pinlerinin modunu belirlemede "pinMode(pin,INPUT_PULLUP)" kullanılması gerekir.
Takılan toggle switchlerin kaç pozisyonlu olduğuna göre 0-1023 arasını uygun sayıda bölüme bölüp buna göre switchin kaçıncı pozisyonda olduğunu hesaplayıp uygun değeri yazan fonksiyon.

Her bir pozisyon için tolerans seviyesi +-15 olarak belirlendiğinden takılan switchin sahip olabileceği maksimum pozisyon sayısı 35. (yani örn. açısal kontrol özellikli bir servo için 35 farklı konum)
Maksimum pozisyon sayısı 35 olsa da kullanılacak switchlerin pin yapısından dolayı fonksiyon sadece 2 ve 3 pozisyonlu switchler için düzenlenmiştir.
Her pozisyonda farklı bir değer okunabilmesi için pozisyon pinlerine Arduino'nun dahili pull-up direncine uygun farklı büyüklükte dirençler takılarak voltaj bölücüleri oluşturulmalıdır.

Örn. bir switch 3 pozisyonlu (SPDT_MSM) olarak tanıtılıp, uygun pinlere direnç bağlandıktan sonra (pin 1 --> GND, pin 2 --> dahili pull-down dirençli sinyal pini, pin 3 --> 30KΩ-GND)
birinci pozisyonda 0, ikinci pozisyonda 1500 ve üçüncü pozisyonda 2000 değerini yazacaktır.
*/
void throttleHold(String, short, int&); // fonksiyon etkin iken seçilen switchin durumuna göre gazı tümden kapatan fonksiyon.
short yonluTrimMap(String, short, short); // reverse yapabilmek için ters veya düz seçenekli, hata durumunda 1500 sonucunu veren fonksiyon.
void trimAdjustCheck(short, short, short, int, int); // "trim değeri elleşilmeli mi?" kontrolü yapan fonksiyon.
void baslamaKontrolu(); /* 
İnternette satılan kumandalardaki gibi başlangıçta gaz kapalı değilse veya switchlerden en az biri 
kapalı durumda değilse sesli uyarı verip iletişimi başlatmama güvenlik özelliğini sağlayan fonksiyon. Eğer tetiklenirse bütün kod durduğu için verici tekrardan başlatılmalı.
*/

// ŞİFRELEME

// ŞİFRELEME

//// FONKSİYONLAR

short trimAileron = cokluEEPROMoku(ailTrimAdresleri[0],ailTrimAdresleri[1]);
short trimElevator = cokluEEPROMoku(eleTrimAdresleri[0],eleTrimAdresleri[1]); 
short trimRudder = cokluEEPROMoku(rudTrimAdresleri[0],rudTrimAdresleri[1]);

void setup() { 
  //baslamaKontrolu();

  pinMode(trimAileronArti, INPUT_PULLUP);
  pinMode(trimAileronEksi, INPUT_PULLUP);
  pinMode(trimElevatorArti, INPUT_PULLUP);
  pinMode(trimElevatorEksi, INPUT_PULLUP);
  pinMode(trimRudderArti, INPUT_PULLUP);
  pinMode(trimRudderEksi, INPUT_PULLUP);

  pinMode(pinSWA, INPUT_PULLUP);
  pinMode(pinSWB, INPUT_PULLUP);
  pinMode(pinSWC, INPUT_PULLUP);
  pinMode(pinSWD, INPUT_PULLUP);

  trimAileron = EEPROM.get(ailTrimAdresleri[0], trimAileron); 
  trimElevator = EEPROM.get(eleTrimAdresleri[0], trimElevator); 
  trimRudder = EEPROM.get(rudTrimAdresleri[0], trimRudder);
  
  radio.begin();
  radio.openWritingPipe(nrf24kod);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW); // güç çıkışı şu an düşük durumda, mesafeyi artırmak için ...(RF24_PA_HIGH/MAX) yapılmalı.
  radio.stopListening();  
}

void loop() {
  //trimAdjustCheck(trimAileronArti, trimAileronEksi, trimAileron, ailTrimAdresleri[0], ailTrimAdresleri[1]);
  //trimAdjustCheck(trimElevatorArti, trimElevatorEksi, trimElevator,eleTrimAdresleri[0], eleTrimAdresleri[1]);
  //trimAdjustCheck(trimRudderArti, trimRudderEksi, trimRudder, rudTrimAdresleri[0], rudTrimAdresleri[1]);
  kanal[0] = yonluTrimMap("düz", aileronPinTX, trimAileron); // reverse için "düz"ü "ters" yapın.
  kanal[1] = yonluTrimMap("düz", elevatorPinTX, trimElevator);
  kanal[2] = yonluTrimMap("düz", gazPinTX, 512);
  kanal[3] = yonluTrimMap("düz", rudderPinTX, trimRudder);
  kanal[4] = swcFonksiyon(pinSWA, 2);
  kanal[5] = swcFonksiyon(pinSWB, 2);
  kanal[6] = swcFonksiyon(pinSWC, 3);
  kanal[7] = swcFonksiyon(pinSWD, 2);
  throttleHold("kapalı",pinSWA,kanal[4]); // switch kapalı durumda iken gazı kapatma özelliğini deaktive etmek için "açık"ı başka bir şey yapın. Bu özellik sadece "açık" iken çalışacak.


  radio.write(&kanal,sizeof(kanal));
  
}

// FONKSİYON TANIMLARI



void cokluEEPROMyaz(short sayi, int highAdres, int lowAdres) {
  byte lowByte = lowByte(sayi);
  byte highByte = highByte(sayi);
  EEPROM.update(highAdres, highByte);
  EEPROM.update(lowAdres, lowByte);
}
                                                                      
short cokluEEPROMoku(int adres1, int adres2) {
  byte ciktiHigh = EEPROM.read(adres1);
  byte ciktiLow = EEPROM.read(adres2);
  short sonuc = (ciktiHigh << 8) + ciktiLow;
  return sonuc;
}

short swcFonksiyon(short switchPin, short pozisyon) {
  if ((pozisyon < 2 || pozisyon > 35) && pozisyon != 3) {
    return 1500;
  }
  else {
  short okuma = analogRead(switchPin);
  if (pozisyon == 3) {
    if (okuma < 150) {
      return 1000;
    }
    else if (okuma > 873) {
      return 1500;
    }
    else {
      return 2000;
    }
  }
  else {
  short bolge = round((float)okuma / (1023 / (pozisyon - 1)));
  return map(bolge, 0, pozisyon - 1, 1000, 2000);
    }
  }
}
void throttleHold(String durum, short switchPin, int &kanal) {
  if (durum == "açık" && digitalRead(switchPin) == LOW) {
    kanal = 0;
  }
}

short yonluTrimMap(String yon, short potPin, short trimDegeri) { // bu fonksiyonda KendinYap kanalının kodundan esinlenildi
  short deger = constrain(analogRead(potPin), 0, 1023);                      // https://www.rcpano.net/2022/10/30/uzun-mesafe-8-kanalli-ve-dijital-trimli-uzaktan-kumanda-yapimi-diy-rc-bolum-2/
  if (deger < trimDegeri) {
    deger = map(deger, 0, trimDegeri, 1000, 1500);
  }
  else {
    deger = map(deger, trimDegeri, 1023, 1500, 2000); 
  }
  if (yon == "düz") {
    return deger;
  }
  else if (yon == "ters") {
    return map(deger, 1000, 2000, 2000, 1000);
  }
  return 1500;
}

void trimAdjustCheck(short trimDugme1, short trimDugme2, short trimDegeri, int trimAdres1, int trimAdres2) {
  unsigned long gecenZaman = millis();
  static bool buzzerTetik = false;
  
  if (trimDegeri == 512 && !buzzerTetik) {
    tone(buzzerPin, 900, 500);
    buzzerTetik = true;
  }
  if (trimDegeri != 512) {
    buzzerTetik = false;
  }
  if (gecenZaman - sonBasma >= aralik) {
    if (digitalRead(trimDugme1) == LOW && trimDegeri < 896 ) {
      trimDegeri += 6;
      trimDegeri = constrain(trimDegeri, 128, 896);
      cokluEEPROMyaz(trimDegeri, trimAdres1, trimAdres2);
      tone(buzzerPin, 1000, 100);
    }
    if (digitalRead(trimDugme2) == LOW && trimDegeri > 128) {
      trimDegeri -= 6;
      trimDegeri = constrain(trimDegeri, 128, 896);
      cokluEEPROMyaz(trimDegeri, trimAdres1, trimAdres2);
      tone(buzzerPin, 800, 100);
    }
    sonBasma = gecenZaman;
  } 
}

void baslamaKontrolu() {
  if (digitalRead(gazPinTX) != LOW || digitalRead(pinSWA) != LOW || digitalRead(pinSWB) != LOW || digitalRead(pinSWC) != LOW || digitalRead(pinSWD) != LOW) {
    while (1) {
      tone(buzzerPin,1200,110);
      delay(430);
      tone(buzzerPin,1300,125);
      delay(200);
      tone(buzzerPin,1400,250);
      delay(750);
    } 
  }
}
