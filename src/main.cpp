// KODUN BOZULMASI MUHTEMEL. ŞİFRELEME ÇALIIŞMALARI DEVAM EDİYOR.


// SON EDIT 16.03.2025
// MUSTAFA ALPER KAYA TARAFINDAN OLUŞTURULDU
// nrf24 modüllerini kullanan dijital trimli 8 kanallı RC kumanda verici kodu
// UÇTAN UCA ŞİFRELEME VE EEPROM OKUMA/YAZMA FONKSİYONLARI İÇİN BÜTÜN VERİLER SHORT TİPİNDEDİR.
// TRİM SERVONUN MENZİLİNİ AZALTACAĞI İÇİN SERVOLARIN TAKILMASINA VE TRİM AYARLARINDA DİKKATLİCE YAPIN

/*
####################################
YAPILACAKLAR/YAPIMI DEVAM EDENLER (önem sırasına göre)
%0          Şu an kullanılan trim fonksiyonun kendi yazacağım hali ile değiştirilmesi
%0          Başlangıçta yapılan gimbal ve switch kontrollerinin yapılandırılması
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

void trimKontrol(short, int, int, short, short);

short yonluTrimMap(String, );

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
 // reverse yapabilmek için ters veya düz seçenekli, hata durumunda 1500 sonucunu veren fonksiyon.
 // "trim değeri elleşilmeli mi?" kontrolü yapan fonksiyon.
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
 
  kanal[4] = swcFonksiyon(pinSWA, 2);
  kanal[5] = swcFonksiyon(pinSWB, 2);
  kanal[6] = swcFonksiyon(pinSWC, 3);
  kanal[7] = swcFonksiyon(pinSWD, 2);
  throttleHold("kapalı",pinSWA,kanal[4]); // switch kapalı durumda iken gazı kapatma özelliğini deaktive etmek için "açık"ı başka bir şey yapın. Bu özellik sadece "açık" iken çalışacak.


  radio.write(&kanal,sizeof(kanal));
  
}

// FONKSİYON TANIMLARI

short cokluEEPROMoku(int adres1, int adres2) {
  byte ciktiHigh = EEPROM.read(adres1);
  byte ciktiLow = EEPROM.read(adres2);
  short sonuc = (ciktiHigh << 8) + ciktiLow;
  return sonuc;
}

void cokluEEPROMyaz(short sayi, int highAdres, int lowAdres) {
  byte lowByte = lowByte(sayi);
  byte highByte = highByte(sayi);
  EEPROM.update(highAdres, highByte);
  EEPROM.update(lowAdres, lowByte);
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
