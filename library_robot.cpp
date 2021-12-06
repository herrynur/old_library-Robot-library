#include "library_robot.h"

void setup()
{
    //Setup pin out dan lain lain pada void setup arduino
    Serial.begin(115200);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(pwm3, OUTPUT);
    pinMode(pwm4, OUTPUT);
    pinMode(buzzer, OUTPUT);

    pinMode(sPin[0], INPUT);
    pinMode(sPin[1], INPUT);
    pinMode(sPin[2], INPUT);
    pinMode(sPin[3], INPUT);
    pinMode(sPin[4], INPUT);
    pinMode(sPin[5], INPUT);
    //Void setup saat di Arduino IDE
    initlibraryrobot();
}

// Program membaca sensor dan menampilkan nilai di Serial monitor
void readsensor()
{
    uint8_t readPin[6];
    //kiri
    readPin[0] = digitalRead(sPin[0]);
    readPin[1] = digitalRead(sPin[1]);
    readPin[2] = digitalRead(sPin[2]);
    //kanan
    readPin[3] = digitalRead(sPin[3]);
    readPin[4] = digitalRead(sPin[4]);
    readPin[5] = digitalRead(sPin[5]);

    //   Serial.print(readPin[0]);
    //   Serial.print(readPin[1]);
    //   Serial.print(readPin[2]);
    //   Serial.print(readPin[3]);
    //   Serial.print(readPin[4]);
    //   Serial.print(readPin[5]);
    //   Serial.println();
}

// Program membaca garis dengan salah satu sensor yang di pilih dengan input nomor pin
int satusensor(int pin)
{
    uint8_t hasilbaca;
    hasilbaca = digitalRead(sPin[pin]);
    return hasilbaca;
}

//buzzer on
void buzzeron(int8_t loop_)
{
   for(int i=1;i<=loop_;i++)
   {
    digitalWrite(buzzer,HIGH);
    majutimer(0,100);
    digitalWrite(buzzer,LOW);
    majutimer(0,100);
   }
}

// Program untuk mendeteksi perempatan dengan membaca sensor paling kanan dan kiri
int detectcross()
{
    uint8_t hasilbaca;
    if (digitalRead(sPin[0]) == 1 && digitalRead(sPin[5]) == 1)
    {
        hasilbaca = 1;
    }
    else
    {
        hasilbaca = 0;
    }
    return hasilbaca;
}

// Program menjalankan motor bagian kanan dengan input speed
void motorjalankanan(int8_t speed)
{
    if (speed >= 0)
    {
        analogWrite(pwm1, speed);
        analogWrite(pwm2, 0);
    }
    if (speed < 0)
    {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, speed * -1);
    }
}

// Program menjalankan motor bagian kiri dengan input speed
void motorjalankiri(int8_t speed)
{
    if (speed >= 0)
    {
        analogWrite(pwm3, speed);
        analogWrite(pwm4, 0);
    }
    if (speed < 0)
    {
        analogWrite(pwm3, 0);
        analogWrite(pwm4, speed * -1);
    }
}

// Program menjalankan motor bagian kanan dengan input speed
void testmotorkanan(int8_t speed)
{
    motorjalankanan(speed);
}

// Program menjalankan motor bagian kiri dengan input speed
void testmotorkiri(int8_t speed)
{
    motorjalankiri(speed);
}

// Program menjalankan kedua motor dengan input speed kiri dan kanan
void testkeduamotor(int8_t speed_kiri, int8_t speed_kanan)
{
    motorjalankanan(speed_kanan);
    motorjalankiri(speed_kiri);
}

// Program belok kiri dengan input speed
void belokiri(int8_t speed)
{
    motorjalankanan(speed);
    motorjalankiri(-speed);
}

// Program belok kanan dengan input speed
void belokanan(int8_t speed)
{
    motorjalankanan(-speed);
    motorjalankiri(speed);
}

// Program belok kanan dengan speed kiri = 2x speed kanan
void belokirimaju(int8_t speed)
{
    motorjalankanan(speed);
    motorjalankiri(speed - (speed / 2));
}

// Program belok kiri dengan speed kanan = 2x speed kiri
void belokananmaju(int8_t speed)
{
    motorjalankanan(speed - (speed / 2));
    motorjalankiri(speed);
}

// Program motor maju dengan input speed dan timeout
void majutimer(int8_t speed, int delay_)
{
    bool isTimeout = false;
    unsigned long timeStart = millis();

    while (!isTimeout)
    {
        testkeduamotor(speed, speed);
        if ((unsigned long)millis() - timeStart > delay_)
        {
            isTimeout = true;
            testkeduamotor(0, 0);
        }
    }
}

// Program belok kanan mencari garis dengan input speed dan pin sensor yang akan mendeteksi
void linefindkanan(int8_t speed, int sensor)
{
    bool isFound = false;
    while (!isFound)
    {
        int pin = satusensor(sensor);
        belokanan(speed);
        if (pin == 1)
        {
            isFound = true;
        }
    }
}

// Program belok kiri mencari garis dengan input speed dan pin sensor yang akan mendeteksi
void linefindkiri(int8_t speed, int sensor)
{
    bool isFound = false;
    while (!isFound)
    {
        int pin = satusensor(sensor);
        belokiri(speed);
        if (pin == 1)
        { 
            isFound = true;
        }
    }
}

// Program mencari error dari nilai pembacaan sensor
int sampling()
{
    uint8_t readPin[6];
    int ska = 0, ski = 0, error = 0;
    //kiri
    readPin[0] = digitalRead(sPin[0]);
    readPin[1] = digitalRead(sPin[1]);
    readPin[2] = digitalRead(sPin[2]);
    //kanan
    readPin[3] = digitalRead(sPin[3]);
    readPin[4] = digitalRead(sPin[4]);
    readPin[5] = digitalRead(sPin[5]);
    //data sensor
    Serial.print(readPin[0]);
    Serial.print(readPin[1]);
    Serial.print(readPin[2]);
    Serial.print(readPin[3]);
    Serial.print(readPin[4]);
    Serial.println(readPin[5]);
    //error
    ska = readPin[5] * 2 + readPin[4] * 4 + readPin[3] * 8;
    ski = readPin[0] * 2 + readPin[1] * 4 + readPin[2] * 8;

    error = ska - ski;
    //Serial.println(error);
    return error;
}

// Program line follower dengan delay timeout
void lf_delay(int8_t speed, int delay_)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.1, KD = 0.1;
    bool isFind = false;
    int find;
    unsigned long timeStart = millis();

    while (!isFind)
    {
        find = detectcross();
        if (find == 1 || (unsigned long)millis() - timeStart > delay_)
        {
            isFind = true;
        }
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;

        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
}

// program line follower mencari persimpangan
void lf_crossfind(int8_t speed)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.1, KD = 0.1;
    bool isFind = false;
    int find;

    while (!isFind)
    {
        find = detectcross();
        if (find == 1)
        {
            isFind = true;
        }
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;

        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
}

// Program line follower
void linefollower(int8_t speed)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.1, KD = 0.1;
    while (true)
    {
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;

        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
}
//Akhir dari library