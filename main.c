#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <math.h>

#define pozz 0.1
//Результат выполнения операции с i2c
typedef enum {
    I2C_SUCCESS = 0,
    I2C_TIMEOUT,
    I2C_ERROR
} t_i2c_status;

void I2C_setup(void);
void TIM4_setup(void);
void TIM1_setup(void);

unsigned int get_UP(void);
void getBoshEeprom(void);
void start_UX(unsigned char b1, unsigned char b2);
int calculateBoschTemperature(void);
long calculateBoschPressure(void);
unsigned long filter(float val);
void clearMemory(void);
void TIM2_setup(void);

#define LED PD_ODR_bit.ODR3
#define BUTTON PD_IDR_bit.IDR2

// This union is used to manipulate a two byte unsigned int
typedef union _WORD_TYPE
{
  unsigned int val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
  };
  unsigned char v[2];
} WORD_TYPE;

// This union is used to manipulate a two byte signed int
typedef union _SIGNED_WORD_TYPE
{
  int val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
  };
  unsigned char v[2];
} SIGNED_WORD_TYPE;

// This union is used to manipulate a four byte signed long
typedef union _LONG_TYPE
{
  long val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
    unsigned char b0;
    unsigned char b1;
  };
  unsigned char v[4];
} LONG_TYPE;





/*
 * Register and the codes written to the register to start temperature and
 * pressure conversions.
 */
#define BMP085_START_REG 0xf4
#define BMP085_START_UT 0x2e // temperature
#define BMP085_START_UP 0xF4 // pressure
#define BMP085_CONVERSION_TIME 4  // conversion time in ms

/*
 * Registers containing the 16-bit temperature and pressure data after a 
 * conversion.  The same register is used, what is read is determined by what
 * was previously written into the Start register.
 */
#define BMP085_UT 0xf6
#define BMP085_UP 0xf6

/*
 * Local registers for storing the calibration data read from the EEPROM.
 * These are setup as unions defined above so that they can be accessed one byte
 * at a time when read from the device EEPROM.
 */
SIGNED_WORD_TYPE Bosch_ac1,
                 Bosch_ac2,
                 Bosch_ac3;
WORD_TYPE Bosch_ac4,
          Bosch_ac5,
          Bosch_ac6;
SIGNED_WORD_TYPE Bosch_b1,
                 Bosch_b2,
                 Bosch_mb,
                 Bosch_mc,
                 Bosch_md;

/*
 * Raw temperature read from module
 */ 
LONG_TYPE Bosch_ut;

/*
 * Raw pressure read from module
 */ 
LONG_TYPE Bosch_up;

/*
 * Intermediate registers for calculations
 */ 
long Bosch_x1,
     Bosch_x2,
     Bosch_x3,
     Bosch_b3,
     Bosch_b5,
     Bosch_b6,
     Bosch_p,
     Bosch_t;

unsigned long Bosch_b4,
              Bosch_b7;

t_i2c_status i2c_wr_reg(unsigned char address, unsigned char reg_addr, unsigned char * data, unsigned char length);
t_i2c_status i2c_rd_reg(unsigned char address, unsigned char reg_addr, unsigned char * data, unsigned char length);

void start_UP(void);
unsigned char in[22];
unsigned int d;
unsigned int q;

float varVario = 10.63;
float varProcess = 0.05;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

const float koef = 18467.436;
unsigned long fil_Pa;
unsigned long ex_fil_Pa;
float fil_Pa_mass[8];

char capture = 0;
char tim_fl=0;

unsigned char goUp;

static unsigned char chastota;
//static unsigned char pick;
//static unsigned char izmerenia;
static unsigned char calibr=0;
static unsigned char t_calibr=0;
static unsigned char ex_chastota=0;

float u;
float fil_delta;
#pragma location=0x4000 //Наш адрес в EEPROM/flash (в данном случае - начало еепромки)
__no_init float adr_eeprom;

//----------------------------------------------------------
//Переполнение таймера TIMR4
#pragma vector=TIM4_OVR_UIF_vector
__interrupt void TIMR4_ISR(void){
  if(chastota == 0) PD_ODR_bit.ODR4 = 0;
  else{
    d++;
    if(d>chastota){
      d=0;
      if(ex_chastota == 1)PD_ODR_bit.ODR4 = !PD_ODR_bit.ODR4;
      else PD_ODR_bit.ODR4 = 0;
    }
  }  
  //
  TIM4_SR_UIF=0;
}
//-----------------------------------------------------------
// Переполнение буффера TIMR1
#pragma vector=TIM1_OVR_UIF_vector
__interrupt void TIMR1_ISR(void)
{
//Интервал 0,103с
//------------------- Ждем стадилизацию фильтра
if(calibr == 0){
  t_calibr++;
  if(t_calibr > 75)   //Надо 7 
    calibr = 1;
  LED = 1;
  } else{   //Как калиброака прошла
    tim_fl=1;  //делаем замеры
  } 
TIM1_SR1_UIF=0;
}
//------------------------------------------------------------------------------
// Переполнение буффера TIMR2
#pragma vector=TIM2_OVR_UIF_vector
__interrupt void TIMR2_ISR(void)
{
//Интервал 0,103с
//------------------- Ждем стадилизацию фильтра
if(goUp >0){
  ex_chastota = !ex_chastota;
  }
else ex_chastota = 1;
TIM2_SR1_UIF=0;
}
//------------------------------------------------------------------------------
//настройка таймера ТИМ4
void TIM4_setup(void){

  TIM4_PSCR=0;                          //задаём предделитель таймера
  TIM4_IER_UIE=1;                       //разрешаем прерывания от таймера
  TIM4_CR1_CEN=1;                       //разрешаем таймер
}

//------------------------------------------------------------------------------
//Настройка таймера TIMER1
void TIM1_setup(void){
TIM1_PSCRH = 0;         //Пределитель 
TIM1_PSCRL = 0x18;      //Переполнение 0,1с
TIM1_CR1_UDIS = 0;      //Разрешение прерывания
TIM1_CR1_URS = 1;       //Прерывание по переполению
TIM1_CR1_DIR = 1;       //Считаем вверх
TIM1_CR1_CMS = 0;
TIM1_IER =0x01;         //разрешаем прерывания от таймера
TIM1_CR1_CEN=1;         //разрешаем таймер
}
//------------------------------------------------------------------------------
//Настройка таймера TIMER2
void TIM2_setup(void){
TIM2_PSCR = 0x05;         //Пределитель 
TIM2_CR1_UDIS = 0;      //Разрешение прерывания
TIM2_CR1_URS = 1;       //Прерывание по переполению
TIM2_IER =0x01;         //разрешаем прерывания от таймера
TIM2_CR1_CEN=1;         //разрешаем таймер
}

//------------------------------------------------------------------------------
void I2C_setup(void){
  unsigned long int ccr;
   
  PB_DDR_bit.DDR4 = 0;
  PB_DDR_bit.DDR5 = 0;
  PB_ODR_bit.ODR5 = 1;  //SDA
  PB_ODR_bit.ODR4 = 1;  //SCL
   
  PB_CR1_bit.C14 = 0;
  PB_CR1_bit.C15 = 0;
   
  PB_CR2_bit.C24 = 0;
  PB_CR2_bit.C25 = 0;
   
  //Частота тактирования периферии MHz
  I2C_FREQR_FREQ = 16;
  //Отключаем I2C
  I2C_CR1_PE = 0;
  //В стандартном режиме скорость I2C max = 100 кбит/с
  //Выбираем стандартный режим 
  I2C_CCRH_F_S = 0;
  //CCR = Fmaster/2*Fiic= 12MHz/2*100kHz
  ccr = 0x50;
  //Set Maximum Rise Time: 1000ns max in Standard Mode
  //= [1000ns/(1/InputClockFrequencyMHz.10e6)]+1
  //= InputClockFrequencyMHz+1
  I2C_TRISER_TRISE = 17;
  I2C_CCRL = ccr & 0xFF;
  I2C_CCRH_CCR = (ccr >> 8) & 0x0F;
  //Включаем I2C
  I2C_CR1_PE = 1;
  //Разрешаем подтверждение в конце посылки
  I2C_CR2_ACK = 1;
}


//******************************************************************************
// Чтение регистра slave-устройства
// Start -> Slave Addr -> Reg. addr -> Restart -> Slave Addr <- data ... -> Stop 
//******************************************************************************                                   
t_i2c_status i2c_rd_reg(unsigned char address, unsigned char reg_addr,
                              unsigned char * data, unsigned char length)
{
   
  //Ждем освобождения шины I2C
  while(I2C_SR3_BUSY);
     
  //Разрешаем подтверждение в конце посылки
  I2C_CR2_ACK = 1;
   
  //Генерация СТАРТ-посылки
  I2C_CR2_START = 1;
  //Ждем установки бита SB
  while(!I2C_SR1_SB);
   
  //Записываем в регистр данных адрес ведомого устройства
  I2C_DR = address & 0xFE;
  //Ждем подтверждения передачи адреса
  while(!I2C_SR1_ADDR);
  //Очистка бита ADDR чтением регистра SR3
  I2C_SR3;
   
  //Ждем освобождения регистра данных RD
  while(!I2C_SR1_TXE);
   
  //Передаем адрес регистра slave-устройства, который хотим прочитать
  I2C_DR = reg_addr;
  //Ловим момент, когда DR освободился и данные попали в сдвиговый регистр
  while(!(I2C_SR1_TXE && I2C_SR1_BTF));
   
  //Генерация СТАРТ-посылки (рестарт)
  I2C_CR2_START = 1;
  //Ждем установки бита SB
  while(!I2C_SR1_SB);
   
  //Записываем в регистр данных адрес ведомого устройства и переходим
  //в режим чтения (установкой младшего бита в 1)
  I2C_DR = address | 0x01;
   
  //Дальше алгоритм зависит от количества принимаемых байт
  //N=1
  if(length == 1){
    //Запрещаем подтверждение в конце посылки
    I2C_CR2_ACK = 0;
    //Ждем подтверждения передачи адреса
    while(!I2C_SR1_ADDR);
     
    //Заплатка из Errata
    __disable_interrupt();
    //Очистка бита ADDR чтением регистра SR3
    I2C_SR3;
     
    //Устанавлием бит STOP
    I2C_CR2_STOP = 1;
    //Заплатка из Errata
    __enable_interrupt();
     
    //Ждем прихода данных в RD
    while(!I2C_SR1_RXNE);
     
    //Читаем принятый байт
    *data = I2C_DR;
  } 
  //N=2
  else if(length == 2){
    //Бит который разрешает NACK на следующем принятом байте
    I2C_CR2_POS = 1;
    //Ждем подтверждения передачи адреса
    while(!I2C_SR1_ADDR);
    //Заплатка из Errata
    __disable_interrupt();
    //Очистка бита ADDR чтением регистра SR3
    I2C_SR3;
    //Запрещаем подтверждение в конце посылки
    I2C_CR2_ACK = 0;
    //Заплатка из Errata
    __enable_interrupt();
    //Ждем момента, когда первый байт окажется в DR,
    //а второй в сдвиговом регистре
    while(!I2C_SR1_BTF);
     
    //Заплатка из Errata
    __disable_interrupt();
    //Устанавлием бит STOP
    I2C_CR2_STOP = 1;
    //Читаем принятые байты
    *data++ = I2C_DR;
    //Заплатка из Errata
    __enable_interrupt();
    *data = I2C_DR;
  } 
  //N>2
  else if(length > 2){
    //Ждем подтверждения передачи адреса
    while(!I2C_SR1_ADDR);
     
    //Заплатка из Errata
    __disable_interrupt();
     
    //Очистка бита ADDR чтением регистра SR3
    I2C_SR3;
     
    //Заплатка из Errata
    __enable_interrupt();
     
    while(length-- > 3){
      //Ожидаем появления данных в DR и сдвиговом регистре
      while(!I2C_SR1_BTF);
      //Читаем принятый байт из DR
      *data++ = I2C_DR;
    }
     
    //Осталось принять 3 последних байта
    //Ждем, когда в DR окажется N-2 байт, а в сдвиговом регистре
    //окажется N-1 байт
    while(!I2C_SR1_BTF);
    //Запрещаем подтверждение в конце посылки
    I2C_CR2_ACK = 0;
    //Заплатка из Errata
    __disable_interrupt();
    //Читаем N-2 байт из RD, тем самым позволяя принять в сдвиговый
    //регистр байт N, но теперь в конце приема отправится посылка NACK
    *data++ = I2C_DR;
    //Посылка STOP
    I2C_CR2_STOP = 1;
    //Читаем N-1 байт
    *data++ = I2C_DR;
    //Заплатка из Errata
    __enable_interrupt();
    //Ждем, когда N-й байт попадет в DR из сдвигового регистра
    while(!I2C_SR1_RXNE);
    //Читаем N байт
    *data++ = I2C_DR;
  }
   
  //Ждем отправки СТОП посылки
  while(I2C_CR2_STOP);
  //Сбрасывает бит POS, если вдруг он был установлен
  I2C_CR2_POS = 0;
   
  return I2C_SUCCESS;
}
//----------------------------------------------------------------------------
void start_UX(unsigned char b1, unsigned char b2){
  //Ждем освобождения шины I2C
  while(I2C_SR3_BUSY);
     
  //Разрешаем подтверждение в конце посылки
  I2C_CR2_ACK = 1;
   
  //Генерация СТАРТ-посылки
  I2C_CR2_START = 1;
  //Ждем установки бита SB
  while(!I2C_SR1_SB);
   
  //Записываем в регистр данных адрес ведомого устройства
  I2C_DR = 0xEE & 0xFE;
  //Ждем подтверждения передачи адреса
  while(!I2C_SR1_ADDR);
  //Очистка бита ADDR чтением регистра SR3
  I2C_SR3;
   
  //Ждем освобождения регистра данных RD
  while(!I2C_SR1_TXE);
  I2C_SR3;
    //Записываем в регистр данных адрес ведомого устройства
  I2C_DR = b1;
    //Ждем подтверждения передачи адреса
  while(!I2C_SR1_TXE);
    //Очистка бита ADDR чтением регистра SR3
  I2C_SR3;
      //Записываем в регистр данных адрес ведомого устройства
  I2C_DR = b2; 
  //Ждем освобождения регистра данных RD
  while(!I2C_SR1_TXE);
  
  //Заплатка из Errata
    __disable_interrupt();
    //Очистка бита ADDR чтением регистра SR3
    I2C_SR3;
     
    //Устанавлием бит STOP
    I2C_CR2_STOP = 1;
    //Заплатка из Errata
    __enable_interrupt();
    
  //Ждем отправки СТОП посылки
  while(I2C_CR2_STOP);
  //Сбрасывает бит POS, если вдруг он был установлен
  I2C_CR2_POS = 0;
  
}


//-----------------------------------------------------------------------------
void getBoshEeprom(void){
  i2c_rd_reg(0xEE, 0xAA, in, 22);
  Bosch_ac1.MSB = in[1];
  Bosch_ac1.LSB = in[0];
  Bosch_ac2.MSB = in[3];
  Bosch_ac2.LSB = in[2];
  Bosch_ac3.MSB = in[5];
  Bosch_ac3.LSB = in[4];
  Bosch_ac4.MSB = in[7];
  Bosch_ac4.LSB = in[6];
  Bosch_ac5.MSB = in[9];
  Bosch_ac5.LSB = in[8];
  Bosch_ac6.MSB = in[11];
  Bosch_ac6.LSB = in[10];
  Bosch_b1.MSB = in[13];
  Bosch_b1.LSB = in[14];
  Bosch_b2.MSB = in[15];
  Bosch_b2.LSB = in[14];
  Bosch_mb.MSB = in[17];
  Bosch_mb.LSB = in[16];
  Bosch_mc.MSB = in[19];
  Bosch_mc.LSB = in[18];
  Bosch_md.MSB = in[21];
  Bosch_md.LSB = in[20];
}
//----------------------------------------------------------------------------
void getBoshUT(void){
  i2c_rd_reg(0xEE, BMP085_UT, in, 2);
  Bosch_ut.MSB = in[1];
  Bosch_ut.LSB = in[0];
  Bosch_ut.b0 = 0;
  Bosch_ut.b1 = 0;
}
//----------------------------------------------------------------------------
void getBoshUP(void){
  i2c_rd_reg(0xEE, BMP085_UP, in, 2);
  Bosch_up.MSB = in[1];
  Bosch_up.LSB = in[0];
  Bosch_up.b0 = 0;
  Bosch_up.b1 = 0;
}
//----------------------------------------------------------------------------
int calculateBoschTemperature(void){
  Bosch_ut.val = (unsigned long)Bosch_ut.val>>16;
  Bosch_x1 = (Bosch_ut.val - Bosch_ac6.val) * Bosch_ac5.val / 32768L;
  Bosch_x2 = Bosch_mc.val * 2048L / (Bosch_x1 + Bosch_md.val);
  Bosch_b5 = Bosch_x1 + Bosch_x2;
  Bosch_t = (Bosch_b5 + 8) / 16L;
  
  return((int)Bosch_t);

}
//----------------------------------------------------------------------------
long calculateBoschPressure(void){
  Bosch_up.val = (unsigned long)Bosch_up.val >> 16;
  Bosch_b6 = Bosch_b5 - 4000L;
  Bosch_x1 = (Bosch_b2.val * (Bosch_b6 * Bosch_b6 / 4096L)) / 2048L;
  Bosch_x2 = Bosch_ac2.val * Bosch_b6 / 2048L;
  Bosch_x3 = Bosch_x1 + Bosch_x2;
  Bosch_b3 = ((Bosch_ac1.val * 4L + Bosch_x3) + 2L) / 4L;
  Bosch_x1 = Bosch_ac3.val * Bosch_b6 / 8192L;
  Bosch_x2 = (Bosch_b1.val * (Bosch_b6 * Bosch_b6 / 4096L)) / 65536;
  Bosch_x3 = ((Bosch_x1 + Bosch_x2) + 2L) / 4L;
  Bosch_b4 = Bosch_ac4.val * (unsigned long)(Bosch_x3 + 32768L) / 32768UL;
  Bosch_b7 = ((unsigned long)Bosch_up.val - Bosch_b3) * 50000UL;
  if(Bosch_b7 < 0x80000000){
    Bosch_p = (Bosch_b7 * 2L) / Bosch_b4;
  }
  else{
    Bosch_p = (Bosch_b7 / Bosch_b4) * 2L;
  }
  Bosch_x1 = (Bosch_p / 256L) * (Bosch_p / 256L);
  Bosch_x1 = (Bosch_x1 * 3038L) / 65536L;
  Bosch_x2 = (-7357L * Bosch_p) / 65536L;
  Bosch_p = Bosch_p + (Bosch_x1 + Bosch_x2 + 3791L) / 16;
  
  return(Bosch_p);

}
//----------------------------------------------------------------------------
unsigned long filter(float val) {  //функция фильтрации
  Pc = P + varProcess;
  G = Pc/(Pc + varVario);
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
  return((unsigned long)Xe);
}
//----------------------------------------------------------------------------
float delta(float poz1, float poz2){
float rez;
rez = log10(poz1/poz2)*18467.436;
return rez;
}
//----------------------------------------------------------------------------
void setPipi(unsigned char pi){
  if(pi>7) goUp = 8;
  TIM2_ARRH = 0x00 - (pi*0x10);
  TIM2_ARRL = 0xFF; 
}
//----------------------------------------------------------------------------
void main( void )
{
  unsigned int d;
  unsigned long p;
  
  goUp = 0;
  ex_chastota = 1;
  
  CLK_CKDIVR_bit.HSIDIV = 0;
  for(d=0; d<60000; d++);  

  //--------- Настройка портов не перефиреальных

  //Светодиод
  PD_DDR_bit.DDR3 = 1;  //Выход
  PD_ODR_bit.ODR3 = 0;  //На выход 0
  PD_CR1_bit.C13 = 1;   //Push-pull output
  PD_CR2_bit.C23 = 0;   //Медленный режим
  
    //Пищалка
  PD_DDR_bit.DDR4 = 1;  //Выход
  PD_ODR_bit.ODR4 = 0;  //На выход 0
  PD_CR1_bit.C14 = 1;   //Push-pull output
  PD_CR2_bit.C24 = 0;   //Медленный режим
  

  //---------- Настройка перефирии
  I2C_setup();
  TIM4_setup();
  TIM1_setup();
  TIM2_setup();
  
  __enable_interrupt();
  
  LED = 1;

  getBoshEeprom();
  
  FLASH_DUKR = 0xAE;
  FLASH_DUKR = 0x56;
  
  adr_eeprom = -0.055;
  
a:

  
  //---------------------------------------------------------------------------------------------
  while(calibr==0){
      start_UX(BMP085_START_REG, BMP085_START_UT);
      for(d=0; d<15000; d++);
      getBoshUT();
      
      start_UX(BMP085_START_REG, BMP085_START_UP);  
      for(d=0; d<65000; d++);
      for(d=0; d<30000; d++);
      getBoshUP();
      p = calculateBoschPressure();
      ex_fil_Pa = fil_Pa;
      fil_Pa = filter(p);
      }
  

    if(tim_fl==1){
        LED=!LED;
        start_UX(BMP085_START_REG, BMP085_START_UT);
        for(d=0; d<15000; d++);
        getBoshUT();
        
        start_UX(BMP085_START_REG, BMP085_START_UP);  
        for(d=0; d<65000; d++);
        for(d=0; d<30000; d++);
        getBoshUP();
        p = calculateBoschPressure();
        ex_fil_Pa = fil_Pa;
        fil_Pa = filter(p);
        
        u = delta(ex_fil_Pa, fil_Pa);
        
        for(capture=0;capture<5; capture++)  fil_Pa_mass[capture] = fil_Pa_mass[capture+1];
        fil_Pa_mass[5] = u;
        
        fil_delta=0;
        for(capture=0;capture<6; capture++) fil_delta = fil_delta + fil_Pa_mass[capture]; 
        fil_delta = fil_delta/6;
         
        if((-0.03f<fil_delta)&&(fil_delta < 0.03f)) {chastota = 0; goUp = 0;}
        else 
          {
            if(-0.03f>fil_delta) {//вниз
              goUp = 0;
              if(fil_delta > -0.5f) 
                chastota = 175-(fil_delta*200);  
              else chastota = 225;
            }
            if(0.03f<fil_delta){//вввех

             goUp = fil_delta/0.055f;
             setPipi(goUp);
              
             if(fil_delta < 0.5) 
              
             chastota = 90-(fil_delta*200);  
             else  {chastota = 40; setPipi(8);}
          }
       }

    tim_fl=0;

    }

  goto a;

}
