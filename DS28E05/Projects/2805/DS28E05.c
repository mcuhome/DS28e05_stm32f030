#include "stm32f0xx_gpio.h"
#include "stm32f0xx.h"
#include "DS28E05.h"
#include "stdio.h"

/**************����ָ���붨��******************************/
/*DS28E05 ROM��������*/
#define Rom_Read_Cmd                0x33    //Read ROM
#define Rom_Match_Cmd               0x55    //Match ROM
#define Rom_Skip_Cmd                 0xCC    //Skip ROM
#define Rom_Search_Cmd              0xF0    //Search ROM

/*DS28E05 �洢����������*/
#define Memory_Read_Cmd             0xF0    //Read Memory
#define Memory_Write_Cmd             0x55    //Write Memory
#define Scratchpad_Read_Cmd         0xAA    //Read Scratchpad
//#define Scratchpad_Write_Cmd        0x0F    //Write Scratchpad
#define Scratchpad_Write_Cmd        0x55    //Write Scratchpad
#define Scratchpad_Copy_Cmd         0x55    //Copy Scratchpad

#define DS28E05_READ_MEMORY   0xf0
#define DS28E05_WRITE_MEMORY  0x55
#define PAGE_MASK            0x70	// 0b01110000
#define SEG_MASK             0x0e 	//0b00001110
#define SEG_SIZE              8
#define BYTES_PER_SEG         2
#define BYTES_PER_PAGE        (SEG_SIZE * BYTES_PER_SEG)
#define NUM_PAGES             7

static int8_t  fac_us=0;//us
static int16_t fac_ms=0;//ms

void delay_init(void)	 
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;	//Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(int16_t)fac_us*1000;//ÿ��ms��Ҫ��systickʱ����   
}	
//��ʱNus
void delay_us(int32_t nus)
{		
	int32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱNms

void delay_ms(int16_t nms)
{	 		  	  
	int32_t temp;		   
	SysTick->LOAD=(int32_t)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
} 



/******************************************
****************�漰DS28E05����*************
*******************************************/
/******************************************
�������ƣ�GPIO_DS28E05_Out_Mode
��    �ܣ�����DS28E05����Ϊ��©���ģʽ
��    ������
����ֵ  ����
*******************************************/
void GPIO_DS28E05_Out_Mode( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        //��©���
	GPIO_Init( DS28E05_GPIO, &GPIO_InitStructure );
}
void GPIO_DS28E05_Out_Mode_init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        //��©���
	GPIO_Init( DS28E05_GPIO, &GPIO_InitStructure );
}
/******************************************
�������ƣ�GPIO_DS28E05_Input_Mode
��    �ܣ�����DS28E05����Ϊ��������ģʽ
��    ������
����ֵ  ����
*******************************************/
static void GPIO_DS28E05_Input_Mode( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   //��������
	//	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //��©���
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init_in( DS28E05_GPIO, &GPIO_InitStructure );
}
/******************************************
1-Wire������λ, �����Ӧ��;��Ӧ�𷵻�0, ��Ӧ�𷵻�1
*******************************************/
 unsigned char Reset( void )
{
	unsigned char testCount = 50;
    GPIO_DS28E05_Out_Mode();
    DS28E05_Write_1();
    DS28E05_Write_0();
    delay_us(60);                //��λ60us�����屣��
    DS28E05_Write_1();                //�ͷ����ߺ�9us��Ӧ��
	GPIO_DS28E05_Input_Mode();
    while(testCount--)
    {
        if( 0 == DS28E05_ReadBit() )
        {
            delay_us(2); 
            return 0;
        }
    }	
	return 1;
}
/******************************************
               д����λ1
*******************************************/
void WriteBit_1( void )
{
    DS28E05_Write_1();
    DS28E05_Write_0();        //�������߱���1us
    delay_us(1);
    DS28E05_Write_1();        //�ͷ�������ʱ����12us
    delay_us(20);//20us
}
/******************************************
                д����λ0
*******************************************/
void WriteBit_0( void )
{
 
    DS28E05_Write_1();
    DS28E05_Write_0();        //�������߱���8-16us
    delay_us(12);	//9-10us
    DS28E05_Write_1();        //�ͷ�������ʱ6us
    delay_us(9);
}
/******************************************
                ������λ
*******************************************/
unsigned char value1;
static unsigned char Read_Bit( void )
{
		GPIO_DS28E05_Out_Mode();
		DS28E05_Write_1();
		DS28E05_Write_0();                //�������߱���1us
		delay_us(1);
		DS28E05_Write_1();                
		DS28e05_IO_IN();				//����Ҫ��һ����һ��Ҫ�üĴ������������������ȡ��ʱ�䣬����⣬������ȡ����Ϊ2--3΢��
		value1 = DS28E05_ReadBit();   //
		delay_us(2);
    return value1;   
}
/******************************************
                д�ֽ�
*******************************************/
static void Write_Byte( unsigned char value )
{
    unsigned char i;
	GPIO_DS28E05_Out_Mode();
    for( i = 0; i < 8; i++ )
    {
        if( value & 0x01 )
        {
            WriteBit_1();
        }
        else
        {
            WriteBit_0();
        }
        value = value >> 1;
    }
}
/******************************************
                ���ֽ�
*******************************************/
static unsigned char Read_Byte( void )
{
    int i, value;

    value = 0;
    for( i = 0; i < 8; i++ )
    {
        value >>= 1;
        if( Read_Bit() ) 
            value |= 0x80;   
    }
    return value;
}

/******************************************
���ܣ���8λ������;48λ���к�;8λCRC��;��ȡ�ɹ�����0
������*id--��ȡ�����ݴ�ŵ�ַ
���أ�0--�����ɹ���1--���߲�����;
*******************************************/
unsigned char DS28E05_ReadRom( unsigned char *id )
{
    unsigned char i,j;
		while(Reset());
		Write_Byte( Rom_Read_Cmd );        //д����
		for( i = 0; i < 8; i++ )
		{
				j= Read_Byte();
				*id++ =  j;     
		}
		return ( 0 );	
}
/******************************************
���ܣ�  ��EPROM
������  tgaddr--Ŀ���ַ;
        len--Ҫ��ȡ���ֽ���;
        *buffer--��ŵ�ַ
���أ�0--�����ɹ���1--���߲�����;
*******************************************/
unsigned char DS28E05_ReadMemory( unsigned char tgaddr, unsigned char len, unsigned char * buffer )
{
    unsigned char i;
    if( Reset() != 0 )
        return ( 1 );
        Write_Byte( Rom_Skip_Cmd );                		//д����  
        Write_Byte( Memory_Read_Cmd );        				//д����
        Write_Byte( tgaddr );                         //д��ַ���ֽ�
        Write_Byte( 0 );                              //д��ַ���ֽ�
		for( i = 0; i < len; i++ )
		{
			buffer[ i ] = Read_Byte();
		}
        Reset();
        return ( 0 );        
}
//дeeprom

int ds28e05WriteMemory(uint8_t address, char *buffer, int length)
{
  uint8_t a,b;
  uint8_t sf,ef;
  uint8_t sb[2]={0};
  uint8_t eb[2]={0};
  uint8_t spage, epage, npage, wpage;
  uint8_t nseg, wseg=0;
  uint8_t wBytes=0, rBytes=0, wAddr = 0;
 
  // Calculate pages
  spage = ((address & PAGE_MASK) >> 4);
  epage = (((address + length) & PAGE_MASK) >> 4);
  if (epage == NUM_PAGES) epage = NUM_PAGES - 1;
  npage = epage - spage;
	printf("spage=%d,epage=%d\n",spage,epage);
	printf("npage=%d,length=%d\n",npage,length);
  //This memory must be written respecting 16bits boundaries
  sf = (address&0x01) != 0;
  ef = ((address+length)&0x01) != 0;
	printf("sf=%d,ef=%d\n",sf,ef);
  if (ef)
  {
    DS28E05_ReadMemory( address+length,1, &eb[1]);
    eb[0] = buffer[length-1];
    length++;
  }
  if (sf)
  {
    DS28E05_ReadMemory(address-1,1, &sb[0]);
    sb[1] = buffer[0];
    length++;
    address--;
  }
  // Write pages
  for (wpage = 0; wpage <= npage; wpage++)
  {
    wAddr = address + wBytes;
    // Calculate segments to write
    if ((length - wBytes) > (BYTES_PER_PAGE))
      // Will we pass a page boudary
      if (wAddr % (SEG_SIZE * BYTES_PER_SEG) == 0)
        nseg = SEG_SIZE;
      else
        nseg = (BYTES_PER_PAGE - (wAddr % (BYTES_PER_PAGE))) >> 1;
    else
      nseg = ((length - wBytes) & SEG_MASK) >> 1;

	printf("nseg=%d\n",nseg);

    if( Reset() != 0 )
        return ( 1 );
	Write_Byte( Rom_Skip_Cmd );  
    Write_Byte(DS28E05_WRITE_MEMORY);
    Write_Byte(wAddr);
    Write_Byte(0xff);
    // Write segments within page
    for (wseg = 0; wseg < nseg; wseg++)
    {
      if (sf)
      {
        Write_Byte(sb[0]);
        Write_Byte(sb[1]);
        wBytes += 2;
        rBytes++;
        sf = 0;
      }
      else if (ef && (length - wBytes) <= 2)
      {
        Write_Byte(eb[0]);
        Write_Byte(eb[1]);
        wBytes += 2;
        rBytes++;
        ef = 0;
      }
      else
      {
        Write_Byte(buffer[rBytes]);
        Write_Byte(buffer[rBytes+1]);
        wBytes += 2;
        rBytes += 2;
      }

      a = Read_Byte();
      b = Read_Byte();

      printf("Readback: %02x %02x (%c%c)\n", a, b, a, b);

      Write_Byte(0xff);

      delay_ms(16);

      a = Read_Byte();

      printf("Verification byte: %02x\n", a);

      if(a !=0xAA)
        goto error;
    }

    Reset();;
  }

  return 1;
error:
  Reset();
  return 0;
}



