#include "stm32f0xx_gpio.h"
#include "stm32f0xx.h"
#include "DS28E05.h"
#include "stdio.h"

/**************操作指令码定义******************************/
/*DS28E05 ROM功能命令*/
#define Rom_Read_Cmd                0x33    //Read ROM
#define Rom_Match_Cmd               0x55    //Match ROM
#define Rom_Skip_Cmd                 0xCC    //Skip ROM
#define Rom_Search_Cmd              0xF0    //Search ROM

/*DS28E05 存储器功能命令*/
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
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;	//为系统时钟的1/8  
	fac_ms=(int16_t)fac_us*1000;//每个ms需要的systick时钟数   
}	
//延时Nus
void delay_us(int32_t nus)
{		
	int32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
//延时Nms

void delay_ms(int16_t nms)
{	 		  	  
	int32_t temp;		   
	SysTick->LOAD=(int32_t)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 



/******************************************
****************涉及DS28E05部分*************
*******************************************/
/******************************************
函数名称：GPIO_DS28E05_Out_Mode
功    能：设置DS28E05引脚为开漏输出模式
参    数：无
返回值  ：无
*******************************************/
void GPIO_DS28E05_Out_Mode( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        //开漏输出
	GPIO_Init( DS28E05_GPIO, &GPIO_InitStructure );
}
void GPIO_DS28E05_Out_Mode_init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        //开漏输出
	GPIO_Init( DS28E05_GPIO, &GPIO_InitStructure );
}
/******************************************
函数名称：GPIO_DS28E05_Input_Mode
功    能：设置DS28E05引脚为浮空输入模式
参    数：无
返回值  ：无
*******************************************/
static void GPIO_DS28E05_Input_Mode( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DS28E05_GPIO_Pin;
	//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   //浮空输入
	//	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //开漏输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init_in( DS28E05_GPIO, &GPIO_InitStructure );
}
/******************************************
1-Wire器件复位, 并检查应答;有应答返回0, 无应答返回1
*******************************************/
 unsigned char Reset( void )
{
	unsigned char testCount = 50;
    GPIO_DS28E05_Out_Mode();
    DS28E05_Write_1();
    DS28E05_Write_0();
    delay_us(60);                //复位60us低脉冲保持
    DS28E05_Write_1();                //释放总线后9us读应答
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
               写数据位1
*******************************************/
void WriteBit_1( void )
{
    DS28E05_Write_1();
    DS28E05_Write_0();        //拉低总线保持1us
    delay_us(1);
    DS28E05_Write_1();        //释放总线延时大于12us
    delay_us(20);//20us
}
/******************************************
                写数据位0
*******************************************/
void WriteBit_0( void )
{
 
    DS28E05_Write_1();
    DS28E05_Write_0();        //拉低总线保持8-16us
    delay_us(12);	//9-10us
    DS28E05_Write_1();        //释放总线延时6us
    delay_us(9);
}
/******************************************
                读数据位
*******************************************/
unsigned char value1;
static unsigned char Read_Bit( void )
{
		GPIO_DS28E05_Out_Mode();
		DS28E05_Write_1();
		DS28E05_Write_0();                //拉低总线保持1us
		delay_us(1);
		DS28E05_Write_1();                
		DS28e05_IO_IN();				//最重要的一步，一定要用寄存器操作，否则会错过读取的时间，经检测，整个读取周期为2--3微妙
		value1 = DS28E05_ReadBit();   //
		delay_us(2);
    return value1;   
}
/******************************************
                写字节
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
                读字节
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
功能：读8位家族码;48位序列号;8位CRC码;读取成功返回0
参数：*id--读取的数据存放地址
返回：0--操作成功；1--总线不可用;
*******************************************/
unsigned char DS28E05_ReadRom( unsigned char *id )
{
    unsigned char i,j;
		while(Reset());
		Write_Byte( Rom_Read_Cmd );        //写命令
		for( i = 0; i < 8; i++ )
		{
				j= Read_Byte();
				*id++ =  j;     
		}
		return ( 0 );	
}
/******************************************
功能：  读EPROM
参数：  tgaddr--目标地址;
        len--要读取的字节数;
        *buffer--存放地址
返回：0--操作成功；1--总线不可用;
*******************************************/
unsigned char DS28E05_ReadMemory( unsigned char tgaddr, unsigned char len, unsigned char * buffer )
{
    unsigned char i;
    if( Reset() != 0 )
        return ( 1 );
        Write_Byte( Rom_Skip_Cmd );                		//写命令  
        Write_Byte( Memory_Read_Cmd );        				//写命令
        Write_Byte( tgaddr );                         //写地址低字节
        Write_Byte( 0 );                              //写地址高字节
		for( i = 0; i < len; i++ )
		{
			buffer[ i ] = Read_Byte();
		}
        Reset();
        return ( 0 );        
}
//写eeprom

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



