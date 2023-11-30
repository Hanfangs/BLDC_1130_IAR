#include "USART.h"

#define USART2_RXBUFFER_LEN  256      //USART接收区容量
static u8 s_byUsart2RxBuffer[USART2_RXBUFFER_LEN]; //USART接收缓冲区
#define USART2_TXBUFFER_LEN  256       //USART发送区容量
static u8 s_byUsart2TxBuffer[USART2_TXBUFFER_LEN]; //USART发送缓冲区

static FlagStatus s_bUsart2RxEndFlag = RESET;//接收结束标志         0
static FlagStatus s_bUsart2TxEndFlag = SET;//发送结束标志           1 

static u8 s_byUsart2StartTxDelay = 0;//开始发送延时
static u8 s_byUsart2TxEndDelay = 0;//结束发送延时

uint16_t  Dat1;
uint16_t YY=0;

/*接收使能*/
void Usart2_RxEnable(void)
{
    GPIO_ResetBits(GPIOB,GPIO_Pin_11);// 隔离485
}

/*发送使能*/
void Usart2_TxEnable(void)
{
    GPIO_SetBits(GPIOB,GPIO_Pin_11);//隔离485
}

void Usart2_StartRx(void)
{
    DMA_InitTypeDef DMA_InitStructure;
  
    s_bUsart2RxEndFlag = RESET;       /////这个很重要
      
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART2->DR);              ///这个是串口的数据
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(s_byUsart2RxBuffer);           ////这个是串口的接收缓冲区   
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = USART2_RXBUFFER_LEN;                   ///256
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     ///  这个值等于0 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);    

    USART_GetFlagStatus(USART2, USART_FLAG_IDLE);
    USART_ReceiveData(USART2);                        ///把这个串口读一下 DR寄存器  空读  在DMA之前把这个空读一下GXD
    USART_ITConfig(USART2, USART_IT_IDLE,ENABLE);    ////串口空闲中断被允许  在这个中断中做了什么呢？  串口中允许的是串口空闲中断，即没有串口收到信号时，变空闲时会有中断
                                                    
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);    ////如果没有活动，则一开始，这个IDLE并不会变有效，而是在有数据之后，无数据了这个IDLE才会有效
    DMA_Cmd(DMA1_Channel6, ENABLE);
}

void Usart2_StartTx(u8 *pbyData, u16 wLength)          ////注意到这个参数长度很重要  要发多少字节就调用多少？
{
    DMA_InitTypeDef DMA_InitStructure;
    
    //2020-11-19修改  Is_Uart2StartTxDelayEnd   s_byUsart2StartTxDelay
    if(s_byUsart2StartTxDelay != 0)//延时未完成，退出       什么地方用到这个延时呢?       发送之前，先要等一下  因为如果还没发完，又马上进入的话。。。
    {
        return ;
    }
    
    s_bUsart2TxEndFlag = RESET;   ///你一旦要调用这个开始发送，此时就将这个发送完成标志设置为0  表示没有发送完成
    wLength = ( wLength < USART2_TXBUFFER_LEN) ? wLength : USART2_TXBUFFER_LEN;
    memcpy(s_byUsart2TxBuffer, pbyData, wLength);  
   
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART2->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(s_byUsart2TxBuffer);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = wLength;                            ////这里就是要发送的字节的长度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure); 
    
    DMA_ClearFlag(DMA1_FLAG_GL7);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);  ///允许DMA中断  中断里面做了什么事呢？     除了清标志，就是 Usart2_TxEnd();  而这个是将 s_byUsart2TxEndDelay=10
                                                       ///也就是说在发送结束后，等一会就是接收了 这个要等的原因是串口还没发完，就将发送允许关的话，则发送还没有结束，结果发送没能成功发出去
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE); 
    DMA_Cmd(DMA1_Channel7, ENABLE);
}

/*接收结束标志    这个初始化之后的值是RESET  表示没有接收结束      这个函数中空闲中断中调用 表示结束了      */
void Usart2_RxEnd(void)
{
    s_bUsart2RxEndFlag = SET;
}

/*发送结束标志        */
void Usart2_TxEnd(void)
{
    s_byUsart2TxEndDelay = 10;
}

/*开始发送延迟            定时器2中不断的调用这个函数  */
void Usart2_StartTxDelay(void)
{
    if( s_byUsart2StartTxDelay > 0)
    {
        s_byUsart2StartTxDelay--;
    }
}

/*发送结束延时 发送延时结束，这个接收会被允许       这个函数一直被调用的 在定时器2中断中 */
void Usart2_TxEndDelay(void)
{
    if( s_byUsart2TxEndDelay > 0)
    {
        s_byUsart2TxEndDelay--;
        
        if(s_byUsart2TxEndDelay == 0)
        {
            s_bUsart2TxEndFlag = SET;
            //gs_Motor_Param.bySendDataFlag = ACK_IDLE;//应答空闲    即给个标志，  又给个状态 
            Usart2_RxEnable();
        }
    }
}

/*
*******************************************************************************
* Function Name  : Is_Uart4StartTxDelayEnd(void)
* Description    : USART发送前操作
* Input          : None
* Output         : None
* Return         : trun or false
*******************************************************************************
*/
FlagStatus Is_Uart2StartTxDelayEnd(void)
{     
    if(0 == s_byUsart2StartTxDelay)//延时完成
    {
        return SET;
    }   
    
    return RESET;
}

/*
*******************************************************************************
* Function Name  : Usart2_ReadForTx
* Description    : USART发送前操作
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************
*/
void Usart2_ReadForTx(void)
{     
    Usart2_TxEnable();
    
    s_byUsart2StartTxDelay = 6;
    
   // s_bUsart2TxEndFlag = RESET;       
}

/*判断接收是否已经完成    完成了等于1     */
FlagStatus Is_Usart2_RxEnd(void)
{
    return ((s_bUsart2RxEndFlag == 1) ?  SET:RESET);
}

/*判断发送是否已经完成*/
FlagStatus Is_Usart2_TxEnd(void)
{
    return (s_bUsart2TxEndFlag == 1) ?  SET:RESET;
}

/*计算接收到的数据的长度*/
u16 Usart2_GetRxLength(void)
{
    return ((u16)(USART2_RXBUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Channel6)));
}

/*把串口接收到的数据s_byUsart2RxBuffer拷贝到pbyData*/
void Usart2_GetRxData(u8 *pbyData, u16 wLength)
{
    memcpy(pbyData, s_byUsart2RxBuffer, wLength);
}

FlagStatus Com2_Recv(u8 *pbyData, u16 *pwLength)
{
    if (Is_Usart2_RxEnd())                    //判断接收是否已经完成  s_bUsart2RxEndFlag这个标志来决定，而这个标志呢Usart2_RxEnd()中设置为1  而这个函数在IDLE中断中会被调用 故。。。
    {
        *pwLength = Usart2_GetRxLength();     //计算接收到的数据的长度
        
        *pwLength = (*pwLength) < MAX_LEN_CMD ? (*pwLength) : MAX_LEN_CMD;  //如果*pwLength大于MAX_LEN_CMD就取MAX_LEN_CMD；

         Usart2_GetRxData(pbyData, *pwLength); //把串口接收到的数据s_byUsart2RxBuffer拷贝到pbyData
        
         Usart2_StartRx();                     //启动下一次的接收  注意在这个函数中，会将这个s_bUsart2RxEndFlag=RESET  从而这个部分不再成立。这个就是重点
        
         return SET;                          //如果接收完成了，返回TRUE
    }
    
    return RESET;                              //如果接收没有完成，返回RESET
}

void Com2_Send(u8 *pbyData, u16 wLength)
{
    Usart2_StartTx(pbyData, wLength);
}

void USART2_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; //RS485用到的GPIO 
    NVIC_InitTypeDef NVIC_InitStructure; //中断向量控制器，初始化中断优先级   
    USART_InitTypeDef USART_InitStructure;//USART配置
    
    /*开启RS485通讯时用到的GPIO的时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);     
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE); 
    
    /*初始化RS485通讯时用到的GPIO*/
        //GPIO_PinRemapConfig(GPIO_Remap_Usart2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure); */           
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;            ///RS485的允许端在这里定义了
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*开启Usart2的时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /*配置Usart2*/
    USART_InitStructure.USART_BaudRate = BAUDRATE_FRE;   //通讯波特率
    /*0：一个起始位，8个数据位，n个停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /*00：1个停止位；*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    /*0：禁止校验控制*/
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /*0：禁止CTS硬件流控制；0：禁止RTS硬件流控制；*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /*1：使能发送。1：使能接收，并开始搜寻RX引脚上的起始位。*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
    /*初始化USART3*/
    USART_Init(USART2, &USART_InitStructure); 
         
    /*使能Usart2*/
    USART_Cmd(USART2, ENABLE); 
    
    /*开启DMA1的时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//**
    
    /*设置中断向量控制器*/
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;              ////串口中断允许了  具体那个特性允许了还要看另一个函数，比如发送中断，比如接收中断等等
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);
    
    /*设置中断向量控制器*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;//**           DMA中断也允许了   中断中做了什么看一下GXD  这个是发送DMA完成中断  但是接收DMA完成中断却是没有的
    NVIC_Init(&NVIC_InitStructure);                                        ///因为接收完成，是靠IDLE中断来实现的，因为你不知道会接收多少个字节过来，只有那个空闲中断才知道
    
    Usart2_RxEnable();                       ////RS485控制引脚
    Usart2_StartRx();      ///
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
   USART_SendData(USART1,USART_ReceiveData(USART1));
	 while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	}
}

