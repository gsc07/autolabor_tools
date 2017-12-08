//此为示意性代码,请根据自己系统相关函数进行适当修改
//主要流程为在串口接收中断函数中,从串口接收数据缓冲区中每次读出一个字节直到缓冲区为空,按照数据包格式判断数据头及CRC检验等.
//CRC校验正确则把payload数据按内存拷贝方式填充到sAHRSData结构体中,这样sAHRSData就是最新一包的姿态数据了
//可以直接以结构变量的方式处理相应的数据.
//有关LRESULT CSkylarkDlg::OnReceiveChar(WPARAM pPackage, LPARAM lParam),其中的pPackage等结构与相应的串口驱动程序有关,
//仅参照其示意功能对应换成自己开发系统相应的函数即可.

#define DL_CLASS_MINIAHRS                               0x0F //姿态数据类型码
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS                0x01 //姿态数据ID码

#define DATA_LINK_MAX_LENGTH  200   //datalink 数据包payload最长值

//DL Read Return Err
#define DL_NO_ERR              0x00
#define DL_UNKNOW_MESSAGE      0x01
#define DL_CHECKSUM_ERR        0x02
#define DL_PAYLOAD_LENGTH_ERR  0x04

#pragma pack(1)
typedef struct 
{
  unsigned char      Flags;            //状态位，暂不用处理
  float       Euler[3];         // 三个欧拉角
  float       Gyro[3];          //经校准后,去bias后的角速度值,度制
  float       Acc[3];           //经校准后,加速度单位g
  float       Mag[3];           //经校准后的指南针值,无量纲
}sAHRSData;
#pragma pack()

//DL package
#pragma pack(1)
typedef struct 
{
  unsigned  char   Header1;  //'T'
  unsigned char   Header2;  //'M'
  unsigned char   Class;   
  unsigned char   ID;
  unsigned short   Length;
  unsigned char   Payload[DATA_LINK_MAX_LENGTH];     
  unsigned char   CheckSum[2]; 
}sDataLink;
#pragma pack()

sAHRSData AHRSData;//AHRS数据全局变量
sDataLink DataLink;//接收数据结构



//1:此函数为VC中相应的串口接收中断函数.主要从接收缓冲区读出数据,然后按照状态转换处理串口数据,并进行数据包解析处理.
//串口消息处理
LRESULT CSkylarkDlg::OnReceiveChar(WPARAM pPackage, LPARAM lParam)
{
  int i = 0;
  int j = 0;
  int lenght = 0;
  unsigned short CheckSum;//16位CRC校验变量
  unsigned char *INT8UP;
  unsigned char Res;
  static unsigned char  DLUARTState= 0;//初始化为判断第一个数据头
  static unsigned int DLUartRDCnt=0; 
  
  pPACKAGE pPack = (pPACKAGE)pPackage;
  ASSERT(pPack);
  ASSERT(pPack->pData);
  
  lenght = pPack->iLen;
  Res=0;
  while(lenght > i)//每次处理一个字节,一直到串口接收缓冲区空
  {
    switch( DLUARTState)
    {
    case  0:   //判断第一个头字节'T'
      if(pPack->pData[i]=='T')  DLUARTState= 1;//第一个数据头正常,进入判断第二个数据头状态
      else  DLUARTState= 0;//第一个数据头错误,返回到判断第一数据数据头状态
      break;   
    case  1://判断第二个头字节'M'
      if(pPack->pData[i]=='M')  DLUARTState= 2;//第二个数据头正确,进入接收class类型码状态
      else  DLUARTState= 0; //第二个数据头错误,返回到判断第一数据头状态
      break;
    case  2://接收class号
      DataLink.Class=pPack->pData[i];
      DLUARTState= 3;    //进入接收ID码状态 
      break;
    case  3://接收ID号
      DataLink.ID=pPack->pData[i];
      DLUARTState= 4; //进入接收数据长度第一字节状态
      break;   
    case  4://接收两个字节的长度
      DataLink.Length=pPack->pData[i];
      DLUARTState= 5; //进入接收数据长度第二字节状态
      break;
    case  5://接收到两个字节的长度
      DataLink.Length=DataLink.Length|(((unsigned short)(pPack->pData[i]))<<8);
      if(DataLink.Length> DATA_LINK_MAX_LENGTH)//数据长度超出最大长度,则数据包肯定是错误的
      {
        DLUARTState= 0; //返回到判断第一个数据头状态
        Res=DL_PAYLOAD_LENGTH_ERR;
      }
      else 
      {
        DLUARTState= 6; //长度没超出范围,进入接收payload数据状态
        DLUartRDCnt=0;
      }
      break;
    case  6://接收姿态结构数据
      if(DataLink.Length==0)//没有payload数据，直接进入接收校验字节状态
      {
        DLUARTState= 7; //进入接收检验字节状态
        i--;
      }
      else
      {
        DataLink.Payload[ DLUartRDCnt]=pPack->pData[i];//把接收到的payload数据放入DataLink.Payload数组中
        DLUartRDCnt++;
        if( DLUartRDCnt==DataLink.Length )//接收payload数据完毕
        {
          DLUARTState= 7; //进入接收检验字节状态
          DLUartRDCnt=0;
        }
      }        
      break;
    case  7://接收第一个校验字节
      DataLink.CheckSum[0]=pPack->pData[i];
      DLUARTState= 8; //进入接收第二个校验字节状态
      break;
    case  8://接收第二个校验字节，表示一幁数据接收完毕
      DataLink.CheckSum[1]=pPack->pData[i];
      DLUARTState= 0; 
      INT8UP=(unsigned char*)&DataLink.Class;//指针指向需要校验校验和的第一个字节上
      
      CheckSum=CRC16(INT8UP, DataLink.Length+4);//计算接收到的数据CRC校验,CRC校验代码参照例程包
      if((CheckSum>>8==DataLink.CheckSum[1])&&((CheckSum&0x00ff)==DataLink.CheckSum[0]))//CRC校验正确
      { 
        　　　	switch(DataLink.Class)//验证Class是否正确
          　　　	{
            　　　	case DL_CLASS_MINIAHRS://Mini AHRS 类
              　　     　 switch(DataLink.ID)//验证ID号是否正确
                　　　		{
                  　　　		case DL_MINIAHRS_ATTITUDE_AND_SENSORS:
                    　　　			INT8UP=(unsigned char *) & AHRSData.Flags;//取首地址
                    　　　			for(i=0;i<DataLink.Length;i++)//数据更新到结构体中
                      　　　			{
                        　　　				(*INT8UP)=DataLink.Payload[i];
                        　　　				INT8UP++;
                        　　　			}
                                              //到此AHRSData结构体中即包含了最新的一包姿态数据，可以如下例调用相应的变量进行显示及其它处理
                    　　　			 //printf显示
                      　　　			str.Format(_T("Roll:  %6.1f \r\n Pitch: %6.1f \r\n Yaw:   %6.1f \r\n"),AHRSData.Euler[0],AHRSData.Euler[1],AHRSData.Euler[2]);//显示发送包值
                                                pDlgAHRS->m_Attitude_Data=str;//对话框变量更新　　         
                                                pDlgAHRS->UpdateData(false);//更新对话框变量
                    　　　                   break;
                    　　　      }
                         break;
                       }

          }
        break;
      default:
        break;
      }
      
      i++;
    }
    
    pPack->Clear();
    delete pPack;
    
    return Res;
    
  }
  
  
  