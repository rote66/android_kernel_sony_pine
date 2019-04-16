/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: Test_FT5X46.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT5X46\FT5X46i\FT5526\FT3X17\FT5436\FT3X27\FT5526i\FT5416\FT5426\FT5435
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "focaltech_test_global.h"
#include "focaltech_test_detail_threshold.h"
#include "focaltech_test_ft5x46.h"
#include "focaltech_test_config_ft5x46.h"
//#include "Comm_FT5X46.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define IC_TEST_VERSION  "Test version: V1.1.0--2015-10-22, (sync version of FT_MultipleTest: V2.7.0.3--2015-07-13)"

/////////////////////////////////////////////////Reg 
#define DEVIDE_MODE_ADDR	0x00
#define REG_LINE_NUM	0x01
#define REG_TX_NUM	0x02
#define REG_RX_NUM	0x03
#define REG_PATTERN_5422        0x53
#define REG_MAPPING_SWITCH      0x54
#define REG_TX_NOMAPPING_NUM        0x55
#define REG_RX_NOMAPPING_NUM      0x56
#define REG_NORMALIZE_TYPE      0x16
#define REG_ScCbBuf0	0x4E
#define REG_ScWorkMode	0x44
#define REG_ScCbAddrR	0x45
#define REG_RawBuf0 0x36
#define REG_WATER_CHANNEL_SELECT 0x09
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
#define REG_FW_PROCESS					0x1A
#define REG_NOISE_SAMPLE_FRAME			0X1C
#define REG_LCD_NOISE_CONFICIENT		0X0F
#define REG_LCD_NOISE_MAX_NG			0X10
#define REG_LCD_NOISE_MODE				0X1B
#define REG_LCD_NOISE_FRAMEFAILED		0X11
#define REG_RELEASECODEID_H				0xAE
#define REG_RELEASECODEID_L				0xAF
/*[Arima_8100][bozhi_lin] 20170320 end*/

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum WaterproofType
{
	WT_NeedProofOnTest,
	WT_NeedProofOffTest,
	WT_NeedTxOnVal,
	WT_NeedRxOnVal,
	WT_NeedTxOffVal,
	WT_NeedRxOffVal,
};
/*******************************************************************************
* Static variables
*******************************************************************************/

static int m_RawData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};
static unsigned char m_ucTempData[TX_NUM_MAX * RX_NUM_MAX*2] = {0};
static bool m_bV3TP = false;

//[LG LV3][zihweishen] Touch AATS add  Panel differ test 2016/08/25 begin
static int m_DifferData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int m_absDifferData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
//[LG LV3][zihweishen] 2016/08/25 end

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
static int RxLinearity[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int TxLinearity[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int minHole[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int maxHole[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int m_NoiseData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
/*[Arima_8100][bozhi_lin] 20170320 end*/

//---------------------About Store Test Dat
//static char g_pStoreAllData[1024*8] = {0};
static char *g_pTmpBuff = NULL;
static char *g_pStoreMsgArea = NULL;
static int g_lenStoreMsgArea = 0;
static char *g_pMsgAreaLine2 = NULL;
static int g_lenMsgAreaLine2 = 0;
static char *g_pStoreDataArea = NULL;
static int g_lenStoreDataArea = 0;
static unsigned char m_ucTestItemCode = 0;
static int m_iStartLine = 0;
static int m_iTestDataCount = 0;

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/


/*******************************************************************************
* Static function prototypes
*******************************************************************************/
//////////////////////////////////////////////Communication function
static int StartScan(void);
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer);
static unsigned char GetPanelRows(unsigned char *pPanelRows);
static unsigned char GetPanelCols(unsigned char *pPanelCols);
static unsigned char GetTxSC_CB(unsigned char index, unsigned char *pcbValue);
//////////////////////////////////////////////Common function
static unsigned char GetRawData(void);
static unsigned char GetChannelNum(void);
//////////////////////////////////////////////about Test
static void InitTest(void);
static void FinishTest(void);
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
#if 1
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int RowArrayIndex, int ColArrayIndex,unsigned char Row, unsigned char Col, unsigned char ItemCount);
#else
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount);
#endif
/*[Arima_8100][bozhi_lin] 20170320 end*/
static void InitStoreParamOfTestData(void);
static void MergeAllTestData(void);
//////////////////////////////////////////////Others 
static void AllocateMemory(void);
static void FreeMemory(void);
static void ShowRawData(void);
static boolean GetTestCondition(int iTestType, unsigned char ucChannelValue);

static unsigned char GetChannelNumNoMapping(void);
static unsigned char SwitchToNoMapping(void);
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
unsigned char FT5X46_TestItem_LCD_NoiseTest(bool * bTestResult);   //add by LiuWeiGuang
unsigned char FT5X46_TestItem_SITORawdataUniformityTest(bool * bTestResult);  // add by LiuWeiGuang
/*[Arima_8100][bozhi_lin] 20170320 end*/


/************************************************************************
* Name: FT5X46_StartTest
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
boolean FT5X46_StartTest()
{
	bool bTestResult = true;
	bool bTempResult = 1;
	unsigned char ReCode=0;
	unsigned char ucDevice = 0;
	int iItemCount=0;
	
	//--------------1. Init part
	InitTest();
	
	//--------------2. test item

//[LG LV3][zihweishen] Touch AATS add  Panel differ test 2016/08/25 begin
	if(0 == g_TestItemNum)
		{
		bTestResult = false;
		}
	else
		{
	          g_TestItemNum++;
		}	  
//[LG LV3][zihweishen] 2016/08/25 end
	
	for(iItemCount = 0; iItemCount < g_TestItemNum; iItemCount++)
	{
		m_ucTestItemCode = g_stTestItem[ucDevice][iItemCount].ItemCode;

		///////////////////////////////////////////////////////FT5X22_ENTER_FACTORY_MODE
		if(Code_FT5X22_ENTER_FACTORY_MODE == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5X46_TestItem_EnterFactoryMode();
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT5X22_CHANNEL_NUM_TEST
		/*if(Code_FT5X22_CHANNEL_NUM_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
		)
		{
		ReCode = FT5X46_TestItem_ChannelsTest(&bTempResult);
		if(ERROR_CODE_OK != ReCode || (!bTempResult))
		{
		bTestResult = false;
		}
		}*/	

		///////////////////////////////////////////////////////FT5X22_RAWDATA_TEST
		if(Code_FT5X22_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5X46_TestItem_RawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}
//[LG LV3][zihweishen] Touch AATS add  Panel differ test 2016/08/25 begin
              ///////////////////////////////////////////////////////  Code_FT5X22_PANELDIFFER_TEST,
                            if(Code_FT5X22_PANELDIFFER_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
                            )
                   {
                            FTS_TEST_DBG("Code_FT5X22_PANELDIFFER_TEST \n");
                            ReCode = FT5X46_TestItem_PanelDifferTest(&bTempResult);
                            if(ERROR_CODE_OK != ReCode || (!bTempResult))
                            {
                                     bTestResult = false;
                                     g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
                            }
                            else
                                     g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
                   }
//[LG LV3][zihweishen] 2016/08/25 end
		
		///////////////////////////////////////////////////////FT5X22_SCAP_CB_TEST
		if(Code_FT5X22_SCAP_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5X46_TestItem_SCapCbTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT5X22_SCAP_RAWDATA_TEST
		if(Code_FT5X22_SCAP_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5X46_TestItem_SCapRawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
	 ///////////////////////////////////////////////////////Code_FT5X46_LCD_NOISE_TEST
        if (Code_FT5X22_LCD_NOISE_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
           )
        {

            ReCode = FT5X46_TestItem_LCD_NoiseTest(&bTempResult);
            if (ERROR_CODE_OK != ReCode || (!bTempResult))
            {
                bTestResult = false;
                g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
            }
            else
                g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
        }

	 ///////////////////////////////////////////////////////Code_FT5X46_SITORawdataUniformityTest
        if (Code_FT5X22_SITO_RAWDATA_UNIFORMITY_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
           )
        {

            ReCode = FT5X46_TestItem_SITORawdataUniformityTest(&bTempResult);
            if (ERROR_CODE_OK != ReCode || (!bTempResult))
            {
                bTestResult = false;
                g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
            }
            else
                g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
        }
/*[Arima_8100][bozhi_lin] 20170320 end*/
	}

	//--------------3. End Part
	FinishTest();
	
	//--------------4. return result
	return bTestResult;
}
/************************************************************************
* Name: InitTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void InitTest(void)
{
	AllocateMemory();//Allocate pointer Memory
	InitStoreParamOfTestData();
	FTS_TEST_DBG("[focal] %s \n", IC_TEST_VERSION);	//show lib version
}
/************************************************************************
* Name: FinishTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void FinishTest(void)
{
	MergeAllTestData();//Merge Test Result
	FreeMemory();//Release pointer memory
}
/************************************************************************
* Name: FT5X46_get_test_data
* Brief:  get data of test result
* Input: none
* Output: pTestData, the returned buff
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int FT5X46_get_test_data(char *pTestData)
{
	if(NULL == pTestData)
	{
		FTS_TEST_DBG("[focal] %s pTestData == NULL \n", __func__);	
		return -1;
	}
	memcpy(pTestData, g_pStoreAllData, (g_lenStoreMsgArea+g_lenStoreDataArea));
	return (g_lenStoreMsgArea+g_lenStoreDataArea);	
}

/************************************************************************
* Name: FT5X46_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_EnterFactoryMode(void)
{	
	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;	//如果不成功，重复进入5次
	int i ;
	unsigned char chPattern=0;

	SysDelay(150);
	for(i = 1; i <= iRedo; i++)
	{
		ReCode = EnterFactory();
		if(ERROR_CODE_OK != ReCode)
		{
			FTS_TEST_DBG("Failed to Enter factory mode...\n");
			if(i < iRedo)
			{
				SysDelay(50);
				continue;
			}
		}
		else
		{
			break;
		}

	}
	SysDelay(300);


	if(ReCode != ERROR_CODE_OK)	
	{	
		return ReCode;
	}

	//进工厂模式成功后，就读出通道数
	ReCode = GetChannelNum();

	////////////设置FIR，0：关闭，1：打开
	//theDevice.m_cHidDev[m_NumDevice]->WriteReg(0xFB, 0);

	//判断是否为V3屏体
	ReCode = ReadReg( REG_PATTERN_5422, &chPattern );
	if (chPattern == 1)
	{
		m_bV3TP = true;
	}
	else
	{
		m_bV3TP = false;
	}

	return ReCode;
}
/************************************************************************
* Name: FT5X46_TestItem_RawDataTest
* Brief:  TestItem: RawDataTest. Check if MCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_RawDataTest(bool * bTestResult)
{
	unsigned char ReCode = 0;
	bool btmpresult = true;
	int RawDataMin;
	int RawDataMax;
	unsigned char ucFre;
	unsigned char strSwitch = 0;
	unsigned char OriginValue = 0xff;
	int index = 0;
	int iRow, iCol;
	int iValue = 0;


	FTS_TEST_DBG("\n\n==============================Test Item: -------- Raw Data  Test \n\n");
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}


	//先判断是否为v3屏体，然后读取0x54的值，并判断与设定的mapping类型是否一致，不一致写入数据
	//rawdata test mapping后，mapping前：0x54=1;mapping后：0x54=0;
	if (m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &strSwitch );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Read REG_MAPPING_SWITCH error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
		
		if (strSwitch != 0)
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n Write REG_MAPPING_SWITCH error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}
		}			
	}

	//逐行逐列归一之后的rawdata值，0X16=0默认
	ReCode = ReadReg( REG_NORMALIZE_TYPE, &OriginValue );//读取原始值	
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n Read  REG_NORMALIZE_TYPE error. Error Code: %d\n", ReCode);
		goto TEST_ERR;
	}

	if (g_ScreenSetParam.isNormalize == Auto_Normalize)
	{
		if(OriginValue != 1)//原始值与需要改变的值不同，则写寄存器为需要的值
		{
			ReCode = WriteReg( REG_NORMALIZE_TYPE, 0x01 );
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n write  REG_NORMALIZE_TYPE error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}
		}
		//设置高频点

		FTS_TEST_DBG( "\n=========Set Frequecy High\n" );
		ReCode = WriteReg( 0x0A, 0x81 );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Set Frequecy High error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}

		FTS_TEST_DBG( "\n=========FIR State: ON\n");
		ReCode = WriteReg(0xFB, 1);//FIR OFF  0：关闭，1：打开
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n FIR State: ON error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
		
		//先前改变了寄存器 需丢三帧数据
		for (index = 0; index < 3; ++index )
		{
			ReCode = GetRawData();
		}

		if( ReCode != ERROR_CODE_OK )  
		{
			FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
			goto TEST_ERR;
		}

		ShowRawData();

		////////////////////////////////To Determine RawData if in Range or not
		for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
		{
			for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
			{
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
				RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
				RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
				iValue = m_RawData[iRow][iCol];
				if(iValue < RawDataMin || iValue > RawDataMax)
				{
					btmpresult = false;
					FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
						iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
				}
			}
		}	

		//////////////////////////////Save Test Data
		Save_Test_Data(m_RawData, 0, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1);
	}
	else
	{	
		if(OriginValue != 0)//原始值与需要改变的值不同，则写寄存器为需要的值
		{
			ReCode = WriteReg( REG_NORMALIZE_TYPE, 0x00 );	
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n write REG_NORMALIZE_TYPE error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}
		}

		ReCode =  ReadReg( 0x0A, &ucFre );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Read frequency error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}


		//设置低频点
		if(g_stCfg_FT5X22_BasicThreshold.RawDataTest_SetLowFreq)
		{
			FTS_TEST_DBG("\n=========Set Frequecy Low\n");
			ReCode = WriteReg( 0x0A, 0x80 );
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n write frequency error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}

			//FIR OFF  0：关闭，1：打开

			FTS_TEST_DBG("\n=========FIR State: OFF\n" );
			ReCode = WriteReg(0xFB, 0);
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n FIR State: OFF error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}
			SysDelay(100);
			//先前改变了寄存器 需丢三帧数据
			for (index = 0; index < 3; ++index )
			{
				ReCode = GetRawData();
			}

			if( ReCode != ERROR_CODE_OK )  
			{
				FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
				goto TEST_ERR;
			}
			ShowRawData();

			////////////////////////////////To Determine RawData if in Range or not
			for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
			{

				for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
				{
					if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
					RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
					RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
					iValue = m_RawData[iRow][iCol];
					if(iValue < RawDataMin || iValue > RawDataMax)
					{
						btmpresult = false;
						FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
							iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
					}
				}
			}

			//////////////////////////////Save Test Data
			Save_Test_Data(m_RawData, 0, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1);
		}


		//设置高频点
		if ( g_stCfg_FT5X22_BasicThreshold.RawDataTest_SetHighFreq )
		{

			FTS_TEST_DBG( "\n=========Set Frequecy High\n");
			ReCode = WriteReg( 0x0A, 0x81 );
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n Set Frequecy High error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}

			//FIR OFF  0：关闭，1：打开

			FTS_TEST_DBG("\n=========FIR State: OFF\n" );
			ReCode = WriteReg(0xFB, 0);
			if(ReCode != ERROR_CODE_OK)		
			{
				FTS_TEST_DBG("\n FIR State: OFF error. Error Code: %d\n", ReCode);
				goto TEST_ERR;
			}
			SysDelay(100);
			//先前改变了寄存器 需丢三帧数据
			for (index = 0; index < 3; ++index )
			{
				ReCode = GetRawData();
			}

			if( ReCode != ERROR_CODE_OK )  
			{
				FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
				if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
			}
			ShowRawData();

			////////////////////////////////To Determine RawData if in Range or not
			for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
			{

				for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
				{
					if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
					RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
					RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
					iValue = m_RawData[iRow][iCol];
					if(iValue < RawDataMin || iValue > RawDataMax)
					{
						btmpresult = false;
						FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
							iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
					}
				}
			}

			//////////////////////////////Save Test Data
			Save_Test_Data(m_RawData, 0, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 2);
		}

	}



	ReCode = WriteReg( REG_NORMALIZE_TYPE, OriginValue );//恢复原来寄存器值
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n Write REG_NORMALIZE_TYPE error. Error Code: %d\n", ReCode);
		goto TEST_ERR;
	}

	//恢复v3屏体的mapping值
	if (m_bV3TP)
	{
		ReCode = WriteReg( REG_MAPPING_SWITCH, strSwitch );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Write REG_MAPPING_SWITCH error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
	}

	//-------------------------Result
	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//RawData Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//RawData Test is NG!\n");
	}
	return ReCode;

TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//RawData Test is NG!\n");
	return ReCode;

}
/************************************************************************
* Name: FT5X46_TestItem_SCapRawDataTest
* Brief:  TestItem: SCapRawDataTest. Check if SCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_SCapRawDataTest(bool * bTestResult)
{
	int i =0;
	int RawDataMin = 0;
	int RawDataMax = 0; 
	int Value = 0;
	boolean bFlag = true;
	unsigned char ReCode = 0;
	boolean btmpresult = true;
	int iMax=0;
	int iMin=0;
	int iAvg=0;
	int ByteNum=0;
	unsigned char wc_value = 0;//waterproof channel value
	unsigned char ucValue = 0;
	int iCount = 0;
	int ibiggerValue = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Scap RawData Test \n\n");
	//-------1.Preparatory work	
	//in Factory Mode
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//get waterproof channel setting, to check if Tx/Rx channel need to test
	ReCode = ReadReg( REG_WATER_CHANNEL_SELECT, &wc_value );
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to read REG_WATER_CHANNEL_SELECT. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//If it is V3 pattern, Get Tx/Rx Num again
	ReCode= SwitchToNoMapping();
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to SwitchToNoMapping. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//-------2.Get SCap Raw Data, Step:1.Start Scanning; 2. Read Raw Data
/*[Arima_8100][bozhi_lin] touch aats test change test case flow and add test_result permission 20170720 begin*/
	/*ReCode = StartScan();
	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to Scan SCap RawData! \n");
		goto TEST_ERR;
	}*/
	for(i = 0; i < 3; i++)
	{
		ReCode = StartScan();
		if(ReCode != ERROR_CODE_OK)
		{	
			FTS_TEST_DBG("Failed to Scan SCap RawData! \n");
			goto TEST_ERR;
		}
/*[Arima_8100][bozhi_lin] 20170720 end*/
		memset(m_iTempRawData, 0, sizeof(m_iTempRawData));

		//防水rawdata
		ByteNum = (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)*2;
		ReCode = ReadRawData(0, 0xAC, ByteNum, m_iTempRawData);
		if(ReCode != ERROR_CODE_OK)
		{	
			FTS_TEST_DBG("Failed to ReadRawData water! \n");
			goto TEST_ERR;
		}
		
		memcpy( m_RawData[0+g_ScreenSetParam.iTxNum], m_iTempRawData, sizeof(int)*g_ScreenSetParam.iRxNum );
		memcpy( m_RawData[1+g_ScreenSetParam.iTxNum], m_iTempRawData + g_ScreenSetParam.iRxNum, sizeof(int)*g_ScreenSetParam.iTxNum );

		//非防水rawdata
		ByteNum = (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)*2;
		ReCode = ReadRawData(0, 0xAB, ByteNum, m_iTempRawData);
		if(ReCode != ERROR_CODE_OK)
		{	
			FTS_TEST_DBG("Failed to ReadRawData no water! \n");
			goto TEST_ERR;
		}
		memcpy( m_RawData[2+g_ScreenSetParam.iTxNum], m_iTempRawData, sizeof(int)*g_ScreenSetParam.iRxNum );
		memcpy( m_RawData[3+g_ScreenSetParam.iTxNum], m_iTempRawData + g_ScreenSetParam.iRxNum, sizeof(int)*g_ScreenSetParam.iTxNum );	
	}


	//-----3. Judge

	//Waterproof ON
	bFlag=GetTestCondition(WT_NeedProofOnTest, wc_value);		
	if(g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_SetWaterproof_ON && bFlag )
	{
		iCount = 0;
		RawDataMin = g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_ON_Min;
		RawDataMax = g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_ON_Max;
		iMax = -m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOnVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("Judge Rx in Waterproof-ON:\n");
		for( i = 0; bFlag && i < g_ScreenSetParam.iRxNum; i++ )
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Min[0][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Max[0][i];				
			Value = m_RawData[0+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;//find the Max value
			if(iMin > Value) iMin = Value;//fine the min value
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("Judge Tx in Waterproof-ON:\n");
		for(i = 0;bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Min[1][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Max[1][i];				
			Value = m_RawData[1+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;//find the Max value
			if(iMin > Value) iMin = Value;//fine the min value
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap RawData in Waterproof-ON, Max : %d, Min: %d, Deviation: %d, Average: %d\n", iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+0, 0, 2, ibiggerValue, 1);
	}

	//Waterproof OFF
	bFlag=GetTestCondition(WT_NeedProofOffTest, wc_value);
	if(g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_SetWaterproof_OFF && bFlag)
	{
		iCount = 0;
		RawDataMin = g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_OFF_Min;
		RawDataMax = g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_OFF_Max;
		iMax = -m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		
		bFlag=GetTestCondition(WT_NeedRxOffVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("Judge Rx in Waterproof-OFF:\n");
		for(i = 0; bFlag && i < g_ScreenSetParam.iRxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Min[0][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Max[0][i];
			Value = m_RawData[2+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			//FTS_TEST_DBG("zaxzax3 Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
			//strTemp += str;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		
		bFlag=GetTestCondition(WT_NeedTxOffVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("Judge Tx in Waterproof-OFF:\n");
		for(i = 0; bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;

			Value = m_RawData[3+g_ScreenSetParam.iTxNum][i];
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Min[1][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Max[1][i];
			//FTS_TEST_DBG("zaxzax4 Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap RawData in Waterproof-OFF, Max : %d, Min: %d, Deviation: %d, Average: %d\n", iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+2, 0, 2, ibiggerValue, 2);
	}
	//-----4. post-stage work
	if(m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &ucValue );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Read REG_MAPPING_SWITCH error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
		
		if (0 !=ucValue )
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			SysDelay(10); 			
			if( ReCode != ERROR_CODE_OK)	
			{
				FTS_TEST_DBG("Failed to switch mapping type!\n ");
				btmpresult = false;
			}
		}	

		//只有自容才会使用Mapping前的，所以该测试项结束以后，需要转到Mapping后
		ReCode = GetChannelNum();
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n GetChannelNum error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
	}

	//-----5. Test Result
	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//SCap RawData Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//SCap RawData Test is NG!\n");
	}
	return ReCode;

TEST_ERR:
	* bTestResult = false;
	FTS_TEST_DBG("\n\n//SCap RawData Test is NG!\n");	
	return ReCode;	
}

/************************************************************************
* Name: FT5X46_TestItem_SCapCbTest
* Brief:  TestItem: SCapCbTest. Check if SCAP Cb is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_SCapCbTest(bool* bTestResult)
{
	int i,/* j, iOutNum,*/index,Value,CBMin,CBMax;
	boolean bFlag = true;
	unsigned char ReCode;
	boolean btmpresult = true;
	int iMax, iMin, iAvg;
	unsigned char wc_value = 0;
	unsigned char ucValue = 0;
	int iCount = 0;
	int ibiggerValue = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: -----  Scap CB Test \n\n");	
	//-------1.Preparatory work	
	//in Factory Mode
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//get waterproof channel setting, to check if Tx/Rx channel need to test
	ReCode = ReadReg( REG_WATER_CHANNEL_SELECT, &wc_value );
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n Read REG_WATER_CHANNEL_SELECT error. Error Code: %d\n", ReCode);
		goto TEST_ERR;
	}

	//If it is V3 pattern, Get Tx/Rx Num again
	bFlag= SwitchToNoMapping();
	if( bFlag )
	{	
		FTS_TEST_DBG("Failed to SwitchToNoMapping! ReCode = %d. \n", ReCode);
		goto TEST_ERR;
	}

	//-------2.Get SCap Raw Data, Step:1.Start Scanning; 2. Read Raw Data
	ReCode = StartScan();
	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to Scan SCap RawData!ReCode = %d. \n", ReCode);
		goto TEST_ERR;
	}


	for(i = 0; i < 3; i++)
	{
		memset(m_RawData, 0, sizeof(m_RawData));
		memset(m_ucTempData, 0, sizeof(m_ucTempData));

		//防水CB
		ReCode = WriteReg( REG_ScWorkMode, 1 );//自容工作方式选择:  1：防水 0:非防水
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("Get REG_ScWorkMode Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = StartScan();
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("StartScan Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = WriteReg( REG_ScCbAddrR, 0 );	
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("Write REG_ScCbAddrR Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = GetTxSC_CB( g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum + 128, m_ucTempData );
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("GetTxSC_CB Failed!\n");
			goto TEST_ERR;
		}
		
		for ( index = 0; index < g_ScreenSetParam.iRxNum; ++index )
		{
			m_RawData[0 + g_ScreenSetParam.iTxNum][index]= m_ucTempData[index];
		}
		for ( index = 0; index < g_ScreenSetParam.iTxNum; ++index )
		{
			m_RawData[1 + g_ScreenSetParam.iTxNum][index] = m_ucTempData[index + g_ScreenSetParam.iRxNum];
		}

		//非防水rawdata
		ReCode = WriteReg( REG_ScWorkMode, 0 );//自容工作方式选择:  1：防水 0:非防水
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("Get REG_ScWorkMode Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = StartScan();
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("StartScan Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = WriteReg( REG_ScCbAddrR, 0 );	
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("Write REG_ScCbAddrR Failed!\n");
			goto TEST_ERR;
		}
		
		ReCode = GetTxSC_CB( g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum + 128, m_ucTempData );
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("GetTxSC_CB Failed!\n");
			goto TEST_ERR;
		}
		for ( index = 0; index < g_ScreenSetParam.iRxNum; ++index )
		{
			m_RawData[2 + g_ScreenSetParam.iTxNum][index]= m_ucTempData[index];
		}
		for ( index = 0; index < g_ScreenSetParam.iTxNum; ++index )
		{
			m_RawData[3 + g_ScreenSetParam.iTxNum][index] = m_ucTempData[index + g_ScreenSetParam.iRxNum];
		}

		if( ReCode != ERROR_CODE_OK )	
		{
			FTS_TEST_DBG("Failed to Get SCap CB!\n");
		}		
	}

	if(ReCode != ERROR_CODE_OK)	goto TEST_ERR;

	//-----3. Judge

	//Waterproof ON
	bFlag=GetTestCondition(WT_NeedProofOnTest, wc_value);			
	if(g_stCfg_FT5X22_BasicThreshold.SCapCbTest_SetWaterproof_ON && bFlag)
	{
		FTS_TEST_DBG("SCapCbTest in WaterProof On Mode:  \n");

		iMax = -m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		iCount = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Rx:  \n");
		for( i = 0;bFlag && i < g_ScreenSetParam.iRxNum; i++ )
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Min[0][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Max[0][i];
			Value = m_RawData[0+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			if(iMax < Value) iMax = Value;//find the Max Value
			if(iMin > Value) iMin = Value;//find the Min Value
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Tx:  \n");
		for(i = 0;bFlag &&  i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Min[1][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Max[1][i];
			Value = m_RawData[1+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap CB in Waterproof-ON, Max : %d, Min: %d, Deviation: %d, Average: %d\n", iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+0, 0, 2, ibiggerValue, 1);
	}

	bFlag=GetTestCondition(WT_NeedProofOffTest, wc_value);
	if(g_stCfg_FT5X22_BasicThreshold.SCapCbTest_SetWaterproof_OFF && bFlag)
	{			
		FTS_TEST_DBG("SCapCbTest in WaterProof OFF Mode:  \n");
		iMax = -m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		iCount = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOffVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Rx:  \n");
		for(i = 0;bFlag &&  i < g_ScreenSetParam.iRxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Min[0][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Max[0][i];
			Value = m_RawData[2+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOffVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Tx:  \n");
		for(i = 0; bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			//if( m_ScapInvalide[1][i] == 0 )      continue;
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Min[1][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Max[1][i];
			Value = m_RawData[3+g_ScreenSetParam.iTxNum][i];

			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):\n", i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap CB in Waterproof-OFF, Max : %d, Min: %d, Deviation: %d, Average: %d\n", iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+2, 0, 2, ibiggerValue, 2);
	}
	//-----4. post-stage work
	if(m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &ucValue );
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n Read REG_MAPPING_SWITCH error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
		
		if (0 != ucValue )
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			SysDelay(10); 			
			if( ReCode != ERROR_CODE_OK)	
			{
				FTS_TEST_DBG("Failed to switch mapping type!\n ");
				btmpresult = false;
			}
		}	

		//只有自容才会使用Mapping前的，所以该测试项结束以后，需要转到Mapping后
		ReCode = GetChannelNum();
		if(ReCode != ERROR_CODE_OK)		
		{
			FTS_TEST_DBG("\n GetChannelNum error. Error Code: %d\n", ReCode);
			goto TEST_ERR;
		}
	}

	//-----5. Test Result

	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//SCap CB Test Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//SCap CB Test Test is NG!\n");
	}
	return ReCode;

TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//SCap CB Test Test is NG!\n");
	return ReCode;	
}

//[LG LV3][zihweishen] Touch AATS add  Panel differ test 2016/08/25 begin
/************************************************************************
* Name: FT5X46_TestItem_PanelDifferTest
* Brief:  TestItem: PanelDifferTest. 
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_PanelDifferTest(bool * bTestResult)
{
         int index = 0;
         int iRow = 0, iCol = 0;
         int iValue = 0;
         unsigned char ReCode = 0,strSwitch = -1;
         bool btmpresult = true;
         int iMax, iMin; //, iAvg;
         int maxValue=0;
         int minValue=32767;
         int AvgValue = 0;
         int InvalidNum=0;
         int i = 0,  j = 0;
         
         unsigned char OriginRawDataType = 0xff;
         unsigned char OriginFrequecy = 0xff;
         unsigned char OriginFirState = 0xff;
 
         FTS_TEST_DBG("");
         FTS_TEST_DBG("\r\n\r\n\r\n==============================Test Item: -------- Panel Differ Test  \r\n\r\n");
 
         ReCode = EnterFactory(); 
         SysDelay(20);
         if(ReCode != ERROR_CODE_OK)               
         {
                   FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
                   goto TEST_ERR;
         }
 
         //-琌-v3蔨礛-0x54-蒓-﹚mapping-琌璓ぃ璓--誹
         //rawdata test mappingmapping玡0x54=1;mapping0x54=0;
         if (m_bV3TP)
         {
                   ReCode = ReadReg( REG_MAPPING_SWITCH, &strSwitch );
                   if(ReCode != ERROR_CODE_OK)               
                   {
                            FTS_TEST_DBG("\n Read REG_MAPPING_SWITCH error. Error Code: %d",  ReCode);
                            goto TEST_ERR;
                   }
                   
                   if (strSwitch != 0)
                   {
                            ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
                            if(ReCode != ERROR_CODE_OK)               
                            {
                                     FTS_TEST_DBG("\n Write REG_MAPPING_SWITCH error. Error Code: %d",  ReCode);
                                     goto TEST_ERR;
                            }
                            //      MUST get channel number again after write reg mapping switch.
                            ReCode = GetChannelNum();
                            if(ReCode != ERROR_CODE_OK)               
                            {
                                     FTS_TEST_DBG("\n GetChannelNum error. Error Code: %d",  ReCode);
                                     goto TEST_ERR;
                            }
                            
                            ReCode = GetRawData();                  
                   }                           
         }
 
         ///////////
         FTS_TEST_DBG("\r\n=========Set Auto Equalization:\r\n");
         ReCode = ReadReg( REG_NORMALIZE_TYPE, &OriginRawDataType );//-﹍ 
         if(ReCode != ERROR_CODE_OK)               
         {
                   FTS_TEST_DBG("Read  REG_NORMALIZE_TYPE error. Error Code: %d",  ReCode);
                   btmpresult = false;
                   goto TEST_ERR;
         }
         
         if (OriginRawDataType != 0)
         {
                   ReCode =WriteReg( REG_NORMALIZE_TYPE, 0x00 );
                   SysDelay(50);
                   if( ReCode != ERROR_CODE_OK )
                   {
                            btmpresult = false;
                            FTS_TEST_DBG("Write reg REG_NORMALIZE_TYPE Failed." );
                            goto TEST_ERR;
                   }
         }
         ///////////
 
         //-竚蔼--
         FTS_TEST_DBG("=========Set Frequecy High" );
         ReCode =ReadReg( 0x0A, &OriginFrequecy);//-﹍  
         if( ReCode != ERROR_CODE_OK )
         {
                   FTS_TEST_DBG("Read reg 0x0A error. Error Code: %d",  ReCode);
                   btmpresult = false;
                   goto TEST_ERR;
         }
 
         ReCode = WriteReg( 0x0A, 0x81);
         SysDelay(10);
         if( ReCode != ERROR_CODE_OK )
         {
                   btmpresult = false;
                   FTS_TEST_DBG("Write reg 0x0A Failed." );
                   goto TEST_ERR;
         }
 
         FTS_TEST_DBG("=========FIR State: OFF" );
         ReCode = ReadReg( 0xFB, &OriginFirState);//-﹍    
         if( ReCode != ERROR_CODE_OK )
         {
                   FTS_TEST_DBG("Read reg 0xFB error. Error Code: %d",  ReCode);
                   btmpresult = false;
                   goto TEST_ERR;
         }
         ReCode = WriteReg( 0xFB, 0);          
         SysDelay(50);
         if( ReCode != ERROR_CODE_OK )
         {
                   FTS_TEST_DBG("Write reg 0xFB Failed." );
                   btmpresult = false;
                   goto TEST_ERR;
         }
         ReCode = GetRawData();
 
         //玡э-盚竟 惠---誹材4-琌ㄏノ-誹
         for ( index = 0; index < 4; ++index )
         {
                   ReCode = GetRawData();
                   if( ReCode != ERROR_CODE_OK )
                   {
                            FTS_TEST_DBG("GetRawData Failed." );
                            btmpresult = false;
                            goto TEST_ERR;
                   }
         }
 
         ////Differ-RawData1/10
         for(i = 0; i < g_ScreenSetParam.iTxNum; i++)
         {
                   for( j= 0; j < g_ScreenSetParam.iRxNum; j++)
                   {
                            m_DifferData[i][j] = m_RawData[i][j]/10;
                   }
         }
 
         ////////////////////////////////
         ///--琌璖-ぇ-
         /*
         {
                   NodeVal nodeOutRange;
                   AnalyzeInfo info( m_iTxNum, m_iRxNum, true );
                   bool bResult = AnalyzeTestResultMCap( m_DifferData, g_stCfg_MCap_DetailThreshold.PanelDifferTest_Min, g_stCfg_MCap_DetailThreshold.PanelDifferTest_Max, 
                            g_stCfg_MCap_DetailThreshold.InvalidNode, info, textTemp, nodeOutRange );
 
                   if( !bResult )
                   {
                            CString strHead = _T("\r\n//========= Out of Threshold in PannelDiffer Test: \r\n");
                            btmpresult = false;
                   }
         }
         */
         ////////////////////
 
         ////////////////////////////////To show value
#if 1
         FTS_TEST_DBG("PannelDiffer :\n");
         for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
         {
                   FTS_TEST_DBG("\nRow%2d:    ", iRow+1);
                   for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
                   {
                   //      if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
 
                            iValue = m_DifferData[iRow][iCol];
                            FTS_TEST_DBG("%4d,  ", iValue);
                   }
                 
         }
        
#endif
 
         ///--琌璖-ぇ-
         ////////////////////////////////To Determine  if in Range or not
         //iMin =  g_stCfg_FT5X22_BasicThreshold.PanelDifferTest_Min;              //         g_stCfg_FT5X22_BasicThreshold.PanelDifferTest_Min[iRow][iCol];
         //iMax = g_stCfg_FT5X22_BasicThreshold.PanelDifferTest_Max;
 
         for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)       //      iRow = 1 ???
         {
                   for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
                   {
                            if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
 
 								iMin = g_stCfg_MCap_DetailThreshold.PanelDifferTest_Min[iRow][iCol];
								iMax = g_stCfg_MCap_DetailThreshold.PanelDifferTest_Max[iRow][iCol];
 
                            iValue = m_DifferData[iRow][iCol];
                            if(iValue < iMin || iValue > iMax)
                            {
                                     btmpresult = false;
                                     printk("\nOut Of Range.  Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
                                               iRow+1, iCol+1, iValue, iMin, iMax);                                                       
                            }
                   }
         }
         ///////////////////////////  end determine
         
         ////////////////////
         ////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Μ栋---誹CSVゅン
         FTS_TEST_DBG("PannelDiffer ABS:\n");
         for( i = 0; i <  g_ScreenSetParam.iTxNum; i++)
         {                 
         //      strTemp += "\r\n";
                  // FTS_TEST_PRINT("\n");
                   for( j = 0; j <  g_ScreenSetParam.iRxNum; j++)
                   {        
                   //      str.Format("%d,", abs(m_DifferData[i][j]));
                   //      strTemp += str;
                            FTS_TEST_DBG("%ld,", abs(m_DifferData[i][j]));
                            m_absDifferData[i][j] = abs(m_DifferData[i][j]); 
 
                            if( NODE_AST_TYPE == g_stCfg_MCap_DetailThreshold.InvalidNode[i][j] || NODE_INVALID_TYPE == g_stCfg_MCap_DetailThreshold.InvalidNode[i][j])
                            {
                                     InvalidNum++;
                                     continue;
                            }
                            maxValue = max(maxValue,m_DifferData[i][j]);
                            minValue = min(minValue,m_DifferData[i][j]);
                            AvgValue += m_DifferData[i][j];
                   }
         }
         
//      SaveTestData(strTemp, m_iTxNum, m_iRxNum);
         Save_Test_Data(m_absDifferData, 0, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1);
         ////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Μ栋---誹CSVゅン
 
         AvgValue = AvgValue/( g_ScreenSetParam.iTxNum*g_ScreenSetParam.iRxNum - InvalidNum); 
         printk("PanelDiffer:Max: %d, Min: %d, Avg: %d ", maxValue, minValue,AvgValue);
 
         ReCode = WriteReg( REG_NORMALIZE_TYPE, OriginRawDataType );//蝋-盚竟
         ReCode = WriteReg( 0x0A, OriginFrequecy );//蝋-盚竟
         ReCode = WriteReg( 0xFB, OriginFirState );//蝋-盚竟
         
         //蝋v3蔨mapping
         if (m_bV3TP)
         {
                   ReCode = WriteReg( REG_MAPPING_SWITCH, strSwitch );
                   if( ReCode != ERROR_CODE_OK)    
                   {
                            FTS_TEST_DBG("Failed to restore mapping type!");
                            btmpresult = false;
                   }
         }
         /////////////
 
         ///////////////////////////-------------------------Result
         if( btmpresult )
         {
                   *bTestResult = true;
                   printk("\n               //Panel Differ Test is OK!\n");
         }
         else
         {
                   * bTestResult = false;
                   printk("\n               //Panel Differ Test is NG!\n");
         }
         return ReCode;
 
TEST_ERR:
 
         * bTestResult = false;
         printk("\n               //Panel Differ Test is NG!\n");
         return ReCode;
}
//[LG LV3][zihweishen] 2016/08/25 end

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
unsigned char FT5X46_TestItem_LCD_NoiseTest(bool * bTestResult)
{

	bool bResultFlag = true;
	bool bTempFlag = true;
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char regData = 0,strSwitch=0,chNoiseValue=0;
	int NoiseValue = 0;
	unsigned char chFreq=0,chFir=0;
	unsigned char  ReleaseIDH=0, ReleaseIDL=0;
	int ReleaseID = 0;
	int i = 0, j = 0;
	int index = 0;
	unsigned char ReleaseIDLow = 0, ReleaseIDHigh = 0;
	int maxNG = 0;
	int continueNG = 0;
	int iRow = 0, iCol = 0;
	int NoiseDataMin = 0;
	int NoiseDataMax = 0;
	int iValue = 0;
	int maxValue=0;
	int minValue=10000.0;

	printk("\r\n\r\n==============================Test Item: -------- LCD Noise Test\r\n");

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement add SW reset 20170711 begin*/
	ReCode = EnterWork();
	if(ReCode != ERROR_CODE_OK)
	{
		printk("\r\n Failed to Enter work Mode. Error Code: %d", ReCode);
		bResultFlag = false;
		goto TEST_END;
	}
	SysDelay(50);

	ReCode = WriteReg( 0xfc, 0xaa );
	if( ReCode != ERROR_CODE_OK)
	{
		printk("\r\nFailed to Write Reg!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}
	ReCode = WriteReg( 0xfc, 0x66 );
	if( ReCode != ERROR_CODE_OK)
	{
		printk("\r\nFailed to Write Reg!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}
	SysDelay(300);
/*[Arima_8100][bozhi_lin] 20170711 end*/

	//enter factory mode
	ReCode = EnterFactory();
	SysDelay(20);
	if( ReCode != ERROR_CODE_OK )
	{
		printk("\r\nFailed to Enter factory Mode!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}

	ReCode = ReadReg( REG_FW_PROCESS, &strSwitch );
	if ((ReCode != ERROR_CODE_OK))
	{
		printk("\r\nFailed to Read Reg!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}
	if (1 != strSwitch)
	{
		printk("\r\nFW Don't support FW mode to test LcdNoiseTest!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}

		//
	ReCode = WriteReg( 0x06, 0x01 );
	SysDelay( 100 );
	if( ReCode != ERROR_CODE_OK)
	{
		printk("\r\nFailed to Write Reg!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}

	//
	ReCode = ReadReg(0xFB, &chFir);
	SysDelay( 10 );
	if (1 != chFir)
	{
		ReCode = WriteReg(0xFB, 1);
		SysDelay( 100 );
		if( ReCode != ERROR_CODE_OK )
		{
			printk("\r\nFailed to Read or Write Reg!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}
		GetRawData();
	}

	//
	ReCode = ReadReg(0x0A, &chFreq);
	SysDelay( 10 );
	if (chFreq != g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_SetFrequency )
	{
		ReCode = WriteReg( 0x0A,g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_SetFrequency);
		SysDelay( 10 );
		//
		for ( i=0; i<30; i++)
		{
				StartScan();
				SysDelay( 20 );
		}
	}

	//
       ReCode =WriteReg(REG_NOISE_SAMPLE_FRAME, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_MaxFrame);
	SysDelay(10);
	//
	ReCode = WriteReg(REG_LCD_NOISE_CONFICIENT, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Conficient);
	SysDelay(10);
	//
	ReCode =WriteReg(REG_LCD_NOISE_MAX_NG, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_MaxNgPoint);
	SysDelay(10);
	//
	ReCode = WriteReg(REG_LCD_NOISE_MODE, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Noise_Mode);
       SysDelay(20);

	if( g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Check_Continue_NG )
	{
		ReCode = WriteReg( 0x13, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Continue_Coefficient);
		SysDelay(10);

	}

	for (  index = 0; index < 3; index++ )
	{
		ReCode = GetRawData();
		if (ReCode == ERROR_CODE_OK)
			break;

		if (index >= 2)
		{
			if (ReCode != ERROR_CODE_OK)
			{
				printk("\r\nGet noise Data, Times: %d, Error Code: 0x%x", index+1, ReCode);
				bResultFlag = false;
				goto TEST_END;
			}
		}
	}

	memset(m_NoiseData, 0, sizeof(m_NoiseData));
	for (i=0;i<TX_NUM_MAX;i++)
	{
		for ( j=0;j<RX_NUM_MAX;j++)
		{
			m_NoiseData[i][j] = m_RawData[i][j];
		}
	}

	ReCode = ReadReg(REG_LCD_NOISE_FRAMEFAILED, &regData);
	if ((ReCode != ERROR_CODE_OK))
	{
		printk("\r\nFailed to Read Reg!\r\n ");
		bResultFlag = false;
		goto TEST_END;
	}

	if( g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Check_Continue_NG )
	{
		ReCode = EnterWork();
		SysDelay(200);
		if ((ReCode != ERROR_CODE_OK))
		{
			printk("\r\nFailed to enter work!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}

		ReCode = ReadReg(0xAF, &ReleaseIDLow);
		SysDelay( 10 );
		if ((ReCode != ERROR_CODE_OK))
		{
			printk("\r\nFailed to Read Reg!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}

		ReCode =ReadReg(0xAE, &ReleaseIDHigh);
		SysDelay( 10 );
		if ((ReCode != ERROR_CODE_OK))
		{
			printk("\r\nFailed to Read Reg!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}

		ReleaseID = (ReleaseIDHigh<<8) + ReleaseIDLow;


		//
		//
		//
		if(ReleaseID < 0x010F)
		{
			printk("\r\nReleaseID = 0X%.4X, this FW is to old, please update to latest FW which release ID should be equal or greater than 0X0112\r\n", ReleaseID);
			bResultFlag = false;
			goto TEST_END;
		}
		else if((0x010F <= ReleaseID) && (ReleaseID < 0x0112))
		{
		       maxNG = regData & 0x3F;
			printk("\r\nGet Max Ng Frame = %d, Set Max Ng Frame = %d\r\n",maxNG, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum);
			if(maxNG > g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum)
			{
				bResultFlag = false;
			}
		}
		else
		{
		       continueNG = (regData >> 6) & 0x03;
			maxNG = regData & 0x3F;
			printk("\r\nGet Max Ng Frame = %d, Set Max Ng Frame = %d,  Get Continue Ng Frame = %d, Set Continue Ng Frame = %d\r\n",
				maxNG, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum, continueNG, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Continue_NG_Frame );
			if(maxNG > g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum)
			{
				bResultFlag = false;
			}
			if(continueNG > g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_Continue_NG_Frame)
			{
				bResultFlag = false;
			}
		}
		ReCode = EnterFactory();
		if ((ReCode != ERROR_CODE_OK))
		{
			printk("\r\nFailed to enter factory!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}

	}
	else
	{
		printk("\r\nGet Max Ng Frame = %d, Set Max Ng Frame = %d\r\n", regData, g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum );
		if(regData > g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_FrameNum)
		{
			bResultFlag = false;
		}
	}

	if(!g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_NoiseThresholdMode)
	{
		ReCode = ReadReg(0x0D, &chNoiseValue);
		SysDelay(5);
		printk("\r\nLCD Noise:Original Value: %d \r\n", chNoiseValue);

		ReCode = EnterWork();
		SysDelay(200);
		if(ReCode != ERROR_CODE_OK)
		{
				printk("\r\n\r\n// Failed to Enter work Mode. Error Code: %d", ReCode);
				bResultFlag = false;
				goto TEST_END;
		}
		ReCode =ReadReg(REG_RELEASECODEID_L, &ReleaseIDL);
		SysDelay(2);
		ReCode = ReadReg(REG_RELEASECODEID_H, &ReleaseIDH);
		SysDelay(2);
		if(ReCode != ERROR_CODE_OK)
		{
				printk("\r\nRead:AF:%d, AE:%d\r\n",ReleaseIDL,ReleaseIDH);
				bResultFlag = false;
				goto TEST_END;
		}

		//
		ReleaseID = (ReleaseIDH<<8) + ReleaseIDL;
		if (ReleaseID >= 0x010e)
		{
			NoiseValue = chNoiseValue*4;
		}else
		{
			NoiseValue = chNoiseValue;
		}
		ReCode =EnterFactory();
		SysDelay(100);
		if(ReCode != ERROR_CODE_OK)
		{
			printk("\r\n\r\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
			bResultFlag = false;
			goto TEST_END;
		}
	}

	for (  iRow = 0; iRow < g_ScreenSetParam.iTxNum; ++iRow )
	{
		for (  iCol = 0; iCol < g_ScreenSetParam.iRxNum; ++iCol )
		{
			if (g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_NoiseThresholdMode)
			{
				maxHole[iRow][iCol] = g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_NoiseMax;

			}else
			{
				maxHole[iRow][iCol] = (int)g_stCfg_MCap_DetailThreshold.LCDNoistTest_Coefficient[iRow][iCol]*NoiseValue/100;
			}
		}
	}


	//  Show Value
	for (iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
	{
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			printk("NoiseData[%d][%d] = %d",  iRow+1, iCol+1, m_NoiseData[iRow][iCol]);
		}
	}

	   ////////////////////////////////To Determine NoiseData if in Range or not
        for (iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
        {
            for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
            {
                if (g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue; //Invalid Node
                NoiseDataMin = minHole[iRow][iCol];
                NoiseDataMax = maxHole[iRow][iCol];
                iValue = m_NoiseData[iRow][iCol];
                if (iValue < NoiseDataMin || iValue > NoiseDataMax)
                {
                    bResultFlag = false;
                    printk("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
                                   iRow+1, iCol+1, iValue, NoiseDataMin, NoiseDataMax);
                }
            }
        }

	

	//cal max min
	for( i = 0; i <  g_ScreenSetParam.iTxNum; i++)
	{
		for( j = 0; j < g_ScreenSetParam.iRxNum; j++)
		{
			if( NODE_INVALID_TYPE == g_stCfg_MCap_DetailThreshold.InvalidNode[i][j] )
			{
				continue;
			}
			maxValue = max(maxValue,m_NoiseData[i][j]);
			minValue = min(minValue,m_NoiseData[i][j]);
		}
	}
	printk("\r\nLCD Noise:Max: %d, Min: %d\n", maxValue, minValue);

	//save data
	Save_Test_Data(  m_NoiseData, 0, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1 );

	//
	ReCode =  WriteReg( REG_LCD_NOISE_MODE,0x00 );
	SysDelay(50);
	if( ReCode != ERROR_CODE_OK)
	{
		printk("\r\nFailed to Read or Write Reg!\r\n ");
		bResultFlag = false;
	}

	//
	ReCode = WriteReg( 0x06,0x00 );
	SysDelay(200);
	if( ReCode != ERROR_CODE_OK)
	{
		printk("\r\nFailed to Read or Write Reg!\r\n ");
		bResultFlag = false;
	}

	if (1 != chFir)
	{
		ReCode = WriteReg(0xFB,chFir);
		SysDelay( 50 );
		if( ReCode != ERROR_CODE_OK )
		{
			printk("\r\nFailed to Read or Write Reg!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}
	}

	if (chFreq != g_stCfg_FT5X22_BasicThreshold.Lcd_Noise_SetFrequency )
	{
		ReCode = WriteReg(0x0A, chFreq);
		SysDelay( 10 );
		if( ReCode != ERROR_CODE_OK )
		{
			printk("\r\nFailed to Read or Write Reg!\r\n ");
			bResultFlag = false;
			goto TEST_END;
		}
	}

TEST_END:
	if(bResultFlag && bTempFlag)
	{
		printk("\r\n//LCD Noise Test is OK!");
		*bTestResult = true;
	}
	else
	{
		printk("\r\n//LCD Noise Test is NG!");
		*bTestResult = false;
	}

	//TestResultLen += sprintf(TestResult+TestResultLen," LCD Noise Test is %s. \n\n", (*bTestResult ? "OK" : "NG"));

	return ReCode;

}

unsigned char FT5X46_TestItem_SITORawdataUniformityTest(bool * bTestResult)
{
	unsigned char ReCode = ERROR_CODE_OK;
	bool btmpresult = true;
	unsigned char strSwitch = -1;
	unsigned char FirValue = 1;
	int index = 0;
	int iRow = 0, iCol = 0;
	int iDeviation = 0;
	int iMax = 0;
	int iBase = 0;
	int maxValue = 0;
	int minValue = 0;
	int iValue = 0;

	printk( "\r\n\r\n==============================Test Item: -------- SITO Rawdata Uniformity Test\r\n" );

	ReCode = EnterFactory();
	if(ReCode != ERROR_CODE_OK)
	{
		printk("\r\n\r\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		btmpresult = false;
	}

	//
	//Uniformity test mapping
	if (m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &strSwitch );
		if (strSwitch != 0)
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			if( ReCode != ERROR_CODE_OK)
			{
				printk("\r\nFailed to switch mapping type!\r\n ");
				btmpresult = false;
			}
		}
	}

	ReCode = ReadReg(0xFB, &FirValue);
	if (ReCode != ERROR_CODE_OK)
	{
		printk("\r\nRead fir Reg Failed.\r\n");
		btmpresult = false;
	}

	ReCode = WriteReg( 0x0A, 0x81 );
	SysDelay(10);

	if (g_ScreenSetParam.isNormalize == Auto_Normalize)
	{
		ReCode = WriteReg(0xFB, 1);

	}else
	{
		ReCode = WriteReg(0xFB, 0);
	}
	SysDelay(100);

	//
	for ( index = 0; index < 3; ++index )
	{
		ReCode = GetRawData();
	}

	if( g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Check_Tx )
	{
		printk("\r\n=========Check Tx Linearity \r\n");

		minValue = 0;
		maxValue = g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Tx_Hole;
		for ( iRow = 0; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for ( iCol = 1; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				if( g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0 )   continue;
				if( g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol-1] == 0 ) continue;
				if( g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 2 )   continue;
				if( g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol-1] == 2 ) continue;

				iDeviation = abs( m_RawData[iRow][iCol] - m_RawData[iRow][iCol-1] );
			       iMax = max( abs(m_RawData[iRow][iCol]), abs(m_RawData[iRow][iCol-1]) );
				iMax = iMax ? iMax : 1;
				TxLinearity[iRow][iCol] = 100 * iDeviation / iMax;

				 iBase = g_stCfg_MCap_DetailThreshold.SITORawdata_TxLinearityTest_Base[iRow][iCol];
				TxLinearity[iRow][iCol] = abs( TxLinearity[iRow][iCol] - iBase );
				iValue = TxLinearity[iRow][iCol];

				printk("TxLinearity[%d][%d] = %d" ,iRow+1, iCol+1, iValue);

				if(iValue > maxValue || iValue < minValue )
				{
					 btmpresult = false;
					  printk("Tx Linearity Out Of Range.  Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
                                       iRow+1, iCol+1, iValue, minValue, maxValue);
				}

			}
		}

		Save_Test_Data(TxLinearity,  0, 1, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum-1, 1 );
	}

	if( g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Check_Rx )
	{
		printk("\r\n=========Check Rx Linearity \r\n");
		minValue = 0;
		maxValue = g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Rx_Hole;
		for ( iRow = 1; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for ( iCol = 0; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow-1][iCol] == 0)continue;
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 2)continue;//
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow-1][iCol] == 2)continue;

				iDeviation = abs( m_RawData[iRow][iCol] - m_RawData[iRow-1][iCol] );
			       iMax = max( abs(m_RawData[iRow][iCol]), abs(m_RawData[iRow-1][iCol]) );
				iMax = iMax ? iMax : 1;
				RxLinearity[iRow][iCol] = 100 * iDeviation / iMax;
				iBase = g_stCfg_MCap_DetailThreshold.SITORawdata_RxLinearityTest_Base[iRow][iCol];
				RxLinearity[iRow][iCol] = abs( RxLinearity[iRow][iCol] - iBase );
				iValue = RxLinearity[iRow][iCol];

				printk("RxLinearity[%d][%d] = %d" ,iRow+1, iCol+1, iValue);
				if(iValue > maxValue || iValue < minValue )
				{
					 btmpresult = false;
					  printk("Rx Linearity Out Of Range.  Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
                                       iRow+1, iCol+1, iValue, minValue, maxValue);
				}
			}
		}

		Save_Test_Data(RxLinearity, 1, 0, g_ScreenSetParam.iTxNum-1, g_ScreenSetParam.iRxNum, 1 );
	}

	ReCode = WriteReg(0xFB, FirValue);
	GetRawData();
	if (ReCode != ERROR_CODE_OK)
	{
		printk("\r\nWrite fir Reg  Failed.\r\n");
		btmpresult = false;
	}


	//
	if (m_bV3TP)
	{
		ReCode = WriteReg( REG_MAPPING_SWITCH, strSwitch );
		if( ReCode != ERROR_CODE_OK)
		{
			printk("\r\nFailed to restore mapping type!\r\n ");
			btmpresult = false;
		}
	}

	if( btmpresult && ReCode == ERROR_CODE_OK )
	{
		printk("\r\n\r\n//SITO Rawdata Uniformity Test is OK!\r\n");
		*bTestResult = true;
	}
	else
	{
		printk("\r\n\r\n//SITO Rawdata Uniformity Test is NG!\r\n");
		*bTestResult = false;
	}

	//TestResultLen += sprintf(TestResult+TestResultLen,"SITO Rawdata Uniformity Test is %s. \n\n", (*bTestResult ? "OK" : "NG"));

	return ReCode;
}
/*[Arima_8100][bozhi_lin] 20170320 end*/

/************************************************************************
* Name: GetPanelRows(Same function name as FT_MultipleTest)
* Brief:  Get row of TP
* Input: none
* Output: pPanelRows
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelRows(unsigned char *pPanelRows)
{
	return ReadReg(REG_TX_NUM, pPanelRows);
}

/************************************************************************
* Name: GetPanelCols(Same function name as FT_MultipleTest)
* Brief:  get column of TP
* Input: none
* Output: pPanelCols
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelCols(unsigned char *pPanelCols)
{
	return ReadReg(REG_RX_NUM, pPanelCols);
}
/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0;
	unsigned char times = 0;
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
	const unsigned char MaxTimes = 250;	//最长等待160ms
/*[Arima_8100][bozhi_lin] 20170320 end*/
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
	if(ReCode == ERROR_CODE_OK)
	{
		RegVal |= 0x80;		//最高位置1，启动扫描
		ReCode = WriteReg(DEVIDE_MODE_ADDR, RegVal);
		if(ReCode == ERROR_CODE_OK)
		{
			while(times++ < MaxTimes)		//等待扫描完成
			{
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
				SysDelay(16);	//8ms
/*[Arima_8100][bozhi_lin] 20170320 end*/
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
				if(ReCode == ERROR_CODE_OK)
				{
					if((RegVal>>7) == 0)	break;
				}
				else
				{
					FTS_TEST_DBG("StartScan read DEVIDE_MODE_ADDR error.\n");
					break;
				}
			}
			if(times < MaxTimes)	ReCode = ERROR_CODE_OK;
			else ReCode = ERROR_CODE_COMM_ERROR;
		}
		else 
			FTS_TEST_DBG("StartScan write DEVIDE_MODE_ADDR error.\n");
	}
	else
		FTS_TEST_DBG("StartScan read DEVIDE_MODE_ADDR error.\n");
	return ReCode;

}	
/************************************************************************
* Name: ReadRawData(Same function name as FT_MultipleTest)
* Brief:  read Raw Data
* Input: Freq(No longer used, reserved), LineNum, ByteNum
* Output: pRevBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1=0;

	//unsigned short BytesNumInTestMode3=0, BytesNumInTestMode2=0,BytesNumInTestMode1=0;
	//unsigned short BytesNumInTestMode6=0, BytesNumInTestMode5=0,BytesNumInTestMode4=0;

	iReadNum=ByteNum/BYTES_PER_TIME;

	if(0 != (ByteNum%BYTES_PER_TIME)) iReadNum++;

	if(ByteNum <= BYTES_PER_TIME)
	{
		BytesNumInTestMode1 = ByteNum;		
	}
	else
	{
		BytesNumInTestMode1 = BYTES_PER_TIME;
	}

	ReCode = WriteReg(REG_LINE_NUM, LineNum);//Set row addr;

	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to write REG_LINE_NUM! \n");
		goto READ_ERR;
	}

	//***********************************************************Read raw data		
	I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
	if(ReCode == ERROR_CODE_OK)
	{
		focal_msleep(10);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, m_ucTempData, BytesNumInTestMode1);
		if(ReCode != ERROR_CODE_OK)
		{	
			FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!1 \n");
			goto READ_ERR;
		}
	}

	for(i=1; i<iReadNum; i++)
	{
		if(ReCode != ERROR_CODE_OK) break;

		if(i==iReadNum-1)//last packet
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+BYTES_PER_TIME*i, ByteNum-BYTES_PER_TIME*i);
			if(ReCode != ERROR_CODE_OK)
			{	
				FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!2 \n");
				goto READ_ERR;
			}
		}
		else
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+BYTES_PER_TIME*i, BYTES_PER_TIME);
			
			if(ReCode != ERROR_CODE_OK)
			{	
				FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!3 \n");
				goto READ_ERR;
			}
		}

	}

	if(ReCode == ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (m_ucTempData[i<<1]<<8)+m_ucTempData[(i<<1)+1];
			//if(pRevBuffer[i] & 0x8000)//有符号位
			//{
			//	pRevBuffer[i] -= 0xffff + 1;
			//}
		}
	}

READ_ERR:
	return ReCode;

}
/************************************************************************
* Name: GetTxSC_CB(Same function name as FT_MultipleTest)
* Brief:  get CB of Tx SCap
* Input: index
* Output: pcbValue
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char GetTxSC_CB(unsigned char index, unsigned char *pcbValue)
{
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char wBuffer[4];

	if(index<128)//单个读取
	{	
		*pcbValue = 0;
		WriteReg(REG_ScCbAddrR, index);
		ReCode = ReadReg(REG_ScCbBuf0, pcbValue);
	}
	else//连续读取，长度为index-128
	{
		WriteReg(REG_ScCbAddrR, 0);
		wBuffer[0] = REG_ScCbBuf0;	
		ReCode = Comm_Base_IIC_IO(wBuffer, 1, pcbValue, index-128);

	}	

	return ReCode;
}


//////////////////////////////////////////////
/************************************************************************
* Name: AllocateMemory
* Brief:  Allocate pointer Memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void AllocateMemory(void)
{
	//New buff
	g_pStoreMsgArea =NULL;	
	if(NULL == g_pStoreMsgArea)
		g_pStoreMsgArea = kmalloc(1024*80, GFP_KERNEL);
	g_pMsgAreaLine2 =NULL;	
	if(NULL == g_pMsgAreaLine2)
		g_pMsgAreaLine2 = kmalloc(1024*80, GFP_KERNEL);
	g_pStoreDataArea =NULL;	
	if(NULL == g_pStoreDataArea)
		g_pStoreDataArea = kmalloc(1024*80, GFP_KERNEL);
	/*g_pStoreAllData =NULL;	
	if(NULL == g_pStoreAllData)
		g_pStoreAllData = kmalloc(1024*8, GFP_KERNEL);*/
	g_pTmpBuff =NULL;	
	if(NULL == g_pTmpBuff)
		g_pTmpBuff = kmalloc(1024*16, GFP_KERNEL);

}
/************************************************************************
* Name: FreeMemory
* Brief:  Release pointer memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void FreeMemory(void)
{
	//Release buff
	if(NULL != g_pStoreMsgArea)
		kfree(g_pStoreMsgArea);

	if(NULL != g_pMsgAreaLine2)
		kfree(g_pMsgAreaLine2);

	if(NULL != g_pStoreDataArea)
		kfree(g_pStoreDataArea);

	/*if(NULL == g_pStoreAllData)
		kfree(g_pStoreAllData);*/

	if(NULL != g_pTmpBuff)
		kfree(g_pTmpBuff);
}

/************************************************************************
* Name: InitStoreParamOfTestData
* Brief:  Init store param of test data
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void InitStoreParamOfTestData(void)
{


	g_lenStoreMsgArea = 0;
	//Msg Area, Add Line1
	g_lenStoreMsgArea += sprintf(g_pStoreMsgArea,"ECC, 85, 170, IC Name, %s, IC Code, %x\n", g_strIcName,  g_ScreenSetParam.iSelectedIC);
	//Line2
	//g_pMsgAreaLine2 = NULL;
	g_lenMsgAreaLine2 = 0;

	//Data Area
	//g_pStoreDataArea = NULL;
	g_lenStoreDataArea = 0;
	m_iStartLine = 11;//The Start Line of Data Area is 11

	m_iTestDataCount = 0;	
}
/************************************************************************
* Name: MergeAllTestData
* Brief:  Merge All Data of test result
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void MergeAllTestData(void)
{
	int iLen = 0;

	//Add the head part of Line2
	iLen= sprintf(g_pTmpBuff,"TestItem, %d, ", m_iTestDataCount);
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea+=iLen;

	//Add other part of Line2, except for "\n"
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pMsgAreaLine2, g_lenMsgAreaLine2);
	g_lenStoreMsgArea+=g_lenMsgAreaLine2;	

	//Add Line3 ~ Line10
	iLen= sprintf(g_pTmpBuff,"\n\n\n\n\n\n\n\n\n");
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea+=iLen;

	///1.Add Msg Area
	memcpy(g_pStoreAllData, g_pStoreMsgArea, g_lenStoreMsgArea);

	///2.Add Data Area
	if(0!= g_lenStoreDataArea)
	{
		memcpy(g_pStoreAllData+g_lenStoreMsgArea, g_pStoreDataArea, g_lenStoreDataArea);
	}

	FTS_TEST_DBG("[focal] %s lenStoreMsgArea=%d,  lenStoreDataArea = %d\n", __func__, g_lenStoreMsgArea, g_lenStoreDataArea);
}


/************************************************************************
* Name: Save_Test_Data
* Brief:  Storage format of test data
* Input: int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount
* Output: none
* Return: none
***********************************************************************/
/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
#if 1
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int RowArrayIndex, int ColArrayIndex,unsigned char Row, unsigned char Col, unsigned char ItemCount)
#else
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount)
#endif
/*[Arima_8100][bozhi_lin] 20170320 end*/
{
	int iLen = 0;
	int i = 0, j = 0;

	//Save  Msg (ItemCode is enough, ItemName is not necessary, so set it to "NA".)
	iLen= sprintf(g_pTmpBuff,"NA, %d, %d, %d, %d, %d, ", \
		m_ucTestItemCode, Row, Col, m_iStartLine, ItemCount);
	memcpy(g_pMsgAreaLine2+g_lenMsgAreaLine2, g_pTmpBuff, iLen);
	g_lenMsgAreaLine2 += iLen;

	m_iStartLine += Row;
	m_iTestDataCount++;

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
	//Save Data 
	for (i = 0+RowArrayIndex; (i < Row+RowArrayIndex) && (i < TX_NUM_MAX); i++)
	{
		for (j = 0+ColArrayIndex; (j < Col+ColArrayIndex) && (j < RX_NUM_MAX); j++)
		{
			if (j == (Col+ColArrayIndex -1)) //The Last Data of the Row, add "\n"
				iLen= sprintf(g_pTmpBuff,"%d, \n", iData[i][j]);	
			else
				iLen= sprintf(g_pTmpBuff,"%d, ", iData[i][j]);	

			memcpy(g_pStoreDataArea+g_lenStoreDataArea, g_pTmpBuff, iLen);
			g_lenStoreDataArea += iLen;		
		}
	}
/*[Arima_8100][bozhi_lin] 20170320 end*/

}

/************************************************************************
* Name: GetChannelNum
* Brief:  Get Channel Num(Tx and Rx)
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; //= new unsigned char;

	//m_strCurrentTestMsg = "Get Tx Num...";
	ReCode = GetPanelRows(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iTxNum = rBuffer[0];	
		if(g_ScreenSetParam.iTxNum > g_ScreenSetParam.iUsedMaxTxNum)
		{
			FTS_TEST_DBG("Failed to get Tx number, Get num = %d, UsedMaxNum = %d\n",
				g_ScreenSetParam.iTxNum, g_ScreenSetParam.iUsedMaxTxNum);
			return ERROR_CODE_INVALID_PARAM;
		}
	}
	else
	{
		FTS_TEST_DBG("Failed to get Tx number\n");
	}

	///////////////m_strCurrentTestMsg = "Get Rx Num...";

	ReCode = GetPanelCols(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iRxNum = rBuffer[0];
		if(g_ScreenSetParam.iRxNum > g_ScreenSetParam.iUsedMaxRxNum)
		{
			FTS_TEST_DBG("Failed to get Rx number, Get num = %d, UsedMaxNum = %d\n",
				g_ScreenSetParam.iRxNum, g_ScreenSetParam.iUsedMaxRxNum);
			return ERROR_CODE_INVALID_PARAM;
		}		
	}
	else
	{
		FTS_TEST_DBG("Failed to get Rx number\n");
	}

	return ReCode;

}
/************************************************************************
* Name: GetRawData
* Brief:  Get Raw Data of MCAP
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetRawData(void)
{
	unsigned char ReCode = ERROR_CODE_OK;
	int iRow = 0;
	int iCol = 0;		

/*[Arima_8100][bozhi_lin] touch aats test LCD noise implement 20170320 begin*/
#if 0
	//--------------------------------------------Enter Factory Mode
	ReCode = EnterFactory();	
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Enter Factory Mode...\n");
		return ReCode;
	}


	//--------------------------------------------Check Num of Channel 
	if(0 == (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)) 
	{
		ReCode = GetChannelNum();
		if( ERROR_CODE_OK != ReCode ) 
		{
			FTS_TEST_DBG("Error Channel Num...\n");
			return ERROR_CODE_INVALID_PARAM;
		}
	}
#endif
/*[Arima_8100][bozhi_lin] 20170320 end*/

	//--------------------------------------------Start Scanning
	FTS_TEST_DBG("Start Scan ...\n");
	ReCode = StartScan();
	if(ERROR_CODE_OK != ReCode) 
	{
		FTS_TEST_DBG("Failed to Scan ...\n");
		return ReCode;
	}

    FTS_TEST_DBG("Succee to Scan ...\n");
	//--------------------------------------------Read RawData, Only MCAP
	memset(m_RawData, 0, sizeof(m_RawData));
    memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
	ReCode = ReadRawData( 1, 0xAA, ( g_ScreenSetParam.iTxNum * g_ScreenSetParam.iRxNum )*2, m_iTempRawData );
    if (ERROR_CODE_OK != ReCode)
    {
        FTS_TEST_DBG("Failed to ReadRawData ...\n");
        return ReCode;
    }

	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			m_RawData[iRow][iCol] = m_iTempRawData[iRow*g_ScreenSetParam.iRxNum + iCol];
		}
	}
	return ReCode;
}
/************************************************************************
* Name: ShowRawData
* Brief:  Show RawData
* Input: none
* Output: none
* Return: none.
***********************************************************************/
static void ShowRawData(void)
{
	int iRow, iCol;
	//----------------------------------------------------------Show RawData
	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		FTS_TEST_DBG("\nTx%2d:  ", iRow+1);
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			FTS_TEST_DBG("%5d    ", m_RawData[iRow][iCol]);
		}
	}
}

/************************************************************************
* Name: GetChannelNumNoMapping
* Brief:  get Tx&Rx num from other Register
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNumNoMapping(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; //= new unsigned char;


	FTS_TEST_DBG("Get Tx Num...\n");
	ReCode =ReadReg( REG_TX_NOMAPPING_NUM,  rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iTxNum= rBuffer[0];	
	}
	else
	{
		FTS_TEST_DBG("Failed to get Tx number\n");
	}


	FTS_TEST_DBG("Get Rx Num...\n");
	ReCode = ReadReg( REG_RX_NOMAPPING_NUM,  rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iRxNum = rBuffer[0];
	}
	else
	{
		FTS_TEST_DBG("Failed to get Rx number\n");
	}

	return ReCode;
}
/************************************************************************
* Name: SwitchToNoMapping
* Brief:  If it is V3 pattern, Get Tx/Rx Num again
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char SwitchToNoMapping(void)
{
	unsigned char chPattern = -1;
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char RegData = -1;
	ReCode = ReadReg( REG_PATTERN_5422, &chPattern );//
	if( ReCode != ERROR_CODE_OK )
	{
		FTS_TEST_DBG("Switch To NoMapping Failed!\n");
		goto READ_ERR;
	}
	
	if(1 == chPattern)// 1: V3 Pattern
	{	
		RegData = -1;
		ReCode =ReadReg( REG_MAPPING_SWITCH, &RegData );
		if( ReCode != ERROR_CODE_OK )
		{
			FTS_TEST_DBG("read REG_MAPPING_SWITCH Failed!\n");
			goto READ_ERR;
		}
		
		if( 1 != RegData ) 
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 1 );  //0-mapping 1-no mampping
			if( ReCode != ERROR_CODE_OK )
			{
				FTS_TEST_DBG("write REG_MAPPING_SWITCH Failed!\n");
				goto READ_ERR;
			}
			focal_msleep(20);
			ReCode = GetChannelNumNoMapping();

			if( ReCode != ERROR_CODE_OK )
			{
				FTS_TEST_DBG("GetChannelNumNoMapping Failed!\n");
				goto READ_ERR;
			}
		}
	}

READ_ERR:	
	return ReCode;
}
/************************************************************************
* Name: GetTestCondition
* Brief:  Check whether Rx or TX need to test, in Waterproof ON/OFF Mode.
* Input: none
* Output: none
* Return: true: need to test; false: Not tested.
***********************************************************************/
static boolean GetTestCondition(int iTestType, unsigned char ucChannelValue)
{
	boolean bIsNeeded = false;
	switch(iTestType)
	{
	case WT_NeedProofOnTest://Bit5:  0：检测防水模式;  1：不检测防水模式
		bIsNeeded = !( ucChannelValue & 0x20 );
		break;
	case WT_NeedProofOffTest://Bit7: 0 普通模式检测； 1：普通模式不检测
		bIsNeeded = !( ucChannelValue & 0x80 );
		break;
	case WT_NeedTxOnVal:
		//Bit6:  0 : 检测防水Rx+Tx； 1：只检测一个通道 
		//Bit2:  0: 只检测防水Tx;  1:  只检测防水Rx
		bIsNeeded = !( ucChannelValue & 0x40 ) || !( ucChannelValue & 0x04 );
		break;			
	case WT_NeedRxOnVal:
		//Bit6:  0 : 检测防水Rx+Tx； 1：只检测一个通道 
		//Bit2:  0: 只检测防水Tx;  1:  只检测防水Rx
		bIsNeeded = !( ucChannelValue & 0x40 ) || ( ucChannelValue & 0x04 );
		break;			
	case WT_NeedTxOffVal://Bit1,Bit0:  00:普通模式Tx; 10: 普通模式Rx+Tx 
		bIsNeeded = (0x00 == (ucChannelValue & 0x03)) || (0x02 == ( ucChannelValue & 0x03 ));
		break;			
	case WT_NeedRxOffVal://Bit1,Bit0:  01: 普通模式Rx;    10: 普通模式Rx+Tx
		bIsNeeded = (0x01 == (ucChannelValue & 0x03)) || (0x02 == ( ucChannelValue & 0x03 ));
		break;
	default:break;
	}
	return bIsNeeded;
}


