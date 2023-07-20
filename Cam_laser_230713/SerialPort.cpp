//#include "StdAfx.h"
#include "SerialPort.h"
#include <process.h>
#include <iostream>
#include <vector>

using namespace std;
/** 線程退出標志 */
bool CSerialPort::s_bExit = false;
/** 當序列埠無數據時,sleep至下次查詢間隔的時間,單位:秒 */
const UINT SLEEP_TIME_INTERVAL = 5;

CSerialPort::CSerialPort(void)
	: m_hListenThread(INVALID_HANDLE_VALUE)
{
	m_hComm = INVALID_HANDLE_VALUE;
	m_hListenThread = INVALID_HANDLE_VALUE;

	InitializeCriticalSection(&m_csCommunicationSync);
}

CSerialPort::~CSerialPort(void)
{
	CloseListenTread();
	ClosePort();
	DeleteCriticalSection(&m_csCommunicationSync);
}

bool CSerialPort::InitPort(UINT portNo /*= 1*/, UINT baud /*= CBR_115200*/, char parity /*= 'N'*/,
	UINT databits /*= 8*/, UINT stopsbits /*= 1*/, DWORD dwCommEvents /*= EV_RXCHAR*/)
{
	/** 臨時變量,將制定參數轉化為字符串形式,以構造DCB結構 */
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** 打開指定序列埠,該函數內部已經有臨界區保護,上面請不要加保護 */
	if (!openPort(portNo))
	{
		return false;
	}

	/** 進入臨界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 是否有錯誤發生 */
	BOOL bIsSuccess = TRUE;

	/** 在此可以設置輸入輸出的緩沖區大小,如果不設置,則系統會設置默認值.
	*  自己設置緩沖區大小時,要注意設置稍大一些,避免緩沖區溢出
	*/
	/*if (bIsSuccess )
	{
	bIsSuccess = SetupComm(m_hComm,10,10);
	}*/

	/** 設置序列埠的超時時間,均設為0,表示不使用超時限制 */
	COMMTIMEOUTS  CommTimeouts;
	CommTimeouts.ReadIntervalTimeout = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 0;
	if (bIsSuccess)
	{
		bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);
	}

	DCB  dcb;
	if (bIsSuccess)
	{
		// 將ANSI字符串轉換為UNICODE字符串
		DWORD dwNum = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t* pwText = new wchar_t[dwNum];
		if (!MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		/** 獲取當前序列埠配置參數,並且構造序列埠DCB參數 */
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb);
		/** 開啟RTS flow控制 */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

		/** 釋放內存空間 */
		delete[] pwText;
	}

	if (bIsSuccess)
	{
		/** 使用DCB參數配置序列埠狀態 */
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}

	/**  清空序列埠緩沖區 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 離開臨界段 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess == TRUE;
}

bool CSerialPort::InitPort(UINT portNo, const LPDCB& plDCB)
{
	/** 打開指定序列埠,該函數內部已經有臨界區保護,上面請不要加保護 */
	if (!openPort(portNo))
	{
		return false;
	}

	/** 進入臨界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 配置序列埠參數 */
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}

	/**  清空序列埠緩沖區 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 離開臨界段 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

void CSerialPort::ClosePort()
{
	/** 如果有序列埠被打開，關閉它 */
	if (m_hComm != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::openPort(UINT portNo)
{
	/** 進入臨界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 把序列埠的編號轉換為設備名 */
	char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	/** 打開指定的序列埠 */
	m_hComm = CreateFileA(szPort,  /** 設備名,COM1,COM2等 */
		GENERIC_READ | GENERIC_WRITE, /** 訪問模式,可同時讀寫 */
		0,                            /** 共享模式,0表示不共享 */
		NULL,                         /** 安全性設置,一般使用NULL */
		OPEN_EXISTING,                /** 該參數表示設備必須存在,否則創建失敗 */
		0,
		0);

	/** 如果打開失敗，釋放資源並返回 */
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** 退出臨界區 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** 檢測線程是否已經開啟了 */
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 線程已經開啟 */
		return false;
	}

	s_bExit = false;
	/** 線程ID */
	UINT threadId;
	/** 開啟序列埠數據監聽線程 */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** 設置線程的優先級,高於普通線程 */
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}

bool CSerialPort::CloseListenTread()
{
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 通知線程退出 */
		s_bExit = true;

		/** 等待線程退出 */
		Sleep(10);

		/** 置線程句柄無效 */
		CloseHandle(m_hListenThread);
		m_hListenThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT CSerialPort::GetBytesInCOM()
{
	DWORD dwError = 0;  /** 錯誤碼 */
	COMSTAT  comstat;   /** COMSTAT結構體,記錄通信設備的狀態信息 */
	memset(&comstat, 0, sizeof(COMSTAT));

	UINT BytesInQue = 0;
	/** 在調用ReadFile和WriteFile之前,通過本函數清除以前遺留的錯誤標志 */
	if (ClearCommError(m_hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; /** 獲取在輸入緩沖區中的字節數 */
	}

	return BytesInQue;
}

UINT WINAPI CSerialPort::ListenThread(void* pParam)
{
	/** 得到本類的指針 */
	CSerialPort* pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// 線程循環,輪詢方式讀取序列埠數據
	while (!pSerialPort->s_bExit)
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** 如果序列埠輸入緩沖區中無數據,則休息一會再查詢 */
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		/** 讀取輸入緩沖區中的數據並輸出顯示 */
		char rxByteArray = 0x00;
		do
		{
			rxByteArray = 0x00;
			if (pSerialPort->ReadChar(rxByteArray) == true)
			{
				std::cout << rxByteArray;
			}
		} while (--BytesInQue);
	}
	return 0;
}

bool CSerialPort::ReadChar(char& cRecved)
{
	BOOL  bResult = TRUE;
	DWORD BytesRead = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 臨界區保護 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 從緩沖區讀取一個字節的數據 */
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	if ((!bResult))
	{
		/** 獲取錯誤碼,可以根據該錯誤碼查出錯誤原因 */
		DWORD dwError = GetLastError();

		/** 清空序列埠緩沖區 */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** 離開臨界區 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return (BytesRead == 1);
}

bool CSerialPort::WriteData(char* pData, unsigned int length)
{
	BOOL   bResult = TRUE;
	DWORD  BytesToSend = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 臨界區保護 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 向緩沖區寫入指定量的數據 */
	bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);
	if (!bResult)
	{
		DWORD dwError = GetLastError();
		/** 清空序列埠緩沖區 */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** 離開臨界區 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}