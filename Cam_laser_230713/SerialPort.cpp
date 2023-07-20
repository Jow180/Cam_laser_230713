//#include "StdAfx.h"
#include "SerialPort.h"
#include <process.h>
#include <iostream>
#include <vector>

using namespace std;
/** �u�{�h�X�Ч� */
bool CSerialPort::s_bExit = false;
/** ��ǦC��L�ƾڮ�,sleep�ܤU���d�߶��j���ɶ�,���:�� */
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
	/** �{���ܶq,�N��w�Ѽ���Ƭ��r�Ŧ�Φ�,�H�c�yDCB���c */
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** ���}���w�ǦC��,�Ө�Ƥ����w�g���{�ɰϫO�@,�W���Ф��n�[�O�@ */
	if (!openPort(portNo))
	{
		return false;
	}

	/** �i�J�{�ɬq */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �O�_�����~�o�� */
	BOOL bIsSuccess = TRUE;

	/** �b���i�H�]�m��J��X���w�R�Ϥj�p,�p�G���]�m,�h�t�η|�]�m�q�{��.
	*  �ۤv�]�m�w�R�Ϥj�p��,�n�`�N�]�m�y�j�@��,�קK�w�R�Ϸ��X
	*/
	/*if (bIsSuccess )
	{
	bIsSuccess = SetupComm(m_hComm,10,10);
	}*/

	/** �]�m�ǦC�𪺶W�ɮɶ�,���]��0,��ܤ��ϥζW�ɭ��� */
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
		// �NANSI�r�Ŧ��ഫ��UNICODE�r�Ŧ�
		DWORD dwNum = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t* pwText = new wchar_t[dwNum];
		if (!MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		/** �����e�ǦC��t�m�Ѽ�,�åB�c�y�ǦC��DCB�Ѽ� */
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb);
		/** �}��RTS flow���� */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

		/** ���񤺦s�Ŷ� */
		delete[] pwText;
	}

	if (bIsSuccess)
	{
		/** �ϥ�DCB�Ѽưt�m�ǦC�𪬺A */
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}

	/**  �M�ŧǦC��w�R�� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** ���}�{�ɬq */
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess == TRUE;
}

bool CSerialPort::InitPort(UINT portNo, const LPDCB& plDCB)
{
	/** ���}���w�ǦC��,�Ө�Ƥ����w�g���{�ɰϫO�@,�W���Ф��n�[�O�@ */
	if (!openPort(portNo))
	{
		return false;
	}

	/** �i�J�{�ɬq */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �t�m�ǦC��Ѽ� */
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}

	/**  �M�ŧǦC��w�R�� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** ���}�{�ɬq */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

void CSerialPort::ClosePort()
{
	/** �p�G���ǦC��Q���}�A������ */
	if (m_hComm != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::openPort(UINT portNo)
{
	/** �i�J�{�ɬq */
	EnterCriticalSection(&m_csCommunicationSync);

	/** ��ǦC�𪺽s���ഫ���]�ƦW */
	char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	/** ���}���w���ǦC�� */
	m_hComm = CreateFileA(szPort,  /** �]�ƦW,COM1,COM2�� */
		GENERIC_READ | GENERIC_WRITE, /** �X�ݼҦ�,�i�P��Ū�g */
		0,                            /** �@�ɼҦ�,0��ܤ��@�� */
		NULL,                         /** �w���ʳ]�m,�@��ϥ�NULL */
		OPEN_EXISTING,                /** �ӰѼƪ�ܳ]�ƥ����s�b,�_�h�Ыإ��� */
		0,
		0);

	/** �p�G���}���ѡA����귽�ê�^ */
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** �h�X�{�ɰ� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** �˴��u�{�O�_�w�g�}�ҤF */
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** �u�{�w�g�}�� */
		return false;
	}

	s_bExit = false;
	/** �u�{ID */
	UINT threadId;
	/** �}�ҧǦC��ƾں�ť�u�{ */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** �]�m�u�{���u����,���󴶳q�u�{ */
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
		/** �q���u�{�h�X */
		s_bExit = true;

		/** ���ݽu�{�h�X */
		Sleep(10);

		/** �m�u�{�y�`�L�� */
		CloseHandle(m_hListenThread);
		m_hListenThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT CSerialPort::GetBytesInCOM()
{
	DWORD dwError = 0;  /** ���~�X */
	COMSTAT  comstat;   /** COMSTAT���c��,�O���q�H�]�ƪ����A�H�� */
	memset(&comstat, 0, sizeof(COMSTAT));

	UINT BytesInQue = 0;
	/** �b�ե�ReadFile�MWriteFile���e,�q�L����ƲM���H�e��d�����~�Ч� */
	if (ClearCommError(m_hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; /** ����b��J�w�R�Ϥ����r�`�� */
	}

	return BytesInQue;
}

UINT WINAPI CSerialPort::ListenThread(void* pParam)
{
	/** �o�쥻�������w */
	CSerialPort* pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// �u�{�`��,���ߤ覡Ū���ǦC��ƾ�
	while (!pSerialPort->s_bExit)
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** �p�G�ǦC���J�w�R�Ϥ��L�ƾ�,�h�𮧤@�|�A�d�� */
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		/** Ū����J�w�R�Ϥ����ƾڨÿ�X��� */
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

	/** �{�ɰϫO�@ */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �q�w�R��Ū���@�Ӧr�`���ƾ� */
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	if ((!bResult))
	{
		/** ������~�X,�i�H�ھڸӿ��~�X�d�X���~��] */
		DWORD dwError = GetLastError();

		/** �M�ŧǦC��w�R�� */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** ���}�{�ɰ� */
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

	/** �{�ɰϫO�@ */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �V�w�R�ϼg�J���w�q���ƾ� */
	bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);
	if (!bResult)
	{
		DWORD dwError = GetLastError();
		/** �M�ŧǦC��w�R�� */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** ���}�{�ɰ� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}