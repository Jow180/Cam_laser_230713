#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <Windows.h>

/** 序列埠通信類
*
* 本類實現了對序列埠的基本操作
* 例如監聽發到指定序列埠的數據、發送指定數據到序列埠
*/

class CSerialPort
{
public:
	CSerialPort(void);
	~CSerialPort(void);

public:

	/** 初始化序列埠函數
	*
	* @param: UINT portNo 序列埠編號,默認值為1,即COM1,注意,盡量不要大於9
	* @param: UINT baud 波特率,默認為9600
	* @param: char parity 是否進行奇偶校驗,'Y'表示需要奇偶校驗,'N'表示不需要奇偶校驗
	* @param: UINT databits 數據位的個數,默認值為8個數據位
	* @param: UINT stopsbits 停止位使用格式,默認值為1
	* @param: DWORD dwCommEvents 默認為EV_RXCHAR,即只要收發任意一個字符,則產生一個事件
	* @return: bool 初始化是否成功
	* @note: 在使用其他本類提供的函數前,請先調用本函數進行序列埠的初始化
	*　　　　　 /n本函數提供了一些常用的序列埠參數設置,若需要自行設置詳細的DCB參數,可使用重載函數
	* /n本序列埠類析構時會自動關閉序列埠,無需額外執行關閉序列埠
	* @see:
	*/
	bool InitPort(UINT portNo = 1, UINT baud = CBR_9600, char parity = 'N', UINT databits = 8, UINT stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);

	/** 序列埠初始化函數
	*
	* 本函數提供直接根據DCB參數設置序列埠參數
	* @param: UINT portNo
	* @param: const LPDCB & plDCB
	* @return: bool 初始化是否成功
	* @note: 本函數提供用戶自定義地序列埠初始化參數
	* @see:
	*/
	bool InitPort(UINT portNo, const LPDCB& plDCB);

	/** 開啟監聽線程
	*
	* 本監聽線程完成對序列埠數據的監聽,並將接收到的數據打印到屏幕輸出
	* @return: bool 操作是否成功
	* @note: 當線程已經處於開啟狀態時,返回flase
	* @see:
	*/
	bool OpenListenThread();

	/** 關閉監聽線程
	*
	*
	* @return: bool 操作是否成功
	* @note: 調用本函數后,監聽序列埠的線程將會被關閉
	* @see:
	*/
	bool CloseListenTread();

	/** 向序列埠寫數據
	*
	* 將緩沖區中的數據寫入到序列埠
	* @param: unsigned char * pData 指向需要寫入序列埠的數據緩沖區
	* @param: unsigned int length 需要寫入的數據長度
	* @return: bool 操作是否成功
	* @note: length不要大於pData所指向緩沖區的大小
	* @see:
	*/
	bool WriteData(char* pData, unsigned int length);

	/** 獲取序列埠緩沖區中的字節數
	*
	*
	* @return: UINT 操作是否成功
	* @note: 當序列埠緩沖區中無數據時,返回0
	* @see:
	*/
	UINT GetBytesInCOM();

	/** 讀取序列埠接收緩沖區中一個字節的數據
	*
	*
	* @param: char & cRecved 存放讀取數據的字符變量
	* @return: bool 讀取是否成功
	* @note:
	* @see:
	*/
	bool ReadChar(char& cRecved);

private:

	/** 打開序列埠
	*
	*
	* @param: UINT portNo 序列埠設備號
	* @return: bool 打開是否成功
	* @note:
	* @see:
	*/
	bool openPort(UINT portNo);

	/** 關閉序列埠
	*
	*
	* @return: void 操作是否成功
	* @note:
	* @see:
	*/
	void ClosePort();

	/** 序列埠監聽線程
	*
	* 監聽來自序列埠的數據和信息
	* @param: void * pParam 線程參數
	* @return: UINT WINAPI 線程返回值
	* @note:
	* @see:
	*/
	static UINT WINAPI ListenThread(void* pParam);

private:

	/** 序列埠句柄 */
	HANDLE m_hComm;

	/** 線程退出標志變量 */
	static bool s_bExit;

	/** 線程句柄 */
	volatile HANDLE m_hListenThread;

	/** 同步互斥,臨界區保護 */
	CRITICAL_SECTION m_csCommunicationSync; //!< 互斥操作序列埠
};

#endif //SERIALPORT_H_