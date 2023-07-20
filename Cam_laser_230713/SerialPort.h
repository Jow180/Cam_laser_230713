#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <Windows.h>

/** �ǦC��q�H��
*
* ������{�F��ǦC�𪺰򥻾ާ@
* �Ҧp��ť�o����w�ǦC�𪺼ƾڡB�o�e���w�ƾڨ�ǦC��
*/

class CSerialPort
{
public:
	CSerialPort(void);
	~CSerialPort(void);

public:

	/** ��l�ƧǦC����
	*
	* @param: UINT portNo �ǦC��s��,�q�{�Ȭ�1,�YCOM1,�`�N,�ɶq���n�j��9
	* @param: UINT baud �i�S�v,�q�{��9600
	* @param: char parity �O�_�i��_������,'Y'��ܻݭn�_������,'N'��ܤ��ݭn�_������
	* @param: UINT databits �ƾڦ쪺�Ӽ�,�q�{�Ȭ�8�Ӽƾڦ�
	* @param: UINT stopsbits �����ϥή榡,�q�{�Ȭ�1
	* @param: DWORD dwCommEvents �q�{��EV_RXCHAR,�Y�u�n���o���N�@�Ӧr��,�h���ͤ@�Өƥ�
	* @return: bool ��l�ƬO�_���\
	* @note: �b�ϥΨ�L�������Ѫ���ƫe,�Х��եΥ���ƶi��ǦC�𪺪�l��
	*�@�@�@�@�@ /n����ƴ��ѤF�@�Ǳ`�Ϊ��ǦC��ѼƳ]�m,�Y�ݭn�ۦ�]�m�ԲӪ�DCB�Ѽ�,�i�ϥέ������
	* /n���ǦC�����R�c�ɷ|�۰������ǦC��,�L���B�~���������ǦC��
	* @see:
	*/
	bool InitPort(UINT portNo = 1, UINT baud = CBR_9600, char parity = 'N', UINT databits = 8, UINT stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);

	/** �ǦC���l�ƨ��
	*
	* ����ƴ��Ѫ����ھ�DCB�ѼƳ]�m�ǦC��Ѽ�
	* @param: UINT portNo
	* @param: const LPDCB & plDCB
	* @return: bool ��l�ƬO�_���\
	* @note: ����ƴ��ѥΤ�۩w�q�a�ǦC���l�ưѼ�
	* @see:
	*/
	bool InitPort(UINT portNo, const LPDCB& plDCB);

	/** �}�Һ�ť�u�{
	*
	* ����ť�u�{������ǦC��ƾڪ���ť,�ñN�����쪺�ƾڥ��L��̹���X
	* @return: bool �ާ@�O�_���\
	* @note: ��u�{�w�g�B��}�Ҫ��A��,��^flase
	* @see:
	*/
	bool OpenListenThread();

	/** ������ť�u�{
	*
	*
	* @return: bool �ާ@�O�_���\
	* @note: �եΥ���ƦZ,��ť�ǦC�𪺽u�{�N�|�Q����
	* @see:
	*/
	bool CloseListenTread();

	/** �V�ǦC��g�ƾ�
	*
	* �N�w�R�Ϥ����ƾڼg�J��ǦC��
	* @param: unsigned char * pData ���V�ݭn�g�J�ǦC�𪺼ƾڽw�R��
	* @param: unsigned int length �ݭn�g�J���ƾڪ���
	* @return: bool �ާ@�O�_���\
	* @note: length���n�j��pData�ҫ��V�w�R�Ϫ��j�p
	* @see:
	*/
	bool WriteData(char* pData, unsigned int length);

	/** ����ǦC��w�R�Ϥ����r�`��
	*
	*
	* @return: UINT �ާ@�O�_���\
	* @note: ��ǦC��w�R�Ϥ��L�ƾڮ�,��^0
	* @see:
	*/
	UINT GetBytesInCOM();

	/** Ū���ǦC�𱵦��w�R�Ϥ��@�Ӧr�`���ƾ�
	*
	*
	* @param: char & cRecved �s��Ū���ƾڪ��r���ܶq
	* @return: bool Ū���O�_���\
	* @note:
	* @see:
	*/
	bool ReadChar(char& cRecved);

private:

	/** ���}�ǦC��
	*
	*
	* @param: UINT portNo �ǦC��]�Ƹ�
	* @return: bool ���}�O�_���\
	* @note:
	* @see:
	*/
	bool openPort(UINT portNo);

	/** �����ǦC��
	*
	*
	* @return: void �ާ@�O�_���\
	* @note:
	* @see:
	*/
	void ClosePort();

	/** �ǦC���ť�u�{
	*
	* ��ť�ӦۧǦC�𪺼ƾکM�H��
	* @param: void * pParam �u�{�Ѽ�
	* @return: UINT WINAPI �u�{��^��
	* @note:
	* @see:
	*/
	static UINT WINAPI ListenThread(void* pParam);

private:

	/** �ǦC��y�` */
	HANDLE m_hComm;

	/** �u�{�h�X�Ч��ܶq */
	static bool s_bExit;

	/** �u�{�y�` */
	volatile HANDLE m_hListenThread;

	/** �P�B����,�{�ɰϫO�@ */
	CRITICAL_SECTION m_csCommunicationSync; //!< �����ާ@�ǦC��
};

#endif //SERIALPORT_H_