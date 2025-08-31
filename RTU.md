# RTU.H    (串口类)

```
#pragma once
#include <modbus.h>
#include <string>
#include <vector>

//串口类
class RTU
{
public:
    RTU();
    ~RTU();

    // 创建并连接端口
    bool openPort(const std::string& portName, int baud, char parity, int dataBit, int stopBit, int slaveId = 1);

    // 关闭端口
    void closePort();

    // 判断是否已连接
    bool isConnected() const;

    // 读保持寄存器
    int readHoldingRegisters(int addr, int num, std::vector<uint16_t>& regs);

    // 写保持寄存器
    int writeHoldingRegisters(int addr, const std::vector<uint16_t>& regs);

    // 读线圈
    int readCoils(int addr, int num, std::vector<uint8_t>& coils);

    // 写单个线圈
    int writeCoil(int addr, int status);

    //连接与否
    bool connected;        

    //创建寄存器区域
    int map_new(int a, int b, int c, int d);

    //信息循环
    void fix_con();

    int rc;

    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];

    //断开连接
    void Disconnect();

private:
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    
};


```

# RTU.CPP

```
#include "pch.h"
#include "RTU.h"
#include <errno.h>

RTU::RTU()
	: ctx(nullptr)
    , connected(false) //创建一个对象构造时，自动对其继续创建指针，便于后面进行使用。
    , mb_mapping(NULL)
{

}

RTU::~RTU()
{
	closePort();
}

//进行连接，和设置从站，后续将添加设置从站ID的功能
bool RTU::openPort(const std::string& portName, int baud, char parity, int dataBit, int stopBit, int slaveId)
{
    closePort(); // 保证先关闭

    ctx = modbus_new_rtu(portName.c_str(), baud, parity, dataBit, stopBit);
    //modbus_t* ctx = modbus_new_ascii("COM2", 9600, 'N', 8, 1);    使用ASCII模式，后期多传入一个BOOL类型进行选择制作即可
    if (!ctx)
        return false;

    modbus_set_slave(ctx, slaveId);

    if (modbus_connect(ctx) == -1) {
        modbus_free(ctx);
        ctx = nullptr;
        connected = false;
        return false;
    }

    connected = true;
    return true;
}

void RTU::closePort()
{
    if (ctx) {
        modbus_close(ctx);
        modbus_free(ctx);
        ctx = nullptr;
    }
    connected = false;
}

bool RTU::isConnected() const
{
    return connected && ctx;
}

int RTU::readHoldingRegisters(int addr, int num, std::vector<uint16_t>& regs)
{
    regs.resize(num);
    if (!isConnected())
        return -1;
    return modbus_read_registers(ctx, addr, num, regs.data());
}

int RTU::writeHoldingRegisters(int addr, const std::vector<uint16_t>& regs)
{
    if (!isConnected())
        return -1;
    return modbus_write_registers(ctx, addr, regs.size(), regs.data());
}

int RTU::readCoils(int addr, int num, std::vector<uint8_t>& coils)
{
    coils.resize(num);
    if (!isConnected())
        return -1;
    return modbus_read_bits(ctx, addr, num, coils.data());
}

int RTU::writeCoil(int addr, int status)
{
    if (!isConnected())
        return -1;
    return modbus_write_bit(ctx, addr, status);
}
//分配寄存器内存
int RTU::map_new(int a, int b, int c, int d)
{
    mb_mapping = modbus_mapping_new(500, 500, 500, 500);
    if (mb_mapping == NULL) {
       // fprintf(stderr, "Failed to allocate mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    return 0;
}


void RTU::fix_con()
{
    // 主循环
    while(connect)
    {
        // 清空查询缓冲区
        memset(query, 0, MODBUS_RTU_MAX_ADU_LENGTH);

        rc = modbus_receive(ctx, query);
        if (rc > 0) {
            // 处理请求,显示在某个区域即可
            
            modbus_reply(ctx, query, rc, mb_mapping);
        }
        else if (rc == -1) {
            // 连接关闭或出错
            break;
        }
    }

}

void RTU::Disconnect()
{
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
}


```

# rtu_从站Dlg.h（主对话框类）

```

// rtu_从站Dlg.h: 头文件
//

#pragma once
#include "RTU.h"


// Crtu从站Dlg 对话框
class Crtu从站Dlg : public CDialogEx
{
// 构造
public:
	Crtu从站Dlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_RTU__DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1Rtu();
	afx_msg void OnBnClickedButton1();
	RTU Port1;

	
};

```

# rtu_从站.cpp

```

// rtu_从站Dlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "rtu_从站.h"
#include "rtu_从站Dlg.h"
#include "afxdialogex.h"
#include "modbus.h"      //包含modbud库
#include "Port_Con.h"
//#include "RTU.h"
#include "string.h"
#include <atlconv.h>
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// Crtu从站Dlg 对话框



Crtu从站Dlg::Crtu从站Dlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_RTU__DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void Crtu从站Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(Crtu从站Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1_RTU, &Crtu从站Dlg::OnBnClickedButton1Rtu)
	ON_BN_CLICKED(IDC_BUTTON1, &Crtu从站Dlg::OnBnClickedButton1)
END_MESSAGE_MAP()


// Crtu从站Dlg 消息处理程序

BOOL Crtu从站Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void Crtu从站Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void Crtu从站Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR Crtu从站Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void Crtu从站Dlg::OnBnClickedButton1Rtu()
{
	// 创建配置对话框实例
	Port_Con dlg;

	// 显示模态对话框
	if (dlg.DoModal() == IDOK) {
		// 获取配置值
		CString strPort = dlg.m_strPort;
		// CString → std::string（推荐用 CT2A）
		CT2A asciiStr(strPort);
		string portName(asciiStr);
		int nBaudRate = dlg.m_nBaudRate;
		int nDataBits = dlg.m_nDataBits;
		int nParity = dlg.m_nParity;
		char parityChar;
		switch (nParity) {
		case 0: parityChar = 'N'; break;
		case 1: parityChar = 'O'; break;
		case 2: parityChar = 'E'; break;
		default: parityChar = 'N'; // 默认无校验
		}
		int nStopBits = dlg.m_nStopBits;

		// 显示配置信息到Edit Box
		CString strMsg;
		strMsg.Format(_T("串口配置:\r\n端口: %s\r\n波特率: %d\r\n数据位: %d\r\n校验位: %d\r\n停止位: %d"),
			strPort, nBaudRate, nDataBits, nParity, nStopBits);

		// 获取Edit Box控件
		CEdit* pEdit = (CEdit*)GetDlgItem(IDC_EDIT4);
		if (pEdit != nullptr)
		{
			pEdit->SetWindowText(strMsg);  // 设置文本内容
			// 或者如果你想追加文本而不是替换：
			// pEdit->SetSel(-1, -1);  // 移动到文本末尾
			// pEdit->ReplaceSel(strMsg);  // 追加文本
		}

		// 这里可以调用您的串口连接函数，对其进行已经全局声明
		Port1.openPort(portName, nBaudRate, parityChar, nDataBits, nStopBits);
		if (Port1.connected != true)
		{
			MessageBox(_T("fail"), _T("提示框"));
		}

		//分配寄存器内存
		int nRet = Port1.map_new(500, 500, 500, 500);
		if (nRet == -1)
		{
			MessageBox(_T("allocate fail"), _T(" 提示框"));
		}

		Port1.fix_con();

	}
	
}

//断开连接
void Crtu从站Dlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码,需要将创建的指针对象设置为全局
	Port1.Disconnect();
}

```

# Port_Con.h(port 页面类)



```
#pragma once
#include "afxdialogex.h"


// Port_Con 对话框，即新页面的类，配置文件类

class Port_Con : public CDialogEx
{
	DECLARE_DYNAMIC(Port_Con)

public:
	Port_Con(CWnd* pParent = nullptr);   // 标准构造函数
	
	// 添加公有成员变量来存储配置
	CString  m_strPort;
	int m_nBaudRate;
	int m_nDataBits;
	int m_nParity;
	int m_nStopBits;

	virtual ~Port_Con();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG1 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnCbnSelchangeCombo3();
	// 添加 OnInitDialog 声明
	virtual BOOL  OnInitDialog();
	CComboBox m_cboCombo1;                //Serial Setting
	CComboBox m_cboCombo2;                //Baud
	CComboBox m_cboCombo3;                //Data bits
	CComboBox m_cboCombo4;               //Parity
	CComboBox m_cboCombo5;               //Stop bits
	afx_msg void OnBnClickedButton1();  //OK键
};

```

# Port_Con.cpp

```
// Port_Con.cpp: 实现文件
//

#include "pch.h"
#include "rtu_从站.h"
#include "afxdialogex.h"
#include "Port_Con.h"


// Port_Con 对话框

IMPLEMENT_DYNAMIC(Port_Con, CDialogEx)

Port_Con::Port_Con(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DIALOG1, pParent)
{

}

Port_Con::~Port_Con()
{
}

void Port_Con::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_cboCombo1);
	DDX_Control(pDX, IDC_COMBO2, m_cboCombo2);
	DDX_Control(pDX, IDC_COMBO3, m_cboCombo3);
	DDX_Control(pDX, IDC_COMBO4, m_cboCombo4);
	DDX_Control(pDX, IDC_COMBO5, m_cboCombo5);
}


BEGIN_MESSAGE_MAP(Port_Con, CDialogEx)
	ON_CBN_SELCHANGE(IDC_COMBO3, &Port_Con::OnCbnSelchangeCombo3)
	ON_BN_CLICKED(IDC_BUTTON1, &Port_Con::OnBnClickedButton1)
END_MESSAGE_MAP()


// Port_Con 消息处理程序

void Port_Con::OnCbnSelchangeCombo3()
{
	// TODO: 在此添加控件通知处理程序代码
}


// Port_Con.cpp
BOOL Port_Con::OnInitDialog()
{
	CDialogEx::OnInitDialog(); // 必须首先调用基类

	// TODO: 在此添加额外的初始化代码

	// 向 IDC_COMBO1 添加内容
	m_cboCombo1.AddString(_T("COM1"));
	m_cboCombo1.AddString(_T("COM2"));
	m_cboCombo1.AddString(_T("COM3"));
	m_cboCombo1.AddString(_T("COM4"));
	m_cboCombo1.AddString(_T("COM5"));
	m_cboCombo1.AddString(_T("COM6"));
	m_cboCombo1.AddString(_T("COM7"));
	m_cboCombo1.AddString(_T("COM8"));

	// 设置默认选中项（比如默认选中 COM1）
	m_cboCombo1.SetCurSel(1);

	m_cboCombo2.AddString(_T("300"));
	m_cboCombo2.AddString(_T("600"));
	m_cboCombo2.AddString(_T("1200"));
	m_cboCombo2.AddString(_T("2400"));
	m_cboCombo2.AddString(_T("4800"));
	m_cboCombo2.AddString(_T("9600"));
	m_cboCombo2.AddString(_T("14400"));
	m_cboCombo2.SetCurSel(5);

	m_cboCombo3.AddString(_T("7 Data bits"));
	m_cboCombo3.AddString(_T("8 Data bits"));
	m_cboCombo3.SetCurSel(1);

	m_cboCombo4.AddString(_T("None Parity"));
	m_cboCombo4.AddString(_T("Odd Parity"));
	m_cboCombo4.AddString(_T("Even Parity"));
	m_cboCombo4.SetCurSel(2);

	m_cboCombo5.AddString(_T("1 Stop Bit"));
	m_cboCombo5.AddString(_T("2 Stop Bit"));
	m_cboCombo5.SetCurSel(0);

	return TRUE; // 除非将焦点设置到控件，否则返回 TRUE
}

//OK对话框，将值取出，传递，创建连接
void Port_Con::OnBnClickedButton1()
{
	// 获取串口号
	int nSel = m_cboCombo1.GetCurSel();
	if (nSel != CB_ERR) {
		m_cboCombo1.GetLBText(nSel, m_strPort);
	}
	// 获取波特率并转换为整数
	nSel = m_cboCombo2.GetCurSel();
	if (nSel != CB_ERR) {
		CString strBaud;
		m_cboCombo2.GetLBText(nSel, strBaud);
		m_nBaudRate = _ttoi(strBaud);
	}
	// 获取数据位
	nSel = m_cboCombo3.GetCurSel();
	if (nSel != CB_ERR) {
		m_nDataBits = (nSel == 0) ? 7 : 8; // 根据索引判断
	}

	// 获取校验位
	nSel = m_cboCombo4.GetCurSel();
	if (nSel != CB_ERR) {
		m_nParity = nSel; // 0=None, 1=Odd, 2=Even
	}

	// 获取停止位
	nSel = m_cboCombo5.GetCurSel();
	if (nSel != CB_ERR) {
		m_nStopBits = (nSel == 0) ? 1 : 2;
	}

	// 关闭对话框（返回 IDOK）
	CDialogEx::OnOK();
}

```

TEST

```
#include <modbus.h>
#include <iostream>
#include <errno.h>

using namespace std;

int main()
{
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
    int rc;

    // 创建RTU从站上下文
    // 参数依次为串口号、波特率、校验、数据位、停止位
   // ctx = modbus_new_rtu("/dev/ttyS0", 9600, 'N', 8, 1); // Windows下可用"COM2"
    ctx = modbus_new_rtu("COM2", 9600, 'N', 8, 1); // Windows下可用"COM2"
    if (ctx == NULL) {
        printf("Unable to create libmodbus context\n");
        return -1;
    }

    // 设置从站ID
    modbus_set_slave(ctx, 1);

    // 打开串口
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // 创建映射区域（线圈、输入、寄存器等）
    mb_mapping = modbus_mapping_new(500, 500, 500, 500);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // 主循环
    for (;;) {
        rc = modbus_receive(ctx, query);
        if (rc > 0) {
            // 处理请求
            modbus_reply(ctx, query, rc, mb_mapping);
        }
        else if (rc == -1) {
            // 连接关闭或出错
            break;
        }
    }

    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
```

```
#include <modbus.h>
#include <iostream>
#include <cstring>
#include <iomanip>
using namespace std;

#define NUM_HOLDING_REGISTERS 500
uint16_t holding_registers_bank[NUM_HOLDING_REGISTERS] = { 0 };

// 打印十六进制数据
void print_hex_data(const char* prefix, const uint8_t* data, int length) {
    cout << prefix;
    for (int i = 0; i < length; i++) {
        cout << setw(2) << setfill('0') << hex << uppercase << (int)data[i] << " ";
    }
    cout << dec << endl;
}

// 获取Modbus TCP事务ID
uint16_t get_transaction_id(const uint8_t* request) {
    return (request[0] << 8) | request[1];
}

// 构建正确的TCP响应头
void build_tcp_response_header(const uint8_t* request, uint8_t* response, int data_length) {
    // 复制事务ID、协议ID、长度
    memcpy(response, request, 6);
    // 更新长度字段 (单元标识符+数据长度)
    response[4] = 0x00;
    response[5] = data_length + 1; // +1 for unit identifier
}

int main() {
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    int rc;

    // 创建TCP从站上下文
    ctx = modbus_new_tcp("127.0.0.1", 502);
    if (ctx == NULL) {
        printf("Unable to create libmodbus context\n");
        return -1;
    }

    // 设置调试模式来查看原始报文
    modbus_set_debug(ctx, TRUE);

    // 创建映射区域
    mb_mapping = modbus_mapping_new(500, 500, NUM_HOLDING_REGISTERS, 500);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // 初始化映射数据
    memcpy(mb_mapping->tab_registers, holding_registers_bank,
        NUM_HOLDING_REGISTERS * sizeof(uint16_t));

    // 设置从站ID
    modbus_set_slave(ctx, 1);

    // 监听连接
    int server_socket = modbus_tcp_listen(ctx, 1);
    if (server_socket == -1) {
        fprintf(stderr, "Listen failed: %s\n", modbus_strerror(errno));
        modbus_mapping_free(mb_mapping);
        modbus_free(ctx);
        return -1;
    }

    cout << "Modbus Slave (ID: 1) listening on 127.0.0.1:502..." << endl;

    int client_socket = modbus_tcp_accept(ctx, &server_socket);
    if (client_socket == -1) {
        fprintf(stderr, "Accept failed: %s\n", modbus_strerror(errno));
        modbus_mapping_free(mb_mapping);
        modbus_free(ctx);
        return -1;
    }

    cout << "Client connected." << endl;

    // 事务ID计数器
    uint16_t transaction_id = 0x000E;

    while (1) {
        rc = modbus_receive(ctx, query);
        if (rc > 0) {
            // 打印接收到的请求报文 (Tx)
            cout << "Tx:" << setw(3) << setfill('0') << transaction_id << "-";
            print_hex_data("", query, rc);

            // 同步数据到映射区域
            memcpy(mb_mapping->tab_registers, holding_registers_bank,
                NUM_HOLDING_REGISTERS * sizeof(uint16_t));

            // 处理请求
            int reply_rc = modbus_reply(ctx, query, rc, mb_mapping);

            if (reply_rc == -1) {
                fprintf(stderr, "Reply error: %s\n", modbus_strerror(errno));
                break;
            }

            // 同步数据回全局数组
            memcpy(holding_registers_bank, mb_mapping->tab_registers,
                NUM_HOLDING_REGISTERS * sizeof(uint16_t));

            // 对于写操作（06功能码），响应应该和请求相同
            // 对于读操作（03功能码），libmodbus会自动构建包含数据的响应
            // 但由于libmodbus直接发送了响应，我们需要模拟显示正确的响应

            uint8_t function_code = query[7]; // TCP模式下功能码在第7字节

            if (function_code == 0x06) {
                // 写单个寄存器 - 响应与请求相同
                cout << "Rx:" << setw(3) << setfill('0') << transaction_id << "-";
                print_hex_data("", query, rc);
            }
            else if (function_code == 0x03) {
                // 读保持寄存器 - 构建正确的响应
                uint16_t start_addr = (query[8] << 8) | query[9];
                uint16_t num_regs = (query[10] << 8) | query[11];

                uint8_t response[MODBUS_TCP_MAX_ADU_LENGTH];
                build_tcp_response_header(query, response, 2 + num_regs * 2);

                // 单元标识符
                response[6] = query[6];
                // 功能码
                response[7] = 0x03;
                // 字节计数
                response[8] = num_regs * 2;

                // 复制寄存器数据
                for (int i = 0; i < num_regs; i++) {
                    response[9 + i * 2] = (holding_registers_bank[start_addr + i] >> 8) & 0xFF;
                    response[10 + i * 2] = holding_registers_bank[start_addr + i] & 0xFF;
                }

                cout << "Rx:" << setw(3) << setfill('0') << transaction_id << "-";
                print_hex_data("", response, 9 + num_regs * 2);
            }

            transaction_id++;

        }
        else if (rc == -1) {
            fprintf(stderr, "Connection error: %s\n", modbus_strerror(errno));
            break;
        }
    }

    cout << "Client disconnected." << endl;

    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}

//<00><08><00><00><00><06><01><06><00><00><00><04>
//Tx:022 - 00 08 00 00 00 06 01 06 00 00 00 04
//[00][08][00][00][00][06][01][06][00][00][00][04]
//Rx:022 - 00 08 00 00 00 06 01 06 00 00 00 04
//Waiting for an indication...
//<00><09><00><00><00><06><01><03><00><00><00><0A>
//Tx:023 - 00 09 00 00 00 06 01 03 00 00 00 0A
//[00][09][00][00][00][17][01][03][14][00][04][00][00][00][00][00][00][00][00][00][00][00][00][00][00][00][00][00][00]
//Rx:023 - 00 09 00 00 00 17 01 03 14 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//Waiting for an indication...
//实现接收到在钱两位接收到01时，实现答应拍照完成。
```

