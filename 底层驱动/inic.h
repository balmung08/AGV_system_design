#ifndef INI2_INIC_H
#define INI2_INIC_H
#include <string>
#include <map>
enum INI_RES
{
    INI_SUCCESS,        // 成功
    INI_ERROR,          // 普通错误
    INI_OPENFILE_ERROR, // 打开文件失败
    INI_NO_ATTR         // 无对应的键值
};

using namespace std;
#define CONFIGLEN 256
typedef map<std::string, std::string> KEYMAP;
typedef map<std::string, KEYMAP> MAINKEYMAP;
class CIniFile
{
public:
    CIniFile(void);
    CIniFile(const char *chFileName);
    CIniFile(const string &strFileName);
    ~CIniFile(void);

    int Init();
    int Init(const char *chFileName);
    int Init(const string &strFileName);
    int Dump();

    int ReadItem(const string &strSection, const string &strKey, const string &strDefault, string &strValue);
    int WriteItem(const string &strSection, const string &strKey, const string &strValue);

private:
    int LoadFile();
    int WriteFile();

    int ReadLine(FILE *pFile, string &strLine);
    int TrimString(string &strToken);

private:
    static const int BUFFER_LEN = 1024;
    string m_strFileName;
    typedef map<string, string> ConfigType;
    map<string, ConfigType> m_mSec2Config;

protected:
    FILE *m_fp;
    char m_szKey[CONFIGLEN];
    MAINKEYMAP m_Map;
};

#ifndef UINT
typedef unsigned int UINT;
#endif
#ifndef LPCSTR
typedef const char *LPCSTR;
#endif
#ifndef LPSTR
typedef char *LPSTR;
#endif
#ifndef INT
typedef int INT;
#endif
#ifndef DWORD
typedef unsigned long DWORD;
#endif
#ifndef BOOL
typedef int BOOL;
#endif
#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

UINT GetPrivateProfileInt(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    INT nDefault,
    LPCSTR lpFileName);
BOOL WritePrivateProfileString(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    LPCSTR lpString,
    LPCSTR lpFileName);
DWORD
GetPrivateProfileString(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    LPCSTR lpDefault,
    LPSTR lpReturnedString,
    DWORD nSize,
    LPCSTR lpFileName);

DWORD
GetPrivateProfileSection(
    LPCSTR lpAppName,
    LPSTR lpReturnedString,
    DWORD nSize,
    LPCSTR lpFileName);

DWORD
GetPrivateProfileSectionNames(
    LPSTR lpszReturnBuffer,
    DWORD nSize,
    LPCSTR lpFileName);

#endif // INI2_INIC_H
