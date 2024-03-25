#include <iostream>
#include "inic.h"
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

CIniFile::CIniFile(void)
{
}
CIniFile::CIniFile(const char *chFileName) : m_strFileName(chFileName)
{
    Init();
}
CIniFile::CIniFile(const string &strFileName) : m_strFileName(strFileName)
{
    Init();
}
CIniFile::~CIniFile(void)
{
}

int CIniFile::Init(const char *pFileName)
{
    m_strFileName = pFileName;
    return LoadFile();
}

int CIniFile::Init(const string &strFileName)
{
    m_strFileName = strFileName;
    return LoadFile();
}
int CIniFile::Init()
{
    return LoadFile();
}

int CIniFile::Dump()
{
    map<string, ConfigType>::iterator tSecIter1 = m_mSec2Config.begin(), tSecIter2 = m_mSec2Config.end();
    ConfigType::iterator tConfigTypeIter1, tConfigTypeIter2;

    while (tSecIter1 != tSecIter2)
    {
        cout << "[" << tSecIter1->first << "]" << endl;
        tConfigTypeIter1 = tSecIter1->second.begin(), tConfigTypeIter2 = tSecIter1->second.end();
        while (tConfigTypeIter1 != tConfigTypeIter2)
        {
            cout << tConfigTypeIter1->first << "=" << tConfigTypeIter1->second << endl;

            ++tConfigTypeIter1;
        }
        cout << endl;
        ++tSecIter1;
    }

    return 0;
}

int CIniFile::ReadItem(const string &strSection, const string &strKey, const string &strDefault, string &strValue)
{
    strValue = strDefault;
    if (!m_mSec2Config.count(strSection))
    {
        return -1;
    }

    ConfigType &tConfigType = m_mSec2Config[strSection];
    if (tConfigType.count(strKey))
    {
        strValue = tConfigType[strKey];
        return 0;
    }
    else
    {
        strValue = strDefault;
        return -1;
    }
}

int CIniFile::WriteItem(const string &strSection, const string &strKey, const string &strValue)
{
    ConfigType &tConfigType = m_mSec2Config[strSection];
    if (tConfigType.count(strKey))
    {
        ; // return -1;
    }
    tConfigType[strKey] = strValue;

    return WriteFile();
}

int CIniFile::LoadFile()
{
    FILE *pFile;

    if (NULL == (pFile = ::fopen(m_strFileName.c_str(), "r")))
    {
        return -1;
    }

    string strLine, strSection;
    string strKey, strValue;
    size_t nPos, nEndPos;
    ConfigType tConfigType;
    while (0 == ReadLine(pFile, strLine))
    {
        if (string::npos != (nPos = strLine.find_first_of("[")))
        {
            if (string::npos == (nEndPos = strLine.find_first_of("]")))
            {
                ::fclose(pFile);
                return -1;
            }

            strSection = strLine.substr(nPos + 1, nEndPos - nPos - 1);
            if (0 > TrimString(strSection))
            {
                ::fclose(pFile);
                return -1;
            }
        }
        else if (string::npos != (nPos = strLine.find_first_of("=")))
        {
            strKey = strLine.substr(0, nPos);
            strValue = strLine.substr(nPos + 1);

            if (0 > TrimString(strKey) || 0 > TrimString(strValue) || strSection.empty())
            {
                ::fclose(pFile);
                return -1;
            }

            m_mSec2Config[strSection][strKey] = strValue;
        }
    }

    return ::fclose(pFile);
}

int CIniFile::WriteFile()
{
    FILE *pFile;

    if (NULL == (pFile = ::fopen(m_strFileName.c_str(), "w")))
    {
        return -1;
    }

    map<string, ConfigType>::iterator tSecIter1 = m_mSec2Config.begin(), tSecIter2 = m_mSec2Config.end();
    ConfigType::iterator tConfigTypeIter1, tConfigTypeIter2;

    string strSection, strConfig;
    while (tSecIter1 != tSecIter2)
    {
        strSection = string("[") + tSecIter1->first + string("]\n");
        ::fwrite(strSection.c_str(), sizeof(char), strSection.length(), pFile);
        tConfigTypeIter1 = tSecIter1->second.begin(), tConfigTypeIter2 = tSecIter1->second.end();
        while (tConfigTypeIter1 != tConfigTypeIter2)
        {
            strConfig = tConfigTypeIter1->first + string("=") + tConfigTypeIter1->second + string("\n");
            ::fwrite(strConfig.c_str(), sizeof(char), strConfig.length(), pFile);

            ++tConfigTypeIter1;
        }
        ::fwrite("\n", sizeof(char), 1, pFile);
        ++tSecIter1;
    }

    return ::fclose(pFile);
}

/*读取FILE文件中的内容*/
int CIniFile::ReadLine(FILE *pFile, string &strLine)
{
    char szBuff[BUFFER_LEN];
    int nLen;
    char *pos = NULL;
    do
    {
        if (NULL == ::fgets(szBuff, BUFFER_LEN, pFile))
        {
            return -1;
        }
        if ((pos = strstr(szBuff, "\n")))
            pos = 0;
        if ((pos = strstr(szBuff, "\r")))
            pos = 0;
        if (0 < (nLen = (int)::strlen(szBuff)))
        {
            break;
        }

    } while (true);

    szBuff[nLen - 1] = '\0';

    strLine = szBuff;

    return 0;
}

int CIniFile::TrimString(string &strToken)
{
    if (strToken.empty())
    {
        return -1;
    }

    size_t nPos = strToken.find_first_not_of(" \t");
    size_t nEndPos = strToken.find_last_not_of(" \t");

    strToken = strToken.substr(nPos, nEndPos - nPos + 1);

    return (strToken.empty()) ? -1 : 0;
}

UINT GetPrivateProfileInt(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    INT nDefault,
    LPCSTR lpFileName)
{
    char lpReturnedString[100] = {0};
    DWORD nRet = GetPrivateProfileString(
        lpAppName,
        lpKeyName,
        "",
        lpReturnedString,
        sizeof(lpReturnedString),
        lpFileName);
    if (nRet == 0)
        return nDefault;
    return atoi(lpReturnedString);
}

BOOL WritePrivateProfileString(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    LPCSTR lpString,
    LPCSTR lpFileName)
{
    FILE *fp;
    static char szLine[10240] = {0};
    static char tmpstr[10240] = {0};
    memset(szLine, 0, sizeof(szLine));
    memset(tmpstr, 0, sizeof(tmpstr));
    int rtnval;
    int i = 0;
    int secFlag = 0;

    if ((fp = fopen(lpFileName, "rw+")) == NULL)
    {
#ifdef DEBUG
        printf("have   no   such   file \n");
#endif
        return FALSE;
    }

    int lineLen = 0;     // 整行长度
    int orgEqualPos = 0; //=号在原行中的位置
    int equalPos = 0;    //=号在去空格后的位置
    strcpy(tmpstr, "[");
    strcat(tmpstr, lpAppName);
    strcat(tmpstr, "]");
    int endFlag;
    while (!feof(fp))
    {

        rtnval = fgetc(fp);
        if (rtnval == EOF)
        {
            // 最后一行可能无换行符号
#ifdef DEBUG
            printf("EOF\n");
#endif
            rtnval = '\n';
            endFlag = 1;
        }
        // 注释行
        if ('#' == rtnval || ';' == rtnval)
        {
            fgets(szLine, sizeof(szLine), fp);
            // reset
            i = 0;
            lineLen = 0;
            orgEqualPos = 0;
            equalPos = 0;
            memset(szLine, 0, sizeof(szLine));
            continue;
        }
        else if ('/' == rtnval)
        {
            szLine[i++] = rtnval;
            lineLen++;
            if ('/' == (rtnval = fgetc(fp))) // 注释行
            {
                fgets(szLine, sizeof(szLine), fp);
                // reset
                i = 0;
                lineLen = 0;
                orgEqualPos = 0;
                equalPos = 0;
                memset(szLine, 0, sizeof(szLine));
                continue;
            }
        }

        if (rtnval != ' ' && rtnval != '\t')
        {
            szLine[i++] = rtnval; // 去掉空格和tab后的字符串
            if (rtnval == '=')
            {
                orgEqualPos = lineLen;
                equalPos = i - 1;
            }
        }

        lineLen++; // 字符

        if (rtnval == '\n')
        {
#ifdef DEBUG
            printf("Line %s\n", szLine);
#endif
            szLine[--i] = '\0';
            if (szLine[--i] == '\r')
                szLine[i--] = '\0';

            if ((equalPos != 0) && (secFlag == 1))
            {
                szLine[equalPos] = '\0';
                if (strcasecmp(szLine, lpKeyName) == 0)
                {
                    // 找到key对应变量
                    int leftPos = ftell(fp);
                    int writePos = leftPos - lineLen + orgEqualPos + 1;
                    fseek(fp, 0, SEEK_END);
                    int leftLen = ftell(fp) - leftPos;
                    char *pLeft = new char[leftLen];
                    fseek(fp, leftPos, SEEK_SET);
                    fread(pLeft, leftLen, 1, fp);
                    fseek(fp, writePos, SEEK_SET);
                    fwrite(lpString, strlen(lpString), 1, fp);
                    fwrite("\n", sizeof(char), 1, fp);
                    fwrite(pLeft, leftLen, 1, fp);
                    delete[] pLeft;
                    pLeft = 0;
                    fclose(fp);
                    return TRUE;
                }
            }

            else
            {
                if (strcasecmp(tmpstr, szLine) == 0)
                {
                    // 找到section
                    secFlag = 1;
                }
                else if (secFlag == 1 && szLine[0] == '[' && szLine[i] == ']')
                { // 进入下个section了，说明没找到
                    int leftPos = ftell(fp) - lineLen;
                    int writePos = leftPos;
                    fseek(fp, 0, SEEK_END);
                    int leftLen = ftell(fp) - leftPos;
                    char *pLeft = new char[leftLen];
                    fseek(fp, leftPos, SEEK_SET);
                    fread(pLeft, leftLen, 1, fp);
                    fseek(fp, writePos, SEEK_SET);
                    fwrite("\n", sizeof(char), 1, fp);
                    fwrite(lpKeyName, strlen(lpKeyName), 1, fp);
                    fwrite("=", sizeof(char), 1, fp);
                    fwrite(lpString, strlen(lpString), 1, fp);
                    fwrite("\n", sizeof(char), 1, fp);
                    fwrite(pLeft, leftLen, 1, fp);
                    delete[] pLeft;
                    pLeft = 0;
                    fclose(fp);
                    return TRUE;
                }
            }
            // reset
            if (endFlag == 1)
                break;
            i = 0;
            lineLen = 0;
            orgEqualPos = 0;
            equalPos = 0;
            memset(szLine, 0, sizeof(szLine));
        }
    }
    // 到文件尾了
    if (secFlag)
    { // 必须有section
        fseek(fp, 0, SEEK_END);
        fwrite("\n", sizeof(char), 1, fp);
        fwrite(lpKeyName, strlen(lpKeyName), 1, fp);
        fwrite("=", sizeof(char), 1, fp);
        fwrite(lpString, strlen(lpString), 1, fp);
        fwrite("\n", sizeof(char), 1, fp);
    }
    fclose(fp);
    return TRUE;
}
DWORD
GetPrivateProfileString(
    LPCSTR lpAppName,
    LPCSTR lpKeyName,
    LPCSTR lpDefault,
    LPSTR lpReturnedString,
    DWORD nSize,
    LPCSTR lpFileName)
{
    FILE *fp;
    static char szLine[10240] = {0};
    static char tmpstr[10240] = {0};
    memset(szLine, 0, sizeof(szLine));
    memset(tmpstr, 0, sizeof(tmpstr));
    int rtnval;
    int i = 0;
    int secFlag = 0;

    if ((fp = fopen(lpFileName, "r")) == NULL)
    {
#ifdef DEBUG
        printf("have   no   such   file \n");
#endif
        return -1;
    }

    int equalPos = 0; //=号在去空格后的位置
    strcpy(tmpstr, "[");
    strcat(tmpstr, lpAppName);
    strcat(tmpstr, "]");
    int endFlag;
    while (!feof(fp))
    {
        rtnval = fgetc(fp);
        if (rtnval == EOF)
        {
            // 最后一行可能无换行符号
#ifdef DEBUG
            printf("EOF\n");
#endif
            rtnval = '\n';
            endFlag = 1;
        }
        // 注释行
        if ('#' == rtnval || ';' == rtnval)
        {
            fgets(szLine, sizeof(szLine), fp);
            // reset
            i = 0;
            equalPos = 0;
            memset(szLine, 0, sizeof(szLine));
            continue;
        }
        else if ('/' == rtnval)
        {
            szLine[i++] = rtnval;
            if ('/' == (rtnval = fgetc(fp))) // 注释行
            {
                fgets(szLine, sizeof(szLine), fp);
                // reset
                i = 0;
                equalPos = 0;
                memset(szLine, 0, sizeof(szLine));
                continue;
            }
        }

        if (rtnval != ' ' && rtnval != '\t')
        {
            szLine[i++] = rtnval; // 去掉空格和tab后的字符串
            if (rtnval == '=')
            {
                equalPos = i - 1;
            }
        }

        if (rtnval == '\n')
        {

#ifdef DEBUG
            printf("Line %s\n", szLine);
#endif
            szLine[--i] = '\0';
            if (szLine[--i] == '\r')
                szLine[i--] = '\0';

            if ((equalPos != 0) && (secFlag == 1))
            {
                szLine[equalPos] = '\0'; //=号变0
                if (strcasecmp(szLine, lpKeyName) == 0)
                {
                    // 找到key对应变量
                    strncpy(lpReturnedString, szLine + equalPos + 1, nSize - 1);
                    lpReturnedString[nSize - 1] = '\0';
                    fclose(fp);
                    return 1;
                }
            }
            else
            {
                if (strcasecmp(tmpstr, szLine) == 0)
                {
                    // 找到section
                    secFlag = 1;
                }
                else if (secFlag == 1 && szLine[0] == '[' && szLine[i] == ']')
                { // 进入下个section了，说明没找到
                    break;
                }
            }

            if (endFlag == 1)
                break;
            // reset
            i = 0;
            equalPos = 0;
            memset(szLine, 0, sizeof(szLine));
        }
    }
    fclose(fp);
    // 没找到则用默认
    strncpy(lpReturnedString, lpDefault, nSize - 1);
    lpReturnedString[nSize - 1] = '\0';
    return 0;
}

DWORD
GetPrivateProfileSection(
    LPCSTR lpAppName,
    LPSTR lpReturnedString,
    DWORD nSize,
    LPCSTR lpFileName)
{
    // 由于项目中未使用，暂未实现
    assert(0);
    return 0;
}

DWORD
GetPrivateProfileSectionNames(
    LPSTR lpszReturnBuffer,
    DWORD nSize,
    LPCSTR lpFileName)
{
    FILE *fp;
    static char szLine[10240] = {0};
    static char tmpstr[10240] = {0};
    memset(szLine, 0, sizeof(szLine));
    memset(tmpstr, 0, sizeof(tmpstr));
    int rPos = 0;
    memset(lpszReturnBuffer, 0, nSize);
    int rtnval;
    int i = 0;
    int endFlag;

    if ((fp = fopen(lpFileName, "r")) == NULL)
    {
#ifdef DEBUG
        printf("have   no   such   file \n");
#endif
        return -1;
    }

    while (!feof(fp))
    {
        rtnval = fgetc(fp);
        if (rtnval == EOF)
        {
            // 最后一行可能无换行符号
#ifdef DEBUG
            printf("EOF\n");
#endif
            rtnval = '\n';
            endFlag = 1;
        }
        // 注释行
        if ('#' == rtnval || ';' == rtnval)
        {
            fgets(szLine, sizeof(szLine), fp);
            // reset
            i = 0;
            memset(szLine, 0, sizeof(szLine));
            continue;
        }
        else if ('/' == rtnval)
        {
            szLine[i++] = rtnval;
            if ('/' == (rtnval = fgetc(fp))) // 注释行
            {
                fgets(szLine, sizeof(szLine), fp);
                // reset
                i = 0;
                memset(szLine, 0, sizeof(szLine));
                continue;
            }
        }

        if (rtnval != ' ' && rtnval != '\t')
        {
            szLine[i++] = rtnval; // 去掉空格和tab后的字符串
        }

        if (rtnval == '\n')
        {

#ifdef DEBUG
            printf("Line %s\n", szLine);
#endif
            szLine[--i] = '\0';
            if (szLine[--i] == '\r')
                szLine[i--] = '\0';

            if (szLine[0] == '[' && szLine[i] == ']')
            {
                // 找到section
                for (int j = 1; j < i && rPos < nSize - 1; j++)
                    lpszReturnBuffer[rPos++] = szLine[j];
                lpszReturnBuffer[rPos++] = '\0';
                if (rPos >= nSize)
                {
                    break;
                }
            }

            if (endFlag == 1)
                break;

            // reset
            i = 0;
            memset(szLine, 0, sizeof(szLine));
        }
    }
    lpszReturnBuffer[rPos] = '\0';
    fclose(fp);
    return 0;
}
