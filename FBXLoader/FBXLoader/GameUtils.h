#pragma once
#include <string>
#include <locale>
#include <codecvt>

class GameUtils
{
public:
	static std::wstring s2ws(const std::string& str);
	static std::string ws2s(const std::wstring& wstr);
};

