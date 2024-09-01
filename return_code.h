#pragma once


enum class ReturnCode
{
	kSuccess = 0,
	kUnknownError = 1,
	kInvalidInput = 101,
	kInvalidModel = 102,

	kErrorInMarginLine = 201,
};


inline int ToInt(ReturnCode code)
{
	return static_cast<int>(code);
}