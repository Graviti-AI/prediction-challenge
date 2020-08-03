#pragma once

#ifndef RT_CTRL_FRAMEWORK_STOP_WATCH_QPC
#define RT_CTRL_FRAMEWORK_STOP_WATCH_QPC

// QPCTimerはWindowsでのみ有効
#if defined(_WIN32) || defined(WIN32) || defined(WIN64) || defined(_WIN64)

/// 
/// QueryPerformanceCounterタイマ for RTCtrlFramework::StopWatch
/// 
/// Windowsでのみ使用可能なパフォーマンスカウンタを提供．
/// 分解能は良い．Speedstepが有効だと正しい時間計測が不可？
/// TurboBoostには対応可能．マルチコアでも大丈夫．
/// 


//For QueryPerformanceX
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")

namespace RTCLib
{

	class QPCTimer
	{
	public:
		
		typedef LONGLONG timer_value;	//タイマー格納用データ型
		
		// タイマーの分解能を取得. ticks/secの次元
		timer_value get_frequency( )
		{
			timer_value ret;
			::QueryPerformanceFrequency( (LARGE_INTEGER*)&ret );
			return ret;
		}
		
		// タイマーの現在地を取得
		timer_value get_time( )
		{
			timer_value ret;
			::QueryPerformanceCounter( (LARGE_INTEGER*)&ret );
			return ret;
		}

		static double elapsed_from( timer_value from, timer_value to )
		{
			//(to - from)
			return 0;
		}

	};

}

#endif // _WIN32 || WIN32 || WIN64 || _WIN64

#endif //RT_CTRL_FRAMEWORK_STOP_WATCH_QPC

