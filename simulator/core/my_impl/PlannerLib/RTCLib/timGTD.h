#pragma once

#ifndef RT_CTRL_FRAMEWORK_STOP_WATCH_GTD
#define RT_CTRL_FRAMEWORK_STOP_WATCH_GTD

#ifdef __linux__

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>

#include <stdint.h>		// uint_64tの定義

/// 
/// GetTimeofDayタイマ for RTCtrlFramework::StopWatch
/// 
/// linuxでのみ使用可能なタイマを提供．
/// 分解能は良い．
/// 詳細は不明．
/// 

namespace RTCLib
{

	class GTDTimer
	{
	public:
		
		typedef double timer_value;	//タイマー格納用データ型
		
		// タイマーの分解能を取得. ticks/secの次元
		timer_value get_frequency( )
		{
			return 1000.0;
		}
		
		// タイマーの現在地を取得
		timer_value get_time( )
		{
			timeval tv;
			gettimeofday( &tv, NULL );
			double ret = static_cast<double>(tv.tv_sec) * 1000.0
							+ static_cast<double>(tv.tv_usec) * 0.001;
			return ret;
		}

		////経過時間を秒で返す
		//double elapsed() const
		//{
		//#if defined(WIN32) || defined(_WIN32)
		//LARGE_INTEGER stopTime;
		//QueryPerformanceCounter(&stopTime);
		//return (double)(stopTime.QuadPart - startTime.QuadPart)/(double)frequency.QuadPart;
		//#else
		//timeval stopTime;
		//gettimeofday(&stopTime, NULL);
		//return (stopTime.tv_sec - startTime.tv_sec) + (stopTime.tv_usec - startTime.tv_usec) * 1e-6;
		//#endif
		//}

	};

}


#endif


#endif //RT_CTRL_FRAMEWORK_STOP_WATCH_GTD

