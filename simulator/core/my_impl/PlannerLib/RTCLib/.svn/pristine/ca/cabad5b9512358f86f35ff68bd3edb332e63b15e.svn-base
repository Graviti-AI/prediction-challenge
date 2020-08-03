#pragma once
 
#ifndef RT_CTRL_FRAMEWORK_STOP_WATCH_RDTSC
#define RT_CTRL_FRAMEWORK_STOP_WATCH_RDTSC

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>

#ifdef WIN32
#include <intrin.h>		// win32, visual studio専用イントリンシック
// 本当はwin32ではなくvs専用なんですが，ディレクティブを忘れ...
#endif

#ifdef __linux__
#include <stdint.h>		// uint_64tの定義
#endif

/// 
/// RDTSC タイマ for RTCtrlFramework::StopWatch
/// 
/// 最近のx86 CPUで使用可能なRDTSC命令によるカウンタ値からのタイマ．
/// 起動から2年ほどで一周します．まず大丈夫だとは思うんだけどねぇ…
/// いちおうWindowsとLinuxで対応可能なはずです．
/// 
/// 分解能は良い．たぶんPC上では最高．
/// Speedstep，TurboBoostには対応不可．あらかじめOffにしておくこと．
/// マルチコアではアウト．マルチスレッドな場合はSetAffinityMaskが必要ですが...
/// 

namespace RTCLib
{

	// CPU周波数が書いてある定義ファイル
	#include "timCPUFrequency.h"

	// RDTSCによるタイマークラス
	class RDTSCTimer{
	public:

		//64bit整数型の定義. linuxではLONLONGではない．
#ifdef __linux__
		//linux
		typedef uint64_t timer_value;		//unsigned long long int
#endif //__linux__
#ifdef WIN32
		typedef LONGLONG timer_value;
#endif // WIN32


#ifdef __linux__		// linux用の実装

		//RDTSC命令のコア部分. インラインアセンブラ，あるいはイントリンシックで記述
//		extern "C" {
			inline timer_value get_time(void)
			{
//#if 0	// 昔の実装
//				timer_value x;
//				__asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
//				return x;
//#else
				// 最近の実装 from http://en.wikipedia.org/wiki/Time_Stamp_Counter
				uint32_t lo, hi;
				__asm__ __volatile__ (      // serialize
					"xorl %%eax,%%eax \n        cpuid"
					::: "%rax", "%rbx", "%rcx", "%rdx");
				/* We cannot use "=A", since this would use %rax on x86_64 and return only the lower 32bits of the TSC */
				__asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
				return (uint64_t)hi << 32 | lo;
//#endif
			}
//		}
#endif //__linux__


#ifdef WIN32		// windows用の実装
		inline timer_value get_time(void)
		{
			// 昔の実装. インラインアセンブラで実装
#if 0
			__asm {
				cpuid
					rdtsc
			}
#else
			// 今の実装．MSVCイントリンシックを使用 from http://en.wikipedia.org/wiki/Time_Stamp_Counter
			return __rdtsc();
#endif

		}
#endif // WIN32

		timer_value get_frequency( void )
		{
			return static_cast<timer_value>(CPU_CLOCK_PER_SEC);
		}

		// CPU周波数の測定装置。ぱそこん変わったときとか。
		// 
		timer_value cpufreq_research( int averaging_sec = 5, std::ostream& os = std::cout)
		{
			
			// これまでの処理をすませるために10msecくらいちょっとWait
			sleep_ms(10);

			// averaging_sec秒だけかけて，平均的なRDTSCの周波数を測定

			// 初期rdtscカウント			
			timer_value st = get_time();					// rdtscを取得
			
			// すりーぷ…
			sleep_sec(averaging_sec);

			// 再度計測
			timer_value ret = (get_time() - st) / averaging_sec;	// sleep後のカウンタ値を取得し，平均する

			// 結果を出力しましょう．
			std::cout << "Measured ticks / 1sec :" << ret;
			std::cout << ", i.e. " << ((ret/1000)/1000.0) << "MHz" << std::endl;
			std::cout << std::endl << "Result is output to stream..." << std::endl << std::endl;
			os << "//This CPU's clock was measured as follows:" << std::endl;
			os << "const double CPU_CLOCK_PER_SEC = " << ret << "; //[Hz]=[clock/sec]" << std::endl;
			os << "const double CPU_SEC_PER_CLOCK = " << 1.0 / static_cast<double>(ret) << "; //[sec/clock]" << std::endl;
			return ret;
		}

	private:
		
		inline void get_ticks(boost::xtime& xt)
		{
			// changed from boost version 1.5
#if BOOST_VERSION >= 105000
			boost::xtime_get( &xt, boost::TIME_UTC_);
#else
			boost::xtime_get( &xt, boost::TIME_UTC);
#endif
		}
		
		void sleep_ms(int ms)
		{
			if(ms < 0)
				ms = 0;
			boost::xtime xt;
			get_ticks(xt);

			// milli seconds sleep
			xt.nsec += 1000 * 1000 * ms;
			boost::thread::sleep( xt );
		}

		void sleep_sec(int sec)
		{
			if(sec < 0)
				sec = 0;
			boost::xtime xt;
			get_ticks(xt);

			// milli seconds sleep
			xt.sec += sec;
			boost::thread::sleep( xt );
		}


	};

}

#endif //RT_CTRL_FRAMEWORK_STOP_WATCH_QPC

