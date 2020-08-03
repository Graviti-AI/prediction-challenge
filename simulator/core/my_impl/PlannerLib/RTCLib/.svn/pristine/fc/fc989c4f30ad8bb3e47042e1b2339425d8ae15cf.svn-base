/*!
 * RTCLib::sleep functions, 
 * 
 * Copyright (c) 2008 by <your name/ organization here>
 *
 * スリープ関数"sleep_sec"と"sleep_msec"を使用可能に。
 * 
 */
#pragma once

#ifndef RT_CTRL_FRAMEWORK_THREAD_SLEEP
#define RT_CTRL_FRAMEWORK_THREAD_SLEEP

#ifdef WIN32
	// windows sleep
	#include <Windows.h>
#else
	#ifdef RTCLIB_SLEEP_BOOST_POSIX
	// boost sleep
	#include <boost/thread.hpp>
	#include <boost/date_time/posix_time/posix_time_types.hpp>
	#endif
#endif

namespace RTCLib
{
	/*!
	 * \brief
	 * sec秒間sleepする．
	 * 
	 * \param sec
	 * sleepする秒数を指定。
	 */
	inline void sleep_sec(int sec = 0)
	{
#ifdef WIN32
		Sleep( sec * 1000 );
#else
	#ifdef RTCLIB_SLEEP_BOOST_POSIX
			boost::thread::sleep(boost::get_system_time() + boost::posix_time::seconds(sec));
	#endif
#endif
	}

	/*!
	 * \brief
	 *  msecミリ秒間sleepする．
	 * 
	 * \param sec
	 * sleepする秒数を指定。
	 */
	inline void sleep_msec(int msec = 0)
	{
#ifdef WIN32
		Sleep( msec );
#else
	#ifdef RTCLIB_SLEEP_BOOST_POSIX
		boost::thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(msec));
	#endif
#endif

	}

}

#endif //RT_CTRL_FRAMEWORK_THREAD_SLEEP

