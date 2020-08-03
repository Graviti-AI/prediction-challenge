#pragma once

#ifndef RT_CTRL_FRAMEWORK_UTILITY
#define RT_CTRL_FRAMEWORK_UTILITY

//#define NO_RTC_INPUT

//! RTControlFrameworkが提供するユーティリティ．
/*! いくつかの便利なヘルパ関数，ユーティリティを提供する．

**** 使い方 ****

#define RTC_INPUT
#include <RtcUtility.h>

...
while(true){
if( 0 != async_key_poll() )break;
}

// ------------CyclicThreadBaseクラスを使うときのテンプレートを-----------
// ------------------------コピー用にまとめておきます---------------------
friend RTCtrlFramework::CyclicThreadBase<HogeHogeThread>;

// 別スレッドに最初に実行される関数．
void Initialize( void );
// 別スレッドに周期的に実行される関数．
void TimerProc( void );
// 別スレッドに最後に実行される関数．
void Terminate( void );
// --------------------------------- ここまで -------------------
*/

//! Enable key input helper if RTC_INPUT defined
#if defined(RTC_INPUT) || !defined(NO_RTC_INPUT)
#include <iostream>
#ifdef WIN32
#include <conio.h>
#endif
#ifdef __linux__
#include <sys/time.h>
#endif
#endif

#ifdef RTC_SINGLETON
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>
#endif

namespace RTCLib{
	////! インデックスアクセッサoperator[]をサポートする
	//template<class LH, class RH>
	//void unsafe_copy_array(LH& receiver, const RH& sender)
	//{
	//	
	//}

	//! OSに依存しないキーボード入力の非ブロックポーリングを提供
	/// 不要であればNO_RTC_INPUTを定義して読み込むこと
#if defined(RTC_INPUT) && !defined(NO_RTC_INPUT)
	namespace Utility{

		/// キーをポーリングする．
		/// もしキーを押していなければ0，押していれば0以外を返す．
		/// Windowsならば_kbhitを用い，linuxなら適宜なんかする．
		///
		int async_key_poll()
		{
#ifdef WIN32
			return _kbhit();
#endif
#ifdef __linux__
			fd_set fdset;
			struct timeval timeout;
			int  rc;
			//int  val;

			timeout.tv_sec = 0;
			timeout.tv_usec = 1;   /* wait for 1 usec for key */

			FD_ZERO(&fdset);
			FD_SET(0, &fdset);	// file '0' show the stdin


			rc = select(1, &fdset, NULL, NULL, &timeout);
			if (rc == -1)  /* select failed */
			{
				printf("stdin cannot be capture\n");
			}
			else if (rc == 0)  /* select timed out */
			{
				return 0;
			}
			return 1;
#endif
		}

		//! OSに依存しないキーボード非同期入力を提供
		/// enterが入力されるまでは入力を決定しない
		/// 途中経過も取得できなくはない．
		/// Warning! ただいまasync_key_block_inputと同じ実装！
		int async_key_input( std::string &str )
		{
			//static std::string s_str;
			int ret = async_key_poll();
			if(ret==0)
			{
				return 0;
			}else{
				std::cin >> str;
				return str.length();
			}
		}

		//! OSに依存しないキーボード準非同期入力を提供
		/// 何かが入力されるまでは非同期でブロックしないが，
		/// 何かキーボードが入力されたら，ブロック入力を試みる．
		int async_key_block_input( std::string &str )
		{
			int ret = async_key_poll();
			if(ret==0)
			{
				return 0;
			}else{
				std::cin >> str;
				return str.length();
			}
		}

	} // end of namespace Utility
#endif //NO_RTC_INPUT

#ifdef RTC_SINGLETON

	//! シングルトンテンプレート
	template <class T>
	class RTCSingleton
	{
		// 各テンプレートクラスTごとに確保される
		static boost::shared_ptr<T> ptr;
		// 各テンプレートクラスTごとに確保される
		static boost::mutex m_mutex;

	public:
		boost::shared_ptr<T> get(){
			if(!ptr)
			{
				boost::mutex::scoped_lock lk(m_mutex);
				if(!ptr)
				{
					ptr.reset( new T() );
				}
			}
			return ptr;
		}
		void lock()		{ m_mutex.lock();	}
		void unlock()	{ m_mutex.unlock(); }

	};
	template<class T>
	boost::shared_ptr<T> RTCSingleton<T>::ptr;

	template<class T>
	boost::mutex RTCSingleton<T>::m_mutex;

#endif //RTC_SINGLETON
} // namespace RTCtrlFramework

#endif //RT_CTRL_FRAMEWORK_UTILITY

