#pragma once

#ifndef RTC_LIB_RTC_MUTEX
#define RTC_LIB_RTC_MUTEX

//! RTControlFrameworkが提供する排他処理オブジェクト．
/*! boostだけではできないWindowsAPI提供のmutexとの互換を取る．
*   linuxの場合，boost::interprocess::interprocess_mutexを使用． 
*   windowsの場合，Windows APIのCreateMutex, WaiteforSingleObject等を使う．
*   このRtcMutex.hをお互い使えば同期もできるし，Winなら他のプロセスとも同期できる．
*   となるといいな．
* **** 使い方 ****
* 
* RtcMutex mutex;
* mutex.create_or_open("GLOBAL_MUTEX_NAME");
* mutex.lock();
* ... do something...
* mutex.unlock();
* 
* bool succeed = mutex.lock(0); // lockできなければすぐに処理を戻す.
* bool succeed = mutex.lock(10); // 10msのタイムアウト処理つきmutex.
* 
*/

#include <string>

#if defined(_WIN32) || defined(_WIN64)
	//! WindowsのtimeSetEventを用いた似非リアルタイム周期タイマー
#include <tchar.h>
#include <Windows.h>
#endif

#ifdef __linux__
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/shared_ptr.hpp>
#endif

namespace RTCLib
{
	// ヘッダだけで書きたいのでダミーでテンプレートｗ
	template< class T>
	class RtcMutexImpl{
	public:
		RtcMutexImpl();
		RtcMutexImpl(std::string name);
		~RtcMutexImpl();

		bool create_or_open(std::string name);
		void close();
		bool lock(int TimeOut_msec=-1);	// -1:Infinite, 0:TryLock, n:timedwait
		void unlock();

		bool is_open(){return p_mutex!=NULL;};
		
		//// for boost mutex compatibility... (not enough)
		//bool lock(){Lock();};
		//bool unlock(){Unlock();};
		//bool locked(){return true};	// ??? どうやって実装？ ???
		//bool try_lock(){ return false};
		//bool timed_lock(t){};

	private:
		std::string name;
		void Release();
		//void Close();

#ifdef __linux__
		typedef boost::interprocess::named_mutex  NamedMutex;
		NamedMutex* p_mutex;
#else
		HANDLE p_mutex;
#endif
	};

	

	template<class T>
	RtcMutexImpl<T>::RtcMutexImpl() : p_mutex(NULL), name(""){

	};
	template<class T>
	RtcMutexImpl<T>::RtcMutexImpl(std::string _name) : p_mutex(NULL){
		create_or_open( _name);
	};

	template<class T>
	RtcMutexImpl<T>::~RtcMutexImpl(){
		if(is_open())
		{
			//unlock();
			close();
		};
	};

	template<class T>
	bool RtcMutexImpl<T>::create_or_open(std::string _name){
		name = _name;
		if(is_open())
		{
			//unlock();
			close();
		}
#ifdef __linux__
		p_mutex = new NamedMutex(open_or_create, name.c_str() );
#else
		p_mutex = CreateMutexA(NULL , FALSE , _name.c_str());
#endif
		if(p_mutex==NULL)
		{
			return false;
		}
		return true;
	};

	template<class T>
	void RtcMutexImpl<T>::close(){
		if(p_mutex!=NULL)
		{	
#ifdef __linux__
			delete p_mutex;
#else
			CloseHandle(p_mutex);
#endif
		}
		p_mutex = NULL;
	}

	template<class T>
	bool RtcMutexImpl<T>::lock(int TimeOut_msec){
#ifdef __linux__
		
		if(TimeOut_msec < 0){
			p_mutex->lock();
		}else if(TimeOut_msec==0){
			return p_mutex->try_lock();
		}else{
			// webがないとわかんないわー
			//p_mutex->timed_lock();
		}
#else
		DWORD ret = 0;
		if(TimeOut_msec < 0)
		{
			ret = WaitForSingleObject(p_mutex, INFINITE);
		}
		else
		{
			ret = WaitForSingleObject(p_mutex, TimeOut_msec);
		}
		switch(ret)
		{
		case WAIT_ABANDONED:
			create_or_open(name);
			break;
		case WAIT_OBJECT_0:
			return true;
		case WAIT_TIMEOUT:
			return false;
		}
#endif
		return true;
	}

	template<class T>
	void RtcMutexImpl<T>::unlock(){
#ifdef __linux__
		p_mutex->unlock();
#else
		ReleaseMutex(p_mutex);
#endif
	}

	// 簡単に使えるためのリネーム
	typedef RtcMutexImpl<void> RtcMutex;

	
}

#endif //RTC_LIB_RTC_MUTEX

