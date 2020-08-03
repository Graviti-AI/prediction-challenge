/*!
 * <共通メモリ経由でのデータ共有テンプレートクラス>
 * 
 * Copyright (c) 2011 by Hiroyuki Okuda
 *
 * PC内部でのIPCを行う際の共通メモリでPOD型を共有．
 * linuxでもwindowsでも共有できるを目指す．
 * ただし，winの場合，標準の共有メモリからもアクセス可にする．
 */

#pragma once

#pragma warning ( push )
#pragma warning ( disable : 4819 )
#pragma warning ( disable : 4345 )

// 共有メモリ通信に必要なヘッダファイル群
#if defined(_WIN32) || defined(_WIN64)
#include <boost/interprocess/windows_shared_memory.hpp>
#endif

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "RtcMutex.h"
#include <string>
#include <iostream>

namespace RTCLib{

// 面倒なのでpublicごめん
// まるちすれっども適当対応なのでごめん
// アクセスするときは，対応するミューテックスを自分でlockしてアクセスしてね.
template <class T>
class SharedMemoryAccessor
{
public:
	// 共有メモリを開く
	bool Open(bool serve_host, const char* sharemem_name, const char* sharemutex_name="");

	// 最初サーバとしてオープンを試み，失敗したらクライアントとしてオープンする
	bool TryOpen(const char* sharemem_name, const char* sharemutex_name="");

	// 名前付きmutexを使ったプロセス間のリソースロック
	bool lock();
	void unlock();

	// 共有メモリのホストプログラムはtrueにて構築，それ以外はfalseにて構築
	SharedMemoryAccessor():m_opened(false), p_shared_data(0){}
	SharedMemoryAccessor(bool serve_host, const char* sharemem_name, const char* sharemutex_name=""):m_opened(false), p_shared_data(0){
		Open(serve_host, sharemem_name, sharemutex_name);
	}
	~SharedMemoryAccessor();

	// マルチプロセスでの排他アクセスオブジェクトをリネーム
	typedef boost::interprocess::interprocess_mutex Ipc_mutex;
	typedef boost::interprocess::scoped_lock<Ipc_mutex> Scoped_lock;

	// ホスト用の自動共有メモリ削除オブジェクト
#ifdef __linux__
	struct Remover{
		Remover(char* name){ 
			std::strncpy(_name, name, 256 );
			boost::interprocess::shared_memory_object::remove( sharemem_name );
			
		}
		~Remover(){ boost::interprocess::shared_memory_object::remove( sharemem_name ); }
		char _name[256];
	};
	std::unique_ptr< Remover > premover;
#endif

	// 共有メモリマネージャ
#ifdef WIN32
	// windowsでは他プロセスとかネイティブと対応できるように．
	std::unique_ptr< boost::interprocess::windows_shared_memory > pshm;
	std::unique_ptr< boost::interprocess::mapped_region > pregion;
#else
	// linuxはboostさんだよりで
	std::unique_ptr< boost::interprocess::managed_shared_memory > pshm;
#endif

	bool m_opened;

	/// データが追加で必要になる場合には下記に追加する．
	/// 共有するデータへのポインタ

	// センシングデータと，それ用のミューテックスオブジェクト
	T* p_shared_data;
	T* Get(){ return p_shared_data; }
	
	/// ↓本当はwin32のAPIとboostのnamed_mutexを切り替えたい...
#ifdef __linux__ 
	//Ipc_mutex* p_named_mutex;
#else
	//HANDLE p_named_mutex;
#endif
	RTCLib::RtcMutex mutex;

};

template <class T>
bool SharedMemoryAccessor<T>::TryOpen(const char* sharemem_name=NULL, const char* sharemutex_name="")
{
	try{
		Open(true, sharemem_name, sharemutex_name);
		return true;
	}
	catch(...){
		try{
			Open(false, sharemem_name, sharemutex_name);
			return true;
		}
		catch(...){
			return false;
		}
	}
}

template <class T>
bool SharedMemoryAccessor<T>::Open(bool serve_host, const char* sharemem_name=NULL, const char* sharemutex_name="")
{
	mutex.create_or_open( sharemutex_name );
	if(serve_host){

		// サーバー動作
#ifdef WIN32
		if(sharemem_name==NULL)sharemem_name = "DEFAULT_SHARED_MEMORY_NAME";

		// For Windows : C#と通信可能なようにWindowsの共有メモリを使うでよ
		pshm.reset( new boost::interprocess::windows_shared_memory( 
			boost::interprocess::create_only, sharemem_name, boost::interprocess::read_write, sizeof(T) )
			);
		pregion.reset( new boost::interprocess::mapped_region( *pshm, boost::interprocess::read_write) );

		// 共有メモリ領域の先頭アドレスを獲得
		void* head = (void*)pregion->get_address();
		
		// 共有データを構築: placement new;
		p_shared_data = new(head) T();

		// mutexを構築
		//p_lrf_mutex = new(head) Ipc_mutex();
		
#else
		// For linux
		// 終了時の自動削除を登録，同時に削除する．
		premover.reset( new Remover() );

		// 新たに共有領域を作成
		pshm.reset( new  managed_shared_memory( boost::interprocess::create_only , SHARED_MEMORY_NAME, 65536 ) );

		// 共有メモリ上にオブジェクトを確保

		//p_lrf_mutex = pshm->construct<Ipc_mutex>(SHARED_LRF_DATA_MUTEX_NAME)();
		p_shared_data = pshm->construct<T>(SHARED_LRF_DATA_NAME)();
		//memset( p_lrf_data, 0, sizeof(p_lrf_data) );

#endif

	}else{

		// アクセスクライアント動作
#ifdef WIN32
		// For Windows : C#と通信可能なようにWindowsの共有メモリを使うでよ
		try{
			pshm.reset( 
				new boost::interprocess::windows_shared_memory( boost::interprocess::open_only, 
																sharemem_name, 
																boost::interprocess::read_write )
				);
		}catch( boost::interprocess::interprocess_exception e)
		{
			std::cerr << "Error! Shared memory is not ready. Please launch host program." << std::endl;
			return false;
		}
		pregion.reset( new boost::interprocess::mapped_region( *pshm, boost::interprocess::read_write) );

		// 共有メモリ領域の先頭アドレスを獲得
		void* head = (void*)pregion->get_address();

		// 共有データ:placed_new
		p_shared_data = (T*)head;//new (head)T(); 

#else
		// For linux
		pshm.reset( new  managed_shared_memory( boost::interprocess::open_only , SHARED_MEMORY_NAME ) );

		//auto res1 = pshm->find<Ipc_mutex>(SHARED_LRF_DATA_MUTEX_NAME);
		//if( res1.second != 1 )return;
		//p_lrf_mutex = res1.first;

		auto res2 = pshm->find<Sensored_Data>(SHARED_LRF_DATA_NAME);
		p_lrf_data = res2.first;

		//auto res3 = pshm->find<Ipc_mutex>(SHARED_POS_DATA_MUTEX_NAME);
		//p_pos_mutex = res3.first;
		
		auto res4 = pshm->find<Shared_Data_POS>(SHARED_POS_DATA_NAME);
		p_pos_data = res4.first;
#endif
	}
	m_opened = true;
	return true;
}

// おそらく自動解放
template <class T>
SharedMemoryAccessor<T>::~SharedMemoryAccessor()
{

}

// 名前付きmutexを使ったプロセス間のリソースのロック
template <class T>
bool SharedMemoryAccessor<T>::lock()
{
	return mutex.lock();
}

// 名前付きmutexを使ったプロセス間のリソースのアンロック
template <class T>
void SharedMemoryAccessor<T>::unlock()
{
	mutex.unlock();
}

}

#pragma warning ( pop )
