#pragma once

#ifndef RTC_LIB_RTC_THREAD
#define RTC_LIB_RTC_THREAD

//! RTC_LIBが提供する周期実行スレッドテンプレート．
/*! boost::threadをベースに，リアルタイム制御機構を提供する．
 * 
 *  **** 使い方 ****
 * 
 *  制御用スレッドクラス，HogeThreadを作成する場合，
 *  
 *  class HogeThread : CyclicThread<HogeThread>{};
 *  
 *  のように，CyclicThreadBaseにテンプレート引数として自分を渡し，継承する．
 *  (後述の例のように，CyclicThread<HogeThread>に対してfriend指定が必要．)←不要？
 *  このスレッドクラスを定義した上で，呼び出し側スレッドで，
 *  以下のようにスレッドを作成，実行開始する．
 * 	 
 * 	HogeThread　hoge_th;
 *  
 *  hoge_th.start();	// スレッドを作成し，実行する．
 * 
 *  // 呼び出し側スレッドでのなんらかの処理をする．
 *  // その間に，ここで，作成したスレッドにて
 * 	// hoge_th.Initialize();
 *  // for(;;){
 * 	//	hoge_th.TimerProc();
 *  // }
 * 	// hoge_th.Terminate();
 *  // といった感じで自動的に実行される．
 * 
 *  hoge_th.stop_and_join();	// 周期実行を停止し，threadの終了を待つ
 * 
 * // ------------CyclicThreadBaseクラスを使うときのテンプレートを-----------
 * // ------------------------コピー用にまとめておきます---------------------
 * class HogeThread : CyclicThread<HogeThread>
 * {
 *		// friend RTCLib::CyclicThread<HogeHogeThread>; <- 不要？
 *		
 *		// 別スレッドstart時に最初に実行される関数．
 *		void Initialize( void );
 *		// 別スレッドに周期的に実行される関数．
 *		void TimerProc( void );
 *		// 別スレッドstop時に最後に実行される関数．
 *		void Terminate( void );
 * };
 * // --------------------------------- ここまで -------------------
 * 
 *  
 * ////////////////////////////////////////////////////////////////////////
 *  // 周期的に処理を実行するタイプのスレッド定義例.
 *  // UsersThreadPeriodicの部分は好きなお名前で．
 *  <main.cpp>
 * 
 *  class UsersThreadPeriodic 
 * 			: public CyclicThread<UsersThreadPeriodic>	// 自分自身をテンプレートとして渡し，
 * {													// それを継承する(CRTPと呼ばれる)のです．
 * 	// 
 * 	// 下記friend指定が必要になります．
 * 	// friend CyclicThread<UsersThreadPeriodic>;
 * 
 * 	// 構築と破壊．普通のクラスと同様に書くことができる．
 * 	// ただし，これらは呼び出し側スレッドが実行する準備であります．
 * 	UsersThreadPeriodic() : CyclicThread(100);	// ここでmsec秒単位で周期を指定(例は100msec)
 * 													// あるいは，set_intervalメソッドを用いる
 * 	virtual ~UsersThreadPeriodic();
 * 		
 * public :
 * 	// 下記，別スレッドで実行する処理を，publicとしてオーバーライドする．
 * 			
 * 	// 別スレッドが，最初に実行する準備用関数．
 * 	void Initialize()
 * 	{
 * 		cout << "Init user!" << endl;
 * 		moge.reset( new hoge );
 * 	}	
 * 
 * 	// 別スレッドが，タイマーで定期的に実行する準備用関数．
 * 	void TimerProc()
 * 	{
 * 		cout << "Timer user!" << endl;
 * 		Sleep(100);
 * 	}
 *  
 * 	// 別スレッドが，最後に実行する終了用関数．
 * 	void Terminate()
 * 	{
 * 		cout << "Term user!" << endl;
 * 	}
 * 
 * // なにか処理に必要なメンバ変数，関数も適宜持つことができる．
 * private :
 * 	int something;
 * 			
 * 	// void Run( void* user_data_type );だけは予約済み．
 * 	// 定期処理を行いたい場合はオーバーライドしないように注意．
 *  // CyclicThreadクラスでは，このRun内に周期実行手順が書かれている．
 * }
 * 
 *  // メインスレッドから，周期実行スレッドを実行する．
 *  void main(void)
 *  {
 * 		UsersThreadPeriodic utp;
 * 		utp.start();			// "Init user!" 
 * 		Sleep(1000);			// 1000msec休止より，約10回"Timer user!"が表示
 * 		utp.stop_and_join();	// "Term user!"
 *  }
*/

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/exception/all.hpp>

#include "ThreadSleep.h"

#if defined(_WIN32) || defined(_WIN64)
	//! WindowsのtimeSetEventを用いた似非リアルタイム周期タイマー
#include <tchar.h>
#include <Windows.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace RTCLib
{
	//! リアルタイム動作が確保できない場合の例外
	struct RTCTimerException : public boost::exception, public std::exception{};


	//! 単純にsleepで待つ周期タイマー
	struct WaiterSleep
	{
		int m_period_msec;
	public:
		WaiterSleep(int msec) : m_period_msec(msec){	};
		~WaiterSleep(){};
		
		bool wait(){
			sleep_msec(m_period_msec);
			return true;
		};
	};

#ifdef ART_ENABLE
	#include "linux/art_task.h"

	//! ART_linuxの機能を用いたリアルタイム周期タイマー
	template<int PRIO>
	struct WaiterArt
	{
		int m_period_msec;
	public:
		WaiterArt(int msec) : m_period_msec(msec)
		{
			int ret = art_enter( PRIO , ART_TASK_PERIODIC, msec*1000 );
			if(ret != 0)
			{
				BOOST_THROW_EXCEPTION( RTCTimerException() );
			}
		};
		~WaiterArt()
		{
			art_exit();
		};
		
		bool wait(){
			art_wait();
			return true;
		};
	};
#endif

#ifdef WIN32
	//! WindowsのtimeSetEventを用いた似非リアルタイム周期タイマー
	struct WaiterMmtimer
	{
		static const int timeout_rate = 100;

		int m_period_msec;
		HANDLE m_Wait_Event;
		MMRESULT m_tid;
	public:
		WaiterMmtimer(int msec) : m_period_msec(msec)
		{
			m_Wait_Event = CreateEvent(nullptr, false , false, nullptr);
			if(m_Wait_Event == INVALID_HANDLE_VALUE)
			{
				BOOST_THROW_EXCEPTION( RTCTimerException() );
			}
			m_tid = timeSetEvent( m_period_msec, 1, (LPTIMECALLBACK)m_Wait_Event, NULL, TIME_PERIODIC | TIME_CALLBACK_EVENT_SET);
		};
		~WaiterMmtimer()
		{
			::timeKillEvent( m_tid );
			CloseHandle( m_Wait_Event ); 
		};
		
		bool wait(){
			// wait for timer event; with timeout by (m_period_msec * timeout_rate) msec
			::WaitForSingleObject(m_Wait_Event,m_period_msec * timeout_rate);
			return true;
		};
	};
#endif

		// CRTPパターン．
	//! 周期実行スレッドを実現するための補助クラス
	template < typename UserThreadClass, typename UserDataType = void >
	class OneshotThread{

		// ユーザが直接実行するメソッド．構築，破壊，実行，中止関係
	public:

		// スレッドの実行メソッドに渡すユーザデータの型
		typedef UserDataType user_data_type;

		/// 構築
		OneshotThread()
		{
			//cout << "destruct thread_base_class;" << endl ;
		}

		/// 破壊
		virtual ~CyclicThread()
		{
			//Thread終了
			join();
			//cout << "destruct thread_base_class;" << endl ;
		}

		/// Run関数を別スレッドとして実行する．
		void start( user_data_type* pud = NULL )
		{
			boost::function< void ( void* ) > func =
				boost::bind(&UserThreadClass::Run, dynamic_cast<UserThreadClass*>(this) , boost::lambda::_1 );
			pth.reset( new boost::thread(func, (user_data_type*)pud ) );
		};

		/// スレッド終了まで待つ．
		void join( )
		{
			pth->join();
		}

		/// 状態しらべる系メソッド
		bool is_terminated()
		{
			return isTerminated;
		}

		bool is_suspended()
		{
			return isSuspended;
		}

		//void interrupt()
		//{
		//	pth->interrupt();
		//}

		// ユーザーがオーバーライドする関数群のベース．
		// オーバーライドしない場合のデフォルト動作を記述
	protected:

		//!  スレッド実行時，別スレッドにて実行される関数．
		/// スレッド作成時に別スレッドにて実行され，デフォルト（オーバーライドしない）では，
		/// (1) Initialize()を呼び出し，
		/// (2) stop()が呼ばれるまで周期的にTimerProc()を実行し，
		/// (3) Terminate()を実行する，
		/// という一連の動作を行う．
		///
		/// 周期実行を望まない場合，これをオーバーライドすると，
		/// オーバーライドしたRunにて記述した処理のみを実行できる．

		virtual void Run( user_data_type* udata ){
			// デフォルト実装では自動ループ実行される
			set_periodic( true );
			set_terminated( false );

			//(void)udata;
			cout << "Run!" << endl;
			

			// 初期化動作
			Initialize( udata );

			// ループ動作
			while( isPeriodic )
			{
				if( isSuspended)
				{	// サスペンド中はsleep
					pth->yield();
				}
				else
				{
					//if( wait_for_next_priod() ){
					if( m_pwait->wait() ){
						TimerProc( udata );
					}
				}
			}
			// リアルタイム動作の停止、あるいは周期実行条件の解除
			m_pwait.reset( static_cast<Waiter*>(0) );

			// 終了動作
			Terminate( udata );
			set_terminated( true );
		};

		//! スレッド実行時，別スレッドにて最初に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行される．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// 最初に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void Initialize( user_data_type* udata ){};

		//! スレッド実行時，別スレッドにて周期的に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行され，
		/// 次に設定された周期ごとにTimerProc()が呼ばれる．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// 周期的に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void TimerProc( user_data_type* udata ){};

		//! スレッド実行時，別スレッドにて最後に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行され，
		/// 次に設定された周期ごとにTimerProc()が呼ばれる．
		/// その後，stop(あるいはstop_and_join)が呼ばれ，周期処理が終了すると，
		/// このTerminate()が実行され，その後にスレッドが終了する．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// スレッド終了前に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void Terminate( user_data_type* udata ){};


	protected:
		// あとはユーザーはしーらないっと．
		boost::shared_ptr< boost::thread > pth;

		bool isTerminated;	// 周期実行モードで，Terminateが呼ばれましたよ
		
		// ミューテックス
		boost::mutex m_mutex;
		boost::condition_variable m_cond;

		// リアルタイム実行のための同期機構
		boost::shared_ptr<Waiter> m_pwait;

		// 次周期を待つ．次周期に来たらtrueを返す．
		// その他の原因で待機終了したらfalseを返す．
		bool wait_for_next_priod()
		{
			Sleep_msec( n_period_msec );
			return true;
		}

		// 実行状況フラグの設定．
		void set_periodic(bool stop_if_false)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isPeriodic = stop_if_false;
		}
		void set_suspend(bool suspend_if_true)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isSuspended = suspend_if_true;
		}
		void set_terminated(bool terminated_if_true)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isTerminated = terminated_if_true;
		}
		

	};


	// CRTPパターン．
	//! 周期実行スレッドを実現するための補助クラス
	template <	class UserThreadClass, //class UserData = void, 
				typename Waiter = WaiterSleep>
	class CyclicThread{

		// ユーザが直接実行するメソッド．構築，破壊，実行，中止関係
	public:

		// スレッドの実行メソッドに渡すユーザデータの型
		typedef void user_data_type;

		/// 構築
		CyclicThread(int interval = 1000)
			 : isPeriodic(false), isTerminated(false), 
				isSuspended(false), n_period_msec(interval)
		{
			//set_interval( interval );

			// リアルタイム動作の開始、あるいは周期実行条件の設定
			m_pwait.reset( new Waiter(n_period_msec) );

			//cout << "construct thread_base_class;" << endl ;
		}

		/// 破壊
		virtual ~CyclicThread()
		{
			//Thread終了
			stop_and_join();
			//cout << "destruct thread_base_class;" << endl ;
		}

		/// Run関数を別スレッドとして実行する．
		void start( user_data_type* pud = NULL )
		{
			boost::function< void ( void* ) > func =
				boost::bind(&UserThreadClass::Run, dynamic_cast<UserThreadClass*>(this) , boost::lambda::_1 );
			pth.reset( new boost::thread(func, (user_data_type*)pud ) );
		};

		/// 実行周期の変更
		void set_interval(int msec = 0)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			if(msec < 0)
				{	msec = 0;	isPeriodic = false;}
			n_period_msec = msec;
		}

		/// スレッドが周期実行形式の場合，サスペンド
		void suspend()
		{
			set_suspend(true);
		}

		/// スレッドが周期実行形式の場合，サスペンド後ならリジューム
		void resume()
		{
			set_suspend(false);
		}


		/// スレッドが周期実行中の場合，停止する．
		/// 停止信号を送るのみで，スレッドは待たない．
		void stop( )
		{
			set_periodic( false );
		}

		/// スレッド終了まで待つ．
		void join( )
		{
			pth->join();
		}

		/// スレッドが周期実行中の場合，停止する．
		/// 停止信号を送り，実際に終了するまで待つ．
		void stop_and_join( )
		{
			stop();
			pth->join();
		}

		/// 状態しらべる系メソッド
		bool is_terminated()
		{
			return isTerminated;
		}

		bool is_suspended()
		{
			return isSuspended;
		}

		//void interrupt()
		//{
		//	pth->interrupt();
		//}

		// ユーザーがオーバーライドする関数群のベース．
		// オーバーライドしない場合のデフォルト動作を記述
	protected:

		//!  スレッド実行時，別スレッドにて実行される関数．
		/// スレッド作成時に別スレッドにて実行され，デフォルト（オーバーライドしない）では，
		/// (1) Initialize()を呼び出し，
		/// (2) stop()が呼ばれるまで周期的にTimerProc()を実行し，
		/// (3) Terminate()を実行する，
		/// という一連の動作を行う．
		///
		/// 周期実行を望まない場合，これをオーバーライドすると，
		/// オーバーライドしたRunにて記述した処理のみを実行できる．

		virtual void Run( user_data_type* udata ){
			// デフォルト実装では自動ループ実行される
			set_periodic( true );
			set_terminated( false );

			//(void)udata;
			cout << "Run!" << endl;
			

			// 初期化動作
			Initialize( udata );

			// ループ動作
			while( isPeriodic )
			{
				if( isSuspended)
				{	// サスペンド中はsleep
					pth->yield();
				}
				else
				{
					//if( wait_for_next_priod() ){
					if( m_pwait->wait() ){
						TimerProc( udata );
					}
				}
			}
			// リアルタイム動作の停止、あるいは周期実行条件の解除
			m_pwait.reset( static_cast<Waiter*>(0) );

			// 終了動作
			Terminate( udata );
			set_terminated( true );
		};

		//! スレッド実行時，別スレッドにて最初に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行される．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// 最初に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void Initialize( user_data_type* udata ){};

		//! スレッド実行時，別スレッドにて周期的に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行され，
		/// 次に設定された周期ごとにTimerProc()が呼ばれる．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// 周期的に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void TimerProc( user_data_type* udata ){};

		//! スレッド実行時，別スレッドにて最後に実行される関数．オーバーライドすること．
		/// start()によってスレッドが起動された際，自動的にRunが実行され，
		/// (Runをオーバーライドしていなければ)最初にInitialize()が実行され，
		/// 次に設定された周期ごとにTimerProc()が呼ばれる．
		/// その後，stop(あるいはstop_and_join)が呼ばれ，周期処理が終了すると，
		/// このTerminate()が実行され，その後にスレッドが終了する．
		/// ユーザーのスレッドクラスにて，この関数をオーバーライドすることで，
		/// スレッド終了前に実行するべき処理を簡潔に記述できる．
		/// オーバーライドしない場合，ここでは何も処理をしない．
		virtual void Terminate( user_data_type* udata ){};


	protected:
		// あとはユーザーはしーらないっと．
		boost::shared_ptr< boost::thread > pth;

		bool isPeriodic;	// 周期実行モードで，周期実行中ですよ
		bool isTerminated;	// 周期実行モードで，Terminateが呼ばれましたよ
		bool isSuspended;	// 周期実行モードで，スレッド停止中(Timerprocを呼ばない)ですよ
		int n_period_msec;	// 周期実行する際の周期 

		// ミューテックス
		boost::mutex m_mutex;
		boost::condition_variable m_cond;

		// リアルタイム実行のための同期機構
		boost::shared_ptr<Waiter> m_pwait;

		// 次周期を待つ．次周期に来たらtrueを返す．
		// その他の原因で待機終了したらfalseを返す．
		bool wait_for_next_priod()
		{
			Sleep_msec( n_period_msec );
			return true;
		}

		// 実行状況フラグの設定．
		void set_periodic(bool stop_if_false)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isPeriodic = stop_if_false;
		}
		void set_suspend(bool suspend_if_true)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isSuspended = suspend_if_true;
		}
		void set_terminated(bool terminated_if_true)
		{
			boost::mutex::scoped_lock lk(m_mutex);
			isTerminated = terminated_if_true;
		}
		

	};


}

#endif //RTC_LIB_RTC_THREAD

