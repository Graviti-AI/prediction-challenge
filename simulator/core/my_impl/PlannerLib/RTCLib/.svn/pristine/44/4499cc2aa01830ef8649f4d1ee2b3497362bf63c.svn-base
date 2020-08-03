#pragma once

#ifndef RT_CTRL_FRAMEWORK_STOP_WATCH
#define RT_CTRL_FRAMEWORK_STOP_WATCH



#include "timGTD.h"	// for linux...
#include "timQPC.h"	// QueryPerformanceCounter
#include "timRDTSC.h"	// RDTSC intrinsic counter

namespace RTCLib
{
	 /*!   
	 *  \brief
	 *  RTControlFramework用タイマー．
	 *
	 *  各種タイマーを用いて，ストイップウオッチ機能を実現．
	 *  タイマ計測は他の機能にゆだねる...
	 *  ただし，splitやlapの取得データの保存機能は無し．
	 *  同一制御ループ内で，同じ計測データを用いるため，get_last_measuredを用意．
	 *  戻り値は秒．lap_in_ms, split_in_ms等ではmsec単位で取得．
	 *  組み込むTimerに応じて精度が変わる．
	 * 
	 * 現在，タイマーとしては，QPCTimer (timQPC.h)，RDTSCTimer (timRDTSC.h)を用意．
	 * 
	 * **** 使い方 ****
	 * StopWatch<QPCTimer> sw;			// QPCTimerを使ったストップウオッチ
	 * sw.start();						// ストップウオッチをスタート
	 * double split = sw.split();		// Splitを取得．split→スタートからの経過時間を取得，
	 * 								// 制御開始時刻からの経過時間の取得などに．
	 * double split_ms = sw.split_in_ms();	//スプリットタイムをmsec単位で取得
	 * 
	 * double lap = sw.lap();			// 開始時刻，あるいは最後にLapした時刻からの経過時間を取得，
	 * 								// いわゆるラップタイム．制御周期の確認などに．
	 * double lap_ms = sw.lap_in_ms();	//ラップタイムをmsec単位で取得
	 * 
	 * double hoka = sw.get_last_measured();	// 最後にLapまたはSplitで取得した計測時間をもう一度取得．
	 * 										// 複数の変数に，経過時間をいれたい場合などに．
	 * double hoka_ms = sw.get_last_measured_in_ms();	// 上記のmsec版．sec単位とmsec単位を保存したい時等に．
	 * 
	 * 
	 * *************** 他のタイマーを使う場合 *******************
	 * 下記の要件がpublicとして公開されていれば，StopWatch用の計測タイマとして使用できる！
	 * 将来的にHPET等を使ったタイマーへと展開したい場合は以下を実装しておくこと．
	 * 
	 * Timerの実装要件：
	 * 
	 * Timer::timer_value								->	タイマー格納用データ型をtypedefしておくこと．
	 * Timer::timer_value Timer::get_frequency( )		->	タイマーの分解能を取得. 1秒あたりに増える数値. value/secの次元
	 * Timer::timer_value Timer::get_time( )			->  タイマーの現在値を取得．
	 */

	//! OSにより自動的にスイッチ
#ifdef WIN32
	typedef QPCTimer RecommendedTimer;
#endif
#ifdef __linux__
	typedef GTDTimer RecommendedTimer;
#endif
	
	template<class Timer = RecommendedTimer>
	class StopWatchWith
	{

	public:

		/// コンストラクタ
		StopWatchWith( double unit = 1.0) : _target_unit(unit), _lat_measurement(0.0)
		{
			_freq = _timer.get_frequency( );
			_begin = _timer.get_time( );
			_end  = _begin;
			_last_for_lap = _begin;
			_is_started = false;
		}

		/// デストラクタ
		~StopWatchWith(void)
		{
		}

		/// タイマー開始
		void start()
		{
			if(_is_started)return;
			_is_started = true;
			_begin = _timer.get_time( );
			_last_for_lap = _begin;
			_end = _begin;
		}

		/// タイマー停止
		double stop()
		{
			_is_started = false;
			return split();
		}

		/// ゼロリセットからの再スタート
		double reset()
		{
			double ret = stop();
			start();
			return ret;
		}

		/// split : startからの経過時間(秒)を取得する(sec)
		double split()
		{
			// 計測と計算
			_lat_measurement = calc_elapsed_time( _begin, _timer.get_time() );
			return _lat_measurement;
		}

		/// split_in_ms : startからの時間(ミリ秒)を取得する(msec)
		double split_in_ms()
		{
			return split()*1000.0;
		}

		/// lap : startからの時間時間(秒)を取得して、時間をリセット
		double lap()
		{
			// 計測と計算
			value_type now = _timer.get_time();
			_lat_measurement = calc_elapsed_time( _last_for_lap, now );
			// リセット
			_last_for_lap = now;
			return _lat_measurement;
		}
		/// lap_in_ms : startからの時間時間(ミリ秒)を取得して、時間をリセット
		double lap_in_ms()
		{
			return lap()*1000.0;
		}

		/// 最後に取得した経過時間をもう一度取得(秒)
		/// 
		/// 計測時間を参照する場合に再計測しずに
		/// 値を参照する。複数回時間を参照したいときに使う。
		double get_last_measured(){
			return _lat_measurement;
		}

		/// 最後に取得した経過時間をもう一度取得(ミリ秒)
		/// 
		/// 計測時間を参照する場合に再計測しずに
		/// 値を参照する。複数回時間を参照したいときに使う。
		double get_last_measured_in_ms()
		{
			return get_last_measured()*1000.0;
		}

	private:
		typedef typename Timer::timer_value value_type;
		Timer _timer;				// 計測用タイマのインスタンス

		value_type _begin;	// タイマ開始時刻
		value_type _end;	// Stopした時刻(未使用)
		value_type _last_for_lap;	// 最後に計測した時刻
		value_type _freq;	// 1sec当たりのティック数. 

		double _target_unit;		// このストップウオッチが戻すべき単位時間．1なら秒．0.001ならmsec
		double _lat_measurement;	// 最後に取得した時間. 最後の取得とまったく同じ時間がほしいときに

		bool _is_started;

		double calc_elapsed_time(value_type st, value_type ed)
		{
			double elapsed;
			elapsed = (ed - st) / (double)_freq / _target_unit; 
			return elapsed;
		}
	};

#ifdef WIN32
	typedef StopWatchWith< QPCTimer > StopWatchQPC;
	typedef StopWatchWith< QPCTimer > StopWatchAuto;
#endif
#ifdef __linux__
	typedef StopWatchWith< GTDTimer > StopWatchGTD;
	typedef StopWatchWith< GTDTimer > StopWatchAuto;
#endif

	typedef StopWatchAuto StopWatch;

	}

#endif //RT_CTRL_FRAMEWORK_STOP_WATCH

