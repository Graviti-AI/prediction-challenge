#pragma once

#ifndef RTCLIB_DATA_STORAGE
#define RTCLIB_DATA_STORAGE


//! ///////////////////////////////////////////////////////////////////////////////////////
/// 
/// 自分のための，Realtime制御フレームワーク整備事業  2011 / Jun - 2011? / ??
///								Copyright @ 奥田裕之 2011 
/// 
/// 第一弾，データ保存のためのクラス
///
/// 要件：C++, STL, Boost C++ library
/// 
/// ///////////////////////////////////////////////////////////////////////////////////////
///  
/// //---------------- 使い方 ----------------------
/// 
/// // 保存したい構造体を定義する．これは自由に作成できるが，
/// // コピー可能でなくてはならない．管理が不明確なため，ポインタ型メンバはできるだけ持たないほうがよい．
/// struct DBUnit
/// {
/// 	int x;		// 組み込み型であれば問題ない
/// 	float y;		
/// 	double z;
///  double uvw[3];	// 配列もできるようになりますた
/// 	int id;
/// 	DWORD flag;
/// 	Hoge hoge;	// ユーザー型でも持てる．要件は後述.
/// };
/// 
/// // データベースを定義作成する．格納要素数を指定．
/// DataStorage<DBUnit, 1000>	db(0);					// ページサイズは1000，要素数は0. 要素数0は省略可
///														// このようにページサイズのみ指定し，空で作成してpush_back，を推奨する．
/// 
/// //DataStorage<DBUnit>	db(0);						// 省略時にはページサイズは1024．
/// //DataStorage<DBUnit, 0>	db(0);						// ページサイズを0にすると，reserveまでページサイズ決定を遅延可能．
///														// この場合最初のdb.reserve(2000)で，ページサイズを2000に設定できる．
///														// ただし，この時reserveが必須．でないとpush_backできない．お勧めしない．
///														// また，一度ページサイズが決まったら変更は不可能．
///
/// // DataStorage<DBUnit> db(2000, 1000);				// 格納する要素数と，ページングサイズを指定．
/// 														// 左であれば，要素数1000のページが2ページ，計2000点．
///														// データ追加時，足りない分は自動的に拡張されるが，
///														// 制御中の再確保を防ぐためにはあらかじめ確保しておく．
///
/// //DataStorage<DBUnit, 1000>	db(10000);				// 1024 x 10ページが確保(10240点分)，
///														// 要素数は10000に．db[9999]までアクセス可能
/// 
///
/// db.reserve( 10000 );									// 10000要素までの容量を確保．要素数は変更しない．vectorのreserveと同様．
///														// 10000/1000=10ページ分の容量を確保しておく．  
///
/// cout << db.size() << endl;	// この時点ではサイズは0．reserveしたのみだから... db[3000]などはアクセス不可
/// int i=-1;
/// while( ++i < 100 )		// 制御ループ等
/// {
///		DBUnit tmp;		// 保存対象となるデータを用意し...
/// 		tmp.x = ... ;	tmp.y = ... ;
///		tmp.hoge.x = ... ; 	tmp.hoge.y = ... ;
/// 
///		// データベースへのデータの追加を行う．ここで，要素はコピー渡しされる．
///		// 注意！ push_backのため，最初の一個は2001要素目に追加される．通常は db(0, 1000)などとして作成されたし
///		db.push_back( tmp );
/// }
/// 
/// // サイズ取得．push_backした数になるはず．
/// cout << "DB Size = " << db.size() << endl;
/// // 要素の読み取り．通常の構造体と同様にメンバのアクセスが可能．
/// cout << "DB[3] = " << db[3].x << endl;
/// // []演算子は参照を返すため，要素への書き込みもできる
/// db[4].x = 256;
/// db[4].flag = 0xff;
/// 
/// // データベースの保存は，あらかじめメンバを登録することで行う．
/// db.register_member( "x", &DBUnit::x );		// xメンバを保存対象として登録，csvに書き出すときのタグを"x"とする
/// db.register_member( "y",  &DBUnit::y );		//
/// db.register_member( "z", &DBUnit::z );		//
/// db.register_member( "id",  &DBUnit::id );	//
/// db.register_member( "HogeX, HogeY",  &DBUnit::hoge ); // ユーザ定義型も保存対象とできる．ただし条件が必要．後述．
/// db.register_member( "flg",  &DBUnit::flag );
/// db.register_member( "uvw[#]", &DBUnit::uvw );// 配列も登録可能．DBunitのuvwのような配列ならば，サイズが自動的に登録され，
///												// 要素数分だけ，自動的に出力されるようになる．上記のようなユーザー定義型も可．
///												// このとき，ラベルに "#" が含まれていれば，要素番号が代入され自動展開される．
///
/// // db.set_delimiter(", ");	// 数値列の区切り，デリミタをセット．デフォルトは ", " なので左は無意味．
/// //							// ","でスペースを詰めた出力，"\t"でタブ区切り(tsv)になる．
/// // 出力のイメージ．．．．
/// // x, y, z, id, HogeX, HogeY, flg, uvw[0], uvw[1], uvw[2] 
/// // 0, 1, 2, 3, 4, 5, 6, ... 
/// // ...
///
/// // データベースの書き出しを行う．ostreamを準備すれば書き出し可能．
/// // coutを渡せば画面出力，fstreamを渡せばファイル保存可能
/// fstream fso("test.csv", ios_base::out);	// fstreamを準備
/// db.print(fso);							// "test.csv"に保存
/// 
/// // データーベースの明示的な解放．しなくてもデストラクタでも解放はされる
/// db.clear();
///
/// 
/// //////////////////////////////////////
/// ///////////   おまけ    //////////////
/// //////////////////////////////////////
/// -----------------------------------------------------------------------------------------------
/// // メンバの登録を行うと，列を数字指定してデータの取得も可能．ただしstringstream経由での取得になるため，なかなか遅い．
/// int nxx = db.get_flexible( 3, 0);		// db[3]の0番目のデータ，すなわちdb[3].xの内容をintとして取得．
/// double dxx = db.get_flexible( 3, 1);		// ただし，この列は，register_memberした順に0から...となる．
/// // string sxx = db.get_flexible( 3, 2);	// db[3].z の中身を，stringとして取得...したいけど，型指定がいらないのは組み込み型のみ．
/// 
/// // ...将来的には以下のように型指定に基づいて，型が一致すれば高速に取得できるようにしたいが，現状，上と同じ実装のため遅い！
/// int nxx = db.get_flexible<int>( 3, 0);		// 
/// double dxx = db.get_flexible<double>( 3, 1);	// 
/// string sxx = db.get_flexible<string>( 3, 2);	// 構造体やクラスの場合は，型を指定することで変換できる．
///
/// -----------------------------------------------------------------------------------------------
/// // 必要があれば保存するメンバの登録情報を削除できる．
/// // メンバの登録(register_member)，データ保存(print())，登録メンバ情報の削除(clear_member_registration)，
/// // 再び登録，データ保存．．．とすれば，
/// // ひとつ目のファイルにはx,y,z，二つ目にはid,hoge,flagなど，異なるデータ列を分けて保存可能．
/// // もちろんこのとき，データを入れなおす必要はない．出力するぞ～っていう情報のみ取り扱う．
/// db.register_member( "x", &DBUnit::x );
/// db.register_member( "y", &DBUnit::y );
/// db.print(cout);	// x,yのみ出力
/// db.clear_member_registration();	//メンバの登録情報をクリア
///
/// db.register_member( "id", &DBUnit::id );
/// db.register_member( "x", &DBUnit::x );
/// db.register_member( "z", &DBUnit::z );
/// db.print(cout);	// id, x, zを出力
/// db.clear();	//保存が終わったのでデータをクリア
/// 
/// -----------------------------------------------------------------------------------------------
/// // ユーザー定義型のメンバも保持することが可能．保持するたけならばコピー可能ならＯＫ．
/// struct Hoge
/// {
///  	int x;
///  	int y;
/// };
/// // ただし，たとえば以下のようにostreamへの出力を定義しておかないと
/// // 出力メンバに登録することができない．
/// ostream& operator<<(ostream& os, Hoge& h)	//この宣言がないと，register_memberした時点でコンパイルエラー
/// {
///  	os << h.x << "," << h.y;	// ここで，最後のカンマは不要なことに注意．自動的に挿入される．
///  	return os;					// 連結するための，ostreamへの参照が必要．
/// }
///

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
//#include <algorithm>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#ifdef __WIN32__ || WIN32
#define snprintf _snprintf
#endif

namespace RTCLib{

	using std::vector;
	using boost::shared_ptr;
	using std::ostream;

	// 配列用のラベルを生成する補助関数．ラベル中に#があったら自動展開
	template<class T>
	std::string numbered_label(std::string label, std::string delim, size_t s, size_t e);

	template <class T, int PageSize = 1024>
	class DataStorage : boost::noncopyable
	{

	public:
		// コンストラクタ．ページサイズはここでのみ設定できる．
		DataStorage(size_t size = 0);
		// デストラクタ．Clearしましょう
		~DataStorage(void);

		// 容量を確保．要素数は変更せず．vectorのreserveと同じ？
		void reserve(size_t reserve_num);
		// 保持要素数を変更．[]でアクセスできる数．
		void resize(size_t resize_num);

		// 内容を削除．イメージはvector
		void clear();

		// 要素参照
		T& operator[](size_t n) const;
		T& get(size_t n) const;

		// 格納済み要素数を取得
		inline size_t size() const {return m_size;}

		// 要素を追加．自動的にページング，インデックスを計算し，不足していればページを拡張する．
		size_t push_back(const T &in);

		// デリミタをセット．書き出しのときだけね...
		void set_delimiter(std::string delimiter);

		// 定義構造体の保存すべきメンバを登録
		template <class S>
		void register_member( std::string label, S T::*anyp );
		// 配列のメンバを登録．
		template <class S, size_t N>
		void register_member( std::string label, S (T::*anyp)[N] );
		// vector配列のメンバを登録．
		template <class S>
		void register_member( std::string label, vector<S> (T::*anyp) );

		// ポインタ型のメンバを登録．←特殊化で実体化できないようにしたいなぁ…
		//template <class S>
		//void register_member( std::string label, S T::*anyp );

		// 保存メンバ情報をクリア
		void clear_member_registration();

		// ストリームを指定して内容を出力
		void print(ostream &fso, size_t start = 0, size_t end = -1 ) const;
		void printheader(ostream &fso) const;
		void print_one_row(ostream &fso, size_t index) const;
		
		// ストリーム変換を利用してデータを取得. ###  ただし，add_print_memberしたもののみ探索  ###
		// さらに，文字列型を経由するため，精度の低下に注意．
		template<class S>
		S get_flexible(size_t row, size_t column);

		// 柔軟出力用オブジェクト... 型キャストを定義		
		struct auto_caster
		{
			size_t m_row;
			size_t m_col;
			DataStorage& m_db;
			auto_caster( DataStorage& db, size_t row, size_t col ) : m_db(db), m_row(row), m_col(col){};
			// 各型への自動変換．いろいろ書いておくと自動的に換えてくれますよ…
			operator char(){return m_db.get_flexible<char>(m_row, m_col);	};
			operator short(){return m_db.get_flexible<short>(m_row, m_col);	};
			operator int(){return m_db.get_flexible<int>(m_row, m_col);	};
			operator long(){return m_db.get_flexible<long>(m_row, m_col);	};
			operator long long(){return m_db.get_flexible<long long>(m_row, m_col);	};
			operator unsigned char(){return m_db.get_flexible<unsigned char>(m_row, m_col);	};
			operator unsigned short(){return m_db.get_flexible<unsigned short>(m_row, m_col);	};
			operator unsigned int(){return m_db.get_flexible<unsigned int>(m_row, m_col);	};
			operator unsigned long(){return m_db.get_flexible<unsigned long>(m_row, m_col);	};
			operator unsigned long long(){return m_db.get_flexible<unsigned long long>(m_row, m_col);	};

			operator float(){return m_db.get_flexible<float>(m_row, m_col);	};
			operator double(){return m_db.get_flexible<double>(m_row, m_col);	};
			operator std::string(){return m_db.get_flexible<std::string>(m_row, m_col);	};
		};

		auto_caster get_flexible(size_t row, size_t column){return auto_caster(*this, row, column);	};

	private:
		vector<shared_ptr<vector<T> > > m_db;
		size_t m_size;	//	保持しているデータ数
		size_t m_page_size;	// ページサイズ
		size_t m_capacity;	// 確保済み容量

		// CSVに保存対象となるメンバ名と，アドレス
		std::vector<std::string> m_tags;

		// addmemberするたびに，出力用関数オブジェクトを追加
		vector<boost::function<void (T&, ostream&)> > m_funcs;
		vector<boost::function<void (T&, ostream&)> > m_tag_funcs;

		// デリミタ．通常はcsvとして", "を用いよう...
		static std::string m_delimiter;

		/////////////////////////////////以下privateメソッド//////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////

		// 配列番号から，ページ番号＋インデックスを計算. 要素外チェックは無し
		inline boost::tuple<size_t, size_t> lin2sub(size_t index) const;

		// 容量増強のためにページを追加．確保のみで要素サイズは不変
		void add_page(size_t adding_size);

		///////////////////////////////////////////////
		/////     数値出力用関数オブジェクト      /////
		///////////////////////////////////////////////

		// 出力用関数オブジェクト. ope()は数値，print_tagはタグを出力
		template <class S>
		struct stream_outputter
		{
			typedef S type;
			explicit stream_outputter( std::string label,  S T::*anyp )
				: mem_func(anyp), m_label(label){};
			S T::*mem_func;
			std::string m_label;
			void operator()( T& trg, ostream &os ){
				os << trg.*mem_func;
			}
			void print_tag( T& trg, ostream &os ){
				os << m_label;
			}
		};

		template <>
		struct stream_outputter<double>
		{
			typedef double type;
			explicit stream_outputter( std::string label,  double T::*anyp )
				: mem_func(anyp), m_label(label){};
			double T::*mem_func;
			std::string m_label;
			void operator()( T& trg, ostream &os ){
				//char buf[128];
				//_snprintf(buf, sizeof(buf), "%d", trg.*mem_func);
				//os << buf;
				os << trg.*mem_func;
			}
			void print_tag( T& trg, ostream &os ){
				os << m_label;
			}
		};

		// 配列に対する関数オブジェクト．配列は配列分だけプリントしましょう．
		template <class S, size_t N>
		struct stream_outputter< S[N] >
		{
			typedef S type;
			explicit stream_outputter( std::string label, S (T::*anyp)[N] ) : mem_func(anyp), m_label(label){};
			S (T::*mem_func)[N];
			std::string m_label;
			void operator()( T& trg, ostream &os ){
				os << (trg.*mem_func)[0];
				for(int i=1; i < N; ++i)
				{	os << m_delimiter << (trg.*mem_func)[i];	}
			}
			void print_tag( T& trg, ostream &os ){
				os << numbered_label<void>( m_label, m_delimiter, 0, N-1);
			}
		};

		// vectorに対する関数オブジェクト．vectorは配列数分だけプリントしましょう．
		template <class S>
		struct stream_outputter< vector<S> >
		{
			typedef S type;
			explicit stream_outputter( std::string label, vector<S> (T::*anyp) ) : mem_func(anyp), m_label(label){};
			vector<S> (T::*mem_func);
			std::string m_label;
			void operator()( T& trg, ostream &os ){
				if( 0==(trg.*mem_func).size() )return;
				os << (trg.*mem_func)[0];
				for(size_t i=1; i < (trg.*mem_func).size(); ++i)
				{	os << m_delimiter << (trg.*mem_func)[i];	}
			}
			void print_tag( T& trg, ostream &os ){
				if((trg.*mem_func).empty())return;
				os << numbered_label<void>( m_label, m_delimiter, 0, (trg.*mem_func).size()-1 );
			}
		};
	};

	////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////以下，実装部//////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////

	// コンストラクタ．ページサイズはここでのみ設定できる．
	template<class T, int PageSize >
	DataStorage<T, PageSize>::DataStorage(size_t size)
		: m_size(0), m_page_size(PageSize), m_capacity(0)
	{
		if(size != 0)
			resize(size);
		//else
		//	add_page(0);	//　サイズ0のページだけ追加しておく
	};

	// デストラクタ．
	template<class T, int PageSize >
	DataStorage<T, PageSize>::~DataStorage(void)
	{
		clear();	// 本来は明示的呼び出しが不要ですが…
	}

	template<class T, int PageSize >
	void DataStorage<T, PageSize>::clear()
	{
		m_db.clear();
		m_size = 0;
		m_capacity = 0;		
	}

	// デリミタの初期化子
	template<class T, int PageSize >
	std::string DataStorage<T, PageSize>::m_delimiter = ", ";

	// デリミタをセット．書き出しのときだけね...
	template<class T, int PageSize >
	void DataStorage<T, PageSize>::set_delimiter(std::string delimiter)
	{
		m_delimiter = delimiter;
	}

	// 配列番号から，ページ番号＋インデックスを計算. 要素外チェックは無し
	template<class T, int PageSize >
	inline boost::tuple<size_t, size_t> DataStorage<T, PageSize>::lin2sub(size_t index) const 
	{
		if(m_page_size==0 || m_db.size()==1 )
			return boost::make_tuple< size_t, size_t >( 0, index );
		size_t page_num = index / m_page_size;
		size_t page_index = index % m_page_size;
		return boost::make_tuple< size_t, size_t >( page_num, page_index );
	}

	// 配列演算子によるアクセス．
	template<class T, int PageSize >
	T& DataStorage<T, PageSize>::operator[](size_t n) const {
		return get(n);
	}

	// n番目のデータにアクセス
	template<class T, int PageSize >
	T& DataStorage<T, PageSize>::get(size_t n) const {
		size_t page;
		size_t index;
		if(n >= size() )
		{
			std::cout << "database subscript out of range, data in index:" 
						<< size() << "is returned." << std::endl;
			n = size()-1;
		}
		boost::tuples::tie< size_t, size_t>(page, index)
			= lin2sub( n );
		return (*m_db[page])[index];
	}

	// 予備容量を確保
	template<class T, int PageSize >
	void DataStorage<T, PageSize>::reserve(size_t reserve_num)
	{
		//// ページサイズが異なれば，一度削除を行う． 
		//if(m_page_size != page_size){
		//	std::cout << "Paging size is changed. Data is cleared." << std::endl;
		//	clear();
		//}
		size_t page_num = 0;
		if ( m_page_size == 0 )
		{	// ページングをしない
			page_num = 1;
			m_page_size = reserve_num;	// 
		}
		else
		{	// ページングする
			// ページ数は・･･･
			page_num = (reserve_num+(m_page_size-1)) / m_page_size;
		}

		for(size_t i=0; i<page_num; i++)
		{
			add_page( m_page_size );
		}
	}

	// 要素数を変更
	template<class T, int PageSize >
	void DataStorage<T, PageSize>::resize(size_t resize_num)
	{
		//cout << "Call resize..." << endl;
		// 保持データ数を変更
		m_size = 0;
		size_t page = 0;
		m_capacity = 0;
		while( resize_num > 0 )	//確保すべき残数が0になるまでページを確保
		{
			size_t num_in_page = 0;	//今回追加するサイズ
			if( resize_num <= m_page_size )
			{	// ページサイズより，必要な数が少ない
				num_in_page = resize_num;
			}
			else
			{	// ページサイズより，必要な数が多い
				num_in_page = m_page_size;
			}

			if( page < m_db.size() )
			{	// ページが確保できまっせ
				m_db[page]->resize( num_in_page );
			}
			else//ページ数がたりませんわ．ページも追加しますわ
			{
				boost::shared_ptr< vector<T> > tmp = boost::shared_ptr< vector<T> >(new vector<T>( ));
				tmp->reserve( m_page_size );
				tmp->resize( num_in_page );
				m_db.push_back( tmp );
			}
			resize_num -= num_in_page;	// 残りの確保数を減少
			m_size += num_in_page;
			m_capacity += m_page_size;
			page++;
		}
		m_db.resize(page);
	}

	// データベースに要素を追加
	template<class T, int PageSize>
	size_t DataStorage<T, PageSize>::push_back(const T &in)
	{
		while (m_size >= m_capacity)
		{	// Capacityをオーバーしている…ページを追加．
			if(m_page_size > 0)	// ページングをしている
			{
				add_page(m_page_size);
			}else{				// ページングしていない
				m_page_size++;
			}
		}

		// 次の一個のページとサイズを取得
		size_t page, index;
		boost::tuples::tie< size_t, size_t>(page, index)
			= lin2sub( m_size );
		// 該当ページに要素を挿入
		m_db[page]->push_back( in );
		// 参照用要素数を増加			
		//m_size++;
		return m_size++;
	}

	// ページを追加，容量を追加で確保する．追加ページ容量も指定可能．
	template<class T, int PageSize >
	void DataStorage<T, PageSize>::add_page(size_t adding_size)
	{
		boost::shared_ptr< vector<T> > tmp = boost::shared_ptr< vector<T> >(new vector<T>());
		tmp->reserve( adding_size );
		m_db.push_back( tmp );
		m_capacity += adding_size;
		//cout << "add_page:" << adding_size << endl;info();
	}

	/////////////////////////////////////////////////////////////////////////////////////////
	// ------------   メンバを登録する部分．実装がちょっと複雑ね・・・   --------------------
	/////////////////////////////////////////////////////////////////////////////////////////

	// メンバを登録する.必要な処理が増えればここにfunctorと一緒に追加するのかな？？
	template <class T, int PageSize >	template <class S>
	void DataStorage<T, PageSize>::register_member( std::string label, S T::*anyp )
	{
		//printf("%s(%s) : %d\n",label.c_str(), typeid(S).name() ,anyp );
		stream_outputter<S> tmp( label, anyp );
		m_funcs.push_back( boost::function<void (T&, ostream&)>( tmp ) );
		m_tag_funcs.push_back( boost::function<void (T&, ostream&)>( 
			boost::bind( &stream_outputter<S>::print_tag, tmp, _1, _2 ) ) );

		m_tags.push_back(label);
	}

	//// vectorのメンバを登録
	template <class T, int PageSize >	template <class S>
	void DataStorage<T, PageSize>::register_member( std::string label, vector<S> T::*anyp )
	{
		//label = duplicate_label(label, N);
		//printf("%s(%s)[%d] : %x\n", label.c_str(), typeid(S).name(), N ,anyp );
		stream_outputter< vector<S> > tmp( label, anyp );
		m_funcs.push_back( boost::function<void (T&, ostream&)>( tmp ) );
		m_tag_funcs.push_back( boost::function<void (T&, ostream&)>( 
			boost::bind( &stream_outputter<vector<S> >::print_tag, tmp, _1, _2 ) ) );

		m_tags.push_back( label );
	}

	//// 配列のメンバを登録．
	template <class T, int PageSize >	template <class S, size_t N>
	void DataStorage<T, PageSize>::register_member( std::string label, S (T::*anyp)[N] )
	{
		//label = duplicate_label(label, N);
		//printf("%s(%s)[%d] : %x\n", label.c_str(), typeid(S).name(), N ,anyp );
		stream_outputter<S[N]> tmp( label, anyp );
		m_funcs.push_back( boost::function<void (T&, ostream&)>( tmp ) );
		m_tag_funcs.push_back( boost::function<void (T&, ostream&)>( 
			boost::bind( &stream_outputter<S[N]>::print_tag, tmp, _1, _2 ) ) );

		m_tags.push_back(label);
	}

	// 保存メンバ情報をクリア
	template <class T, int PageSize >
	void DataStorage<T, PageSize>::clear_member_registration( )
	{
		m_tags.clear();
		m_funcs.clear();
		m_tag_funcs.clear();
	}

	// ストリームに出力
	template <class T, int PageSize >
	void DataStorage<T, PageSize>::print(ostream &fso, size_t start = 0, size_t end = -1 ) const {
		// ヘッダ出力
		//for(size_t i = 0; i < m_tags.size(); i++ )
		//{
		//	fso << m_tags[i] << m_delimiter;
		//}

		if(end == -1)end = size();

		ios_base::sync_with_stdio(false);


		for(size_t i = 0; i < m_tag_funcs.size(); i++ )
		{
			//fso << m_tags[i] << m_delimiter;
			m_tag_funcs[i]( get(0), fso );
			fso << m_delimiter;
		}
		fso << std::endl;

		// 中身の出力
		//string str_buf;
		//str_buf.reserve(100000);
		//std::stringstream ss(str_buf);
		//std::ostream& ss = fso;

		if(m_funcs.size() > 0)
		{

			for( size_t j=start; j < end; j++)
			{
				//str_buf.clear();
				for(size_t i = 0; i < m_funcs.size(); i++ )
				{
					T& tmp = get(j);
					m_funcs[i]( tmp , fso);
					fso << m_delimiter;
				}
				//std::cout << ss.str();
				fso << std::endl;
				//fso << ss.str();
			}
		}
		//fso << ss.str();
	}
	
		// ストリームに出力
	template <class T, int PageSize >
	void DataStorage<T, PageSize>::printheader(ostream &fso) const {
		// ヘッダ出力
		//for(size_t i = 0; i < m_tags.size(); i++ )
		//{
		//	fso << m_tags[i] << m_delimiter;
		//}
		for(size_t i = 0; i < m_tag_funcs.size(); i++ )
		{
			//fso << m_tags[i] << m_delimiter;
			m_tag_funcs[i]( get(0), fso );
			fso << m_delimiter;
		}
		fso << std::endl;
	}

		// ストリームに出力
	template <class T, int PageSize >
	void DataStorage<T, PageSize>::print_one_row(ostream &fso, size_t index) const {
		// 中身の出力
		std::stringstream ss;

		for(size_t i = 0; i < m_funcs.size(); i++ )
		{
			T& tmp = get(index);
			m_funcs[i]( tmp , ss);
			ss << m_delimiter;
		}
		//std::cout << ss.str();
		ss << std::endl;
		fso << ss.str();
		ss.str("");
	}


	// ストリーム変換を利用してデータを取得
	template<class T, int PageSize > template<class S>
	S DataStorage<T, PageSize>::get_flexible(size_t row, size_t column)
	{
		if( row >= size() )return S();
		if( column >= m_funcs.size() )return S();
		T& trg = get(row);

		std::stringstream ss;
		m_funcs[column](trg, ss);
		S tmp;
		ss >> tmp;
		return tmp;
	}

	// 配列用のラベルを生成する補助関数．ラベル中に#があったら自動展開
	// cppじゃなくヘッダに書いちゃいたいからダミーテンプレートにしときますｗ
	template<class T>
	std::string numbered_label(std::string label, std::string delim, size_t s, size_t e)
	{
		size_t i = label.find( '#' );
		if( i == label.npos )	// 置換すべきナンバーが無い
			return label;

		std::string ret = "";
		for(size_t j=s; j<=e; j++)	// 文字列中に#があった場合，自動展開
		{
			std::string tmp = label;
			std::string numstr = boost::lexical_cast<std::string, size_t>(j);
			
			while( true ){
				size_t ind = tmp.find( '#' );
				if(ind==tmp.npos)break;
				tmp.replace( ind, 1, numstr);
			}

			ret = ret + tmp;
			if(j!=e)ret = ret + delim;
		}
		return ret;
	}

}

#endif //RTCLIB_DATA_STORAGE
