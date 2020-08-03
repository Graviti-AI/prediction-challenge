#pragma once

#ifndef RTC_LIB_FUSION_CSV
#define RTC_LIB_FUSION_CSV

/*!
* ///////////////////////////////////////////////////////////////////////////////////////
* 
* 自分のための，Realtime制御フレームワーク整備事業  2011 / Jun - 2011? / ??
*								Copyright @ 奥田裕之 2011 
* 
* Boost::fusionを使ったCSV読み込みクラス.
* 詳細はFusionCSVSample.cppを参照．
*
* 要件：C++, STL, Boost C++ library
* 
* ///////////////////////////////////////////////////////////////////////////////////////
*  
* //---------------- 使い方 ----------------------
* 
* // まずはヘッダのinclude
* #include <FusionCSV.h>
* 
* // 構造体を宣言＋同時にFusionに登録する．
* //-------------------------------------------
* // BOOST_FUSION_DEFINE_STRUCT has the same effect as struct definition and BOOST_FUSION_ADAPT_STRUCT.
* BOOST_FUSION_DEFINE_STRUCT(
*         // マクロ第一引数にネームスペースを登録．
*         // Globalな場合は以下のようにする．
*         BOOST_PP_EMPTY(), 
*         Data,						// 構造体名
*         (std::string, namae)		// 以下，登録するメンバの型とメンバ名を追加
*         (int, wanryoku)
*         (double, kiyosa)
*         (int, subayasa)
*         (int, tairyoku)
*         (int, maryoku)
*         (int, seishin)
*         (int, miryoku)
* );
*
* //--------------------------------------------
* // CSVからの読み込みは...
* //
* // まずはクラスを定義．テンプレートに先に登録したものを追加
* RTCLib::FusionCSVLoader< Data > db;
*	
* // CSVから読み込み．CSVには，ラベルにメンバ名が書かれていること．
* // ただし大文字小文字は区別する．
* db.Load( "data.csv" );
*
* // データベースは通常のvector<Data>なので追加等の操作もできる．
* Data tmp;
* db.m_data.push_back(tmp);
*
* // データベースの中身をCSVに出力する．
* db.Write("data2.csv" );
*	
*
*/



#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#pragma warning (disable: 4503)
#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/define_struct.hpp>
 

namespace RTCLib { namespace spirit {
 
        // Dummy type for csv separator
        // Using to make use of original meta_create_sequence implementation
        struct csv_separator {};
 
        namespace detail {
 
                // extracted from boost/spirit/home/support/auto/meta_create.hpp (v1.46)
                template <typename T>
                struct is_fusion_sequence_but_not_proto_expr
                  : boost::mpl::and_<
                                boost::fusion::traits::is_sequence<T>
                          , boost::mpl::not_<boost::proto::is_expr<T> > >
                {};
        }
 
}}
 
namespace boost { namespace spirit { namespace traits {
 
        // deep_copy may be unnecessary...
 
        // specialization for csv_separator
        template<>
        struct create_parser<RTCLib::spirit::csv_separator>
        {
                typedef proto::result_of::deep_copy<
                        BOOST_TYPEOF(qi::lit(','))
                >::type type;
 
                static type call()
                {
                        return proto::deep_copy(qi::lit(','));
                }
        };
 
        // specialization for std::string
        template<>
        struct create_parser<std::string>
        {
                typedef proto::result_of::deep_copy<
                        BOOST_TYPEOF(*(qi::char_ - qi::lit(',')))
                >::type type;
 
                static type call()
                {
                        return proto::deep_copy(*(qi::char_ - qi::lit(',')));
                }
        };
 
        // specialization for Fusion Sequence
        // meta_create_sequence with slight modification
        template <typename Sequence>
        struct create_parser<Sequence, typename boost::enable_if<
                                RTCLib::spirit::detail::is_fusion_sequence_but_not_proto_expr<Sequence> 
                        >::type>
        {
 
                // create a mpl sequence from the given fusion sequence
                typedef typename mpl::fold<
                        typename fusion::result_of::as_vector<Sequence>::type
                  , mpl::vector<>, mpl::push_back<mpl::push_back<mpl::_1, mpl::_2>, RTCLib::spirit::csv_separator>
                >::type sequence_type_;
                typedef typename mpl::if_<mpl::empty<sequence_type_>, sequence_type_, typename mpl::pop_back<sequence_type_>::type>::type sequence_type;
 
                typedef make_nary_proto_expr<
                        sequence_type, proto::tag::shift_right, qi::domain
                > make_proto_expr;
 
                typedef typename make_proto_expr::type type;
 
                static type call()
                {
                        return make_proto_expr::call();
                }
        };
 
 
}}}


namespace RTCLib{

	template<typename Attr>
	void parse_line(const std::string &s, Attr &attr)
	{
		namespace qi = boost::spirit::qi;

		typedef std::string::const_iterator Iterator;
		Iterator first = s.begin(), last = s.end();

		qi::phrase_parse(first, last, qi::auto_, qi::space, attr);
		//boost::fusion::out(std::cout, attr); // output as fusion sequence
		//std::cout << std::endl;
	}

	//! boost::fusionを使った超簡単CSVローダー
	template<typename Attr>
	class FusionCSVLoader
	{
	public:
		FusionCSVLoader(){};
		virtual ~FusionCSVLoader(){};

		std::vector<Attr> m_data;

		int Load(std::string fn, int skip_line = 0)
		{
			std::ifstream ifs(fn);
			std::string line;
			for(;skip_line>0;skip_line--)
			{
				std::getline(ifs, line);
			}

			while(!ifs.eof())
			{
				std::getline(ifs, line);

				if(line.empty() || trimstring(line) == "\n")
				{
					continue;
				}

				Attr tmp;
				parse_line(line, tmp);
				m_data.push_back( tmp );
			}
			return 0;
		}

		int Write(std::string fn, std::string tag = "" )
		{
			std::ofstream ofs(fn);
			if( !tag.empty() ) ofs << tag << std::endl;

			std::stringstream ss;

			for each( const Attr &tmp in m_data)
			{
				boost::fusion::for_each(tmp, writer(ss));
				//boost::fusion::out(ofs, tmp); // output as fusion sequence
				ss << std::endl;
				std::string line;
				getline(ss, line);
				ofs << trimlastcomma(line);
				ofs << std::endl;
			}
			return 0;
		}

	private:
		struct writer
		{
		public:
			writer( std::ostream &os ) : m_os(os){};
			std::ostream &m_os;

			template <class T>
			void operator()(const T& x) const
			{
				m_os << x << ", ";
			}
		};

		std::string trimstring(const std::string& s) {
			if(s.length() == 0)
				return s;
			int b = s.find_first_not_of(" \t\r\n");
			int e = s.find_last_not_of(" \t\r\n");
			if(b == -1) // 左右両端に、スペース、タブ、改行がない。
				return "";
			return std::string(s, b, e - b + 1);
		};

		std::string trimlastcomma(std::string s) {
			int e = s.find_last_of(",");
			s[e] = ' ';
			//std::replace( s.rbegin(), s.rend(), ',', ' ');
			//if(s.length() == 0)
			//	return s;
			//std::string::iterator b = s.begin();
			//std::string::iterator e = s.find_last(",");
			//if(b == -1 || b==e) // 左右両端に、スペース、タブ、改行がない。
			//	return "";
			return s;//std::string(s);
		};
	};
}

#endif //RTC_LIB_FUSION_CSV

