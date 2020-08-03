
#pragma once

#ifndef RTCLIB_XML_LOADER
#define RTCLIB_XML_LOADER


/*!
* ///////////////////////////////////////////////////////////////////////////////////////
* 
* 自分のための，Realtime制御フレームワーク整備事業  2011 / Jun - 2011? / ??
*								Copyright @ 奥田裕之 2011 
* 
* 第一弾，パラメータ保存のためのクラス
*
* 要件：C++, STL, Boost C++ library
* 
* ///////////////////////////////////////////////////////////////////////////////////////
*  
* //---------------- 使い方 ----------------------
*  // 最初にxmlドキュメントを読み込む．相対アドレス可能．たぶん絶対アドレスも可能．
*  // 文字列はchar系．xmlだけど，Sjisで… 文字コードの変え方は忘れました
* 	XMLloader xml("./TestRoot.xml");	// サンプルとしてTestroot.xml
*
*  
*  // カーソル的なインターフェース(IXmlElement)で順にタグをたどれる．
*	IXmlElement tmp = xml >> "Data";		// xmlのルートから，Dataへとたどる
*
*  // 各種情報取得．GetTag()でタグ名取得，IXmlElementをGetText(),またはstringキャストで，タグ内テキスト取得．
*  // <Hoge>Mogemoge</Hoge> ならば，GetTag() -> "Hoge", (string)tmp -> "Mogemoge"
*	cout << "[" << tmp.GetTag() << "]\"" << (string)tmp << "\"" << endl;
* 
*  // さらにたどってみる．>>演算子で順番にタグを下階層に降りていく
*	IXmlElement tmp2 = tmp >> "Cars" >> "Car";		// Cars内にはCarが複数ある.この場合最初のCarを示す．
*  
*  // 複数同種の"Car"タグがある場合，[]演算子で定義順に参照可能．()演算子はアトリビュート取得．
*	cout << "Car0:" << tmp2[0]("carID") << endl;	// 0番目のCarを取得，
*	cout << "Car1:" << tmp2[1]("carID") << endl;	// ()演算子でアトリビュート"carID"を取得
*													// <Cars>
*													//   <Car carID="3"/>
*													//   <Car carID="4"/>
*													// </Cars>	とあれば，それぞれ3と4が取得可能
*	string carIDstr = tmp2[0].GetAttribute("carID");			// 関数形式でアトリビュートを取得
*	double carIDdbl = tmp2[0].GetAttributeAs<double>("carID");	// 型を指定してアトリビュートを指定．
*														// boost::lexical_castによる変換をしてくれる．変換不可能なら例外も．
*  
*  // 指示をまとめて指定することも可能．演算子の優先順位があるので，下のように囲う．
*	// これで，[Root->Data->Carsの二番目]を参照．
*	IXmlElement tmp3 = (xml >> "Data" >> "Cars")[1];
*	cout << tmp3.GetTag() << ",";
*	tmp3 = tmp3 >> "Data2";				// ここで，このData2は，TestChild.xmlが入れ子読み込みされている．
*	cout << tmp3.GetTag() << ",";		// コメントに，下記のディレクティブを挿入すると，xmlが読み込まれる．
*	tmp3 = tmp3 >> "Cars2";				// <!-- ###INCLUDE:"TestChild.xml"### ほげほげほげ-->
*	cout << tmp3.GetTag() << ",";		// [ ###INCLUDE:"*****"### ]で，左右にスペースが1個は必須だが，
*	tmp3 = tmp3 >> "Car";				// それ以外のコメントが続いてもよい． 
*	cout << tmp3.GetTag() << ",";		// ""の中身以外はそのままの指定が必要．上記をコピーされたし．
*
*	cout << endl;
*	for(int i=0; i<12;i++)
*	{
*		cout << "id = " << tmp3[i]("carID")  << endl;	// 配列外アクセスをしても，警告だけで制御は続行
*	}
*/



#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/lexical_cast.hpp>

// XMLアクセッサ
#include <tinyxml.h>

#pragma comment (lib, "XMLLoader.lib")

namespace RTCLib
{
	using std::vector;
	using std::string;
	using std::map;
	using boost::shared_ptr;
	using boost::weak_ptr;
	//using boost::lambda;
	
	class IXmlElement;

	class XMLloader
	{
	public:
		class Element;
	private:
		typedef vector< shared_ptr<Element> > ElementPtrArray;
		typedef weak_ptr<Element> ElementWeakPtr;
		typedef vector< ElementWeakPtr > ElementWeakPtrArray;
	
	public:
		friend class IXmlElement;
		class Element{
			friend class XMLloader;
			friend class IXmlElement;

		private:
			bool isInvalid;
			unsigned int depth_;
			string tag_;
			string val_;
			ElementPtrArray childs_;
			// 親兄弟への参照．削除責任は持たない
			ElementWeakPtrArray brothers_;
			Element* parent_;
			// アトリビュート
			map<string,string> attributes_;

			// invalidな場合はこれを使って返す
			static std::shared_ptr<Element> invalid_;
			static bool ismodifying_;

		public:
			Element() {clear();}
			//Element(Element& p) : parent_(&p) {clear();}
			~Element(){};
			// 比較演算はタグで行うことに．インスタンスが等しいというわけではないので注意．
			bool operator==(string key){return tag_ == key;};

			// 代入演算子．通常は使ってはいけません…
			const Element& operator=(const Element& rhs);

			// 削除するっす
			void clear(){
				isInvalid=false;
				depth_=0;tag_="";val_="";
				childs_.clear();attributes_.clear();
				parent_ = nullptr;
			};

			// 有効かどうかの判定
			bool isValid(){return !isInvalid;};

			// 変更可能モードとの行き来
			static bool ToModifyingMode(){ismodifying_=true;};
			static bool ToReadingMode(Element &rte ){
				rte.reset_relationships(rte.childs_); 
				ismodifying_=false;
			};

			// 親子，兄弟関係の再セット
			void reset_relationships(ElementPtrArray &trg);

			// タグを取得
			string const GetTag(){return tag_;};

			// stringにCastするとタグ内テキストを取得
			string const GetText();
			operator const string();

			// アトリビュートを取得
			string const GetAttribute(string atr);
			string const operator()(string atr);
			// キャスト付きでアトリビュートを取得．lexical_castによる変換．
			template<class T>
			T GetAttributeAs(string str);

			// 子参照
			Element& GetChilds(string key);
			Element& operator>>(string key);

			// 同名タグの配列参照．xmlへの記載順になるはず…
			Element& operator[](size_t n);

		};

	public:
		// public Methods
		XMLloader(void);
		XMLloader(std::string fn);

		~XMLloader(void);

		int loadXML(std::string fn);
		void clear();

		Element& operator>>(string key);

	private:

		// Members
		TinyXMLPerser::TiXmlDocument doc_;
		string filename_;
		
		Element root_;

		enum XML_LOADER_DEBUG_LEVEL{
			NO_OUTPUT=0,
			PRINT_TAG = 1,
			PRINT_VAL = 2,
			PRINT_ATTRIBUTES = 4,
			WARNING_NO_ATTRIBUTE = 8,
			WARNING_BAD_REF_ELEMENT = 16,

			COMPLETE_DEBUG = 0xFF,
		};
		static const XML_LOADER_DEBUG_LEVEL debuglevel_ = (XML_LOADER_DEBUG_LEVEL) (WARNING_NO_ATTRIBUTE ^ WARNING_BAD_REF_ELEMENT);

		// private Methods
		void parse_layer(TinyXMLPerser::TiXmlNode* parent, ElementPtrArray& ele, unsigned int depth=0);
		void parse_element(TinyXMLPerser::TiXmlElement* trg, ElementPtrArray& p, unsigned int depth=0);

		// コメントを解析してネストされたXMLを展開
		void parse_comment(string const &str, ElementPtrArray& p, unsigned int depth);
		void parse_includion(string const &fn, ElementPtrArray& p, unsigned int depth);
		// 親子，兄弟を設定
		void reset_relationships();

	};

	////////////////////////////////////templateの実装//////////////////////
	// キャスト付きアトリビュート取得
	template<class T>
	T XMLloader::Element::GetAttributeAs(string str)
	{
		T ret = T();
		try{
			ret = boost::lexical_cast<T>(GetAttribute(str));
		}catch( boost::bad_lexical_cast ex){
			// デフォルト値；
		}
		return ret;
	};
	///////////////////////////////////////////////////////////////////////

	//typedef XMLloader::Element& XMLElementRef;
	typedef XMLloader::Element* XMLElementPtr;

	// 要素へのインターフェース．本体に影響を及ぼさないように．
	class IXmlElement{
		friend class XMLloader;
		friend class XMLloader::Element;
	private:
		typedef XMLloader::Element Ele;
		XMLloader::Element* ref_;

	public:
		IXmlElement() : ref_(NULL){};
		IXmlElement(Ele &ele) : ref_(&ele){};
		
		Ele& operator=(Ele &ele){ref_=&ele;return *ref_;};

		// 存在を確認
		bool isValid(){return ref_->isValid();};

		// タグを取得
		string const GetTag(){return ref_->GetTag();};

		// stringにCastするとタグ内テキストを取得
		string const GetText(){
			if(NULL != ref_)	return (*ref_).GetText();
			else				return string(""); 
		};
		operator string(){return GetText();};

		// アトリビュートを取得
		string GetAttribute(string atr){
			if(NULL != ref_)	return (*ref_).GetAttribute(atr);
			else				return string(""); 
		};
		
		// アトリビュートを取得
		template<typename U>
		U GetAttributeAs(string atr){
			if(NULL != ref_)	return (*ref_).GetAttributeAs<U>(atr);
			else				return U(); 
		};

		string operator()(string atr){return GetAttribute(atr);};

		// 子参照
		Ele& GetChilds(string key){
			if(NULL != ref_)	return ref_->GetChilds(key);
			else				return *Ele::invalid_; 
		}
		//Element& operator[](string key);
		Ele& operator>>(string key){return GetChilds(key);};

		// 同名タグの配列参照．xmlへの記載順になるはず…
		Ele& operator[](size_t n){
			if(NULL != ref_)	return (*ref_)[n];
			else				return *Ele::invalid_; 
		};

	};

	
}

#endif //RTCLIB_XML_LOADER
