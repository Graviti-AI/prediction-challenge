/*!
* RTCLib::STLHelper functions, 
* 
* Copyright (c) 2013 by Hiroyuki Okuda, Suzlab. Nagoya Univ.
*
* Helper functions for Eigen
* 
*/
#pragma once

#ifndef RTCLIB_STL_HELPER
#define RTCLIB_STL_HELPER

#include <string>
#include <iostream>
#include <sstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>


namespace RTCLib
{
	class STLHelper
	{
	public:
		
		//! Convert string array to container(list, vector...)
		/// Example : 
		/// auto vec = STLHelper::string2stl< vector<double> >( "1,2,3", ", " );
		template< typename Container >
		static Container string2stl( std::string str , std::string delim = ", ")
		{
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> colsep( delim.c_str(), "", boost::drop_empty_tokens);
			tokenizer col_token( str  , colsep);

			Container row;

			for( tokenizer::iterator itr2 = col_token.begin(); itr2 != col_token.end(); itr2++)
			{
				std::string tmp(*itr2);
				row.push_back( boost::lexical_cast<Container::value_type>(*itr2) );
			}
			return row;
		}

		//! Convert container to string (list, vector...)
		/// Example : 
		/// auto vec = STLHelper::string2stl< vector<double> >( "1,2,3", ", " );
		template< typename Container >
		static std::string stl2string( Container trg, std::string delim = "," )
		{
			std::string ret = "";
			for( auto itr = trg.begin(); itr != trg.end(); ++itr)
			{
				ret += boost::lexical_cast< std::string >( *itr );
				ret += delim;
			}
			boost::trim_if( ret, boost::is_any_of(delim) );
			return ret;
		}

	};

}

#endif //RTCLIB_EIGEN_HELPER

