/*!
* RTCLib::EigenHelper functions, 
* 
* Copyright (c) 2013 by Hiroyuki Okuda, Suzlab. Nagoya Univ.
*
* Helper functions for Eigen
* 
*/
#pragma once

#ifndef RTCLIB_EIGEN_HELPER
#define RTCLIB_EIGEN_HELPER

#include <Eigen/Core>
#include <sstream>
#include <vector>
#include <list>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>

#include "STLHelper.h"

namespace RTCLib
{
	class EigenHelper
	{
	public:

		///// --- Convert functions between STL container and EigenVecor ---
		template< typename  EigenArray, typename  Container >
		static void eigen2stl( const EigenArray &src, Container &trg )
		{
			size_t size = src.size();
			trg.resize(src.size());
			typename Container::iterator itr = trg.begin();
			for( int i = 0; i < src.size(); ++i)
			{
				*(itr++) = src(i);
			}
		}

		template< typename  EigenArray, typename  Container >
		static Container eigen2stl( const EigenArray &src)
		{
			Container trg;
			eigen2str( src, trg );
			return trg;
		}

		template<  typename  Container, typename  EigenArray >
		static void stl2eigen( const Container &src, EigenArray &trg  )
		{
			trg.resize(src.size());
			typename Container::const_iterator itr = src.begin();
			for( size_t i = 0; i < src.size(); ++i)
			{
				trg(i) = *(itr++);
			}
		}

		template<  typename  Container, typename  EigenArray >
		static EigenArray stl2eigen( const Container &src )
		{
			EigenArray trg;
			stl2eigen( src, trg );
			return trg;
		}

		///// --- Convert functions from string to EigenArray ---

		//! for Eigen::MatrixXd(f)
		template< typename T, int r, int c>
		static Eigen::Matrix<T, r, c, 0, r, c>& string2eigen( Eigen::Matrix<T, r, c, 0, r, c>& lh, std::string str )
		{
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

			boost::char_separator<char> rowsep(";", "", boost::drop_empty_tokens);
			tokenizer row_token( str , rowsep);

			// map to arrays of vector
			//typedef  Eigen::Matrix<T, c, 1, 0, c, 1> col_vec_type;
			typedef  std::vector<T > col_vec_type;
			
			std::list< col_vec_type > data;

			for( tokenizer::iterator itr = row_token.begin(); itr != row_token.end(); itr++)
			{
				std::string row_str( *itr );
				//data.push_back( col_vec_type() );
				data.push_back( STLHelper::string2stl<col_vec_type >( row_str ) );
			}

			size_t row = 0;
			size_t col = 0;
			if( r == -1 )
			{
				row = data.size();
			}
			else
			{
				row = r;
			}
			if( row != data.size() )
			{
				std::cerr << "Size mismatch in operator<<(... std::string)" << std::endl;
				throw( std::out_of_range("Size mismatch in operator<<(... std::string)") );
			}else{

			}

			if( c == -1 )
			{
				col = data.front().size();
			}
			else
			{
				col = c;
			}
			// check for jag array

			for( auto itr = data.begin(); itr != data.end(); ++itr)
			{
				if( col != (*itr).size() )
				{
					std::cerr << "Size mismatch in operator<<(... std::string)" << std::endl;
				}
			}

			lh.resize( row, col);
			typename std::list< col_vec_type >::iterator itr = data.begin();
			for( size_t i = 0; i < row ; i++)
			{
				for( size_t j = 0; j < col; j++)
				{
					lh( i, j ) = (*itr)[ j ];
				}
				itr++;
			}

			return lh;
		}

		//! for Eigen::VectorXd(f)
		template< typename T, int r>
		static Eigen::Matrix<T, r, 1, 0, r, 1>& string2eigen( Eigen::Matrix<T, r, 1, 0, r, 1>& lh, std::string str )
		{
			std::vector<T> row;
			row = STLHelper::string2stl<std::vector<T> >( str , ", ");
			
			stl2eigen( row, lh);
			return lh;
		}

		//! for any type
		static void save2csv(Eigen::MatrixXd target, std::string filename)
		{
			std::ofstream ofs(filename);

			for(int row = 0; row < target.rows(); row++)
			{
				for(int col = 0; col < target.cols(); col++)
				{
					ofs << target(row, col) << ',';
				}
				ofs << std::endl;
			}
			return ;
		}


	};

	//! for Eigen::MatrixXd(f), stream operator
	template< typename T, int r, int c>
	Eigen::Matrix<T, r, c, 0, r, c>& operator<<( Eigen::Matrix<T, r, c, 0, r, c>& lh, std::string str )
	{
		return EigenHelper::string2eigen( lh, str );
	}

	//! for Eigen::VectorXd(f). stream operator
	template< typename T, int r>
	Eigen::Matrix<T, r, 1, 0, r, 1>& operator<<( Eigen::Matrix<T, r, 1, 0, r, 1>& lh, std::string str )
	{
		return EigenHelper::string2eigen( lh, str );
	}



}

#endif //RTCLIB_EIGEN_HELPER

