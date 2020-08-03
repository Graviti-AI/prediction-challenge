/*!
 * RTCLib::Utilities functions, 
 * 
 * Copyright (c) 2013 by Hiroyuki Okuda, Suzlab. Nagoya Univ.
 *
 * Mathematical utilities
 * 
 */
#pragma once

#ifndef RTCLIB_UTILITIES
#define RTCLIB_UTILITIES


namespace RTCLib
{
	template< class T>
	T linear_interporate(T x, T x0, T x1, T y0, T y1 )
	{
		T alpha = (x - x0) / (x1 - x0);
		T ret = y0 + alpha * (y1 - y0);
		return ret;
	}
	
	
}

#endif //RTCLIB_UTILITIES

