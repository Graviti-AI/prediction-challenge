/*!
 * Interface of Channel for RTCLib communication classes
 * 
 * Copyright (c) 2014 by Hiroyuki Okuda, Nagoya Univ.
 */
#pragma once

//!
class ISerialChannel
{
protected:
	ISerialChannel(void);
	virtual ~ISerialChannel(void)=0; 

public:

};

//!
class ISerialChannelSender : ISerialChannel
{
protected:
	ISerialChannelSender(void);
	virtual ~ISerialChannelSender(void)=0; 

public:
	virtual int Write( char* buf, size_t len )=0;
};

class ISerialChannelReceiver : ISerialChannel
{
protected:
	ISerialChannelReceiver(void);
	virtual ~ISerialChannelReceiver(void)=0; 

public:
	virtual int Write( char* buf, size_t len )=0;
};

