/*!
 * <���ʃ������o�R�ł̃f�[�^���L�e���v���[�g�N���X>
 * 
 * Copyright (c) 2011 by Hiroyuki Okuda
 *
 * PC�����ł�IPC���s���ۂ̋��ʃ�������POD�^�����L�D
 * linux�ł�windows�ł����L�ł����ڎw���D
 * �������Cwin�̏ꍇ�C�W���̋��L������������A�N�Z�X�ɂ���D
 */

#pragma once

#pragma warning ( push )
#pragma warning ( disable : 4819 )
#pragma warning ( disable : 4345 )

// ���L�������ʐM�ɕK�v�ȃw�b�_�t�@�C���Q
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

// �ʓ|�Ȃ̂�public���߂�
// �܂邿������ǂ��K���Ή��Ȃ̂ł��߂�
// �A�N�Z�X����Ƃ��́C�Ή�����~���[�e�b�N�X��������lock���ăA�N�Z�X���Ă�.
template <class T>
class SharedMemoryAccessor
{
public:
	// ���L���������J��
	bool Open(bool serve_host, const char* sharemem_name, const char* sharemutex_name="");

	// �ŏ��T�[�o�Ƃ��ăI�[�v�������݁C���s������N���C�A���g�Ƃ��ăI�[�v������
	bool TryOpen(const char* sharemem_name, const char* sharemutex_name="");

	// ���O�t��mutex���g�����v���Z�X�Ԃ̃��\�[�X���b�N
	bool lock();
	void unlock();

	// ���L�������̃z�X�g�v���O������true�ɂč\�z�C����ȊO��false�ɂč\�z
	SharedMemoryAccessor():m_opened(false), p_shared_data(0){}
	SharedMemoryAccessor(bool serve_host, const char* sharemem_name, const char* sharemutex_name=""):m_opened(false), p_shared_data(0){
		Open(serve_host, sharemem_name, sharemutex_name);
	}
	~SharedMemoryAccessor();

	// �}���`�v���Z�X�ł̔r���A�N�Z�X�I�u�W�F�N�g�����l�[��
	typedef boost::interprocess::interprocess_mutex Ipc_mutex;
	typedef boost::interprocess::scoped_lock<Ipc_mutex> Scoped_lock;

	// �z�X�g�p�̎������L�������폜�I�u�W�F�N�g
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

	// ���L�������}�l�[�W��
#ifdef WIN32
	// windows�ł͑��v���Z�X�Ƃ��l�C�e�B�u�ƑΉ��ł���悤�ɁD
	std::unique_ptr< boost::interprocess::windows_shared_memory > pshm;
	std::unique_ptr< boost::interprocess::mapped_region > pregion;
#else
	// linux��boost���񂾂���
	std::unique_ptr< boost::interprocess::managed_shared_memory > pshm;
#endif

	bool m_opened;

	/// �f�[�^���ǉ��ŕK�v�ɂȂ�ꍇ�ɂ͉��L�ɒǉ�����D
	/// ���L����f�[�^�ւ̃|�C���^

	// �Z���V���O�f�[�^�ƁC����p�̃~���[�e�b�N�X�I�u�W�F�N�g
	T* p_shared_data;
	T* Get(){ return p_shared_data; }
	
	/// ���{����win32��API��boost��named_mutex��؂�ւ�����...
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

		// �T�[�o�[����
#ifdef WIN32
		if(sharemem_name==NULL)sharemem_name = "DEFAULT_SHARED_MEMORY_NAME";

		// For Windows : C#�ƒʐM�\�Ȃ悤��Windows�̋��L���������g���ł�
		pshm.reset( new boost::interprocess::windows_shared_memory( 
			boost::interprocess::create_only, sharemem_name, boost::interprocess::read_write, sizeof(T) )
			);
		pregion.reset( new boost::interprocess::mapped_region( *pshm, boost::interprocess::read_write) );

		// ���L�������̈�̐擪�A�h���X���l��
		void* head = (void*)pregion->get_address();
		
		// ���L�f�[�^���\�z: placement new;
		p_shared_data = new(head) T();

		// mutex���\�z
		//p_lrf_mutex = new(head) Ipc_mutex();
		
#else
		// For linux
		// �I�����̎����폜��o�^�C�����ɍ폜����D
		premover.reset( new Remover() );

		// �V���ɋ��L�̈���쐬
		pshm.reset( new  managed_shared_memory( boost::interprocess::create_only , SHARED_MEMORY_NAME, 65536 ) );

		// ���L��������ɃI�u�W�F�N�g���m��

		//p_lrf_mutex = pshm->construct<Ipc_mutex>(SHARED_LRF_DATA_MUTEX_NAME)();
		p_shared_data = pshm->construct<T>(SHARED_LRF_DATA_NAME)();
		//memset( p_lrf_data, 0, sizeof(p_lrf_data) );

#endif

	}else{

		// �A�N�Z�X�N���C�A���g����
#ifdef WIN32
		// For Windows : C#�ƒʐM�\�Ȃ悤��Windows�̋��L���������g���ł�
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

		// ���L�������̈�̐擪�A�h���X���l��
		void* head = (void*)pregion->get_address();

		// ���L�f�[�^:placed_new
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

// �����炭�������
template <class T>
SharedMemoryAccessor<T>::~SharedMemoryAccessor()
{

}

// ���O�t��mutex���g�����v���Z�X�Ԃ̃��\�[�X�̃��b�N
template <class T>
bool SharedMemoryAccessor<T>::lock()
{
	return mutex.lock();
}

// ���O�t��mutex���g�����v���Z�X�Ԃ̃��\�[�X�̃A�����b�N
template <class T>
void SharedMemoryAccessor<T>::unlock()
{
	mutex.unlock();
}

}

#pragma warning ( pop )
