#pragma once

#include <px4.h>
#include <stdint.h>
#include <pthread.h>

namespace px4
{
namespace logger
{

class LogWriter
{
public:
	LogWriter(uint8_t *buffer, size_t buffer_size);

	pthread_t thread_start();

	void start_log(const char *filename);

	void stop_log();

	bool write(void *ptr, size_t size);

	void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	void notify()
	{
		pthread_cond_broadcast(&_cv);
	}

private:
	static void *run_helper(void *);

	void run();

	size_t get_read_ptr(void **ptr, bool *is_part);

	void mark_read(size_t n)
	{
		_count -= n;
	}

	char		_filename[64];
	int			_fd;
	uint8_t 	*_buffer;
	const size_t	_buffer_size;
	const size_t	_min_write_chunk;
	size_t			_head = 0;
	size_t			_count = 0;
	size_t		_total_written = 0;
	bool		_should_run = false;
	pthread_mutex_t		_mtx;
	pthread_cond_t		_cv;
};

}
}
