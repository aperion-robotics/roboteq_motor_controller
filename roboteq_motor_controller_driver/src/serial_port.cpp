#include <roboteq_motor_controller_driver/serial_port.hpp>

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace roboteq_motor_controller_driver
{

namespace
{

bool to_speed(int baud, speed_t& speed)
{
	switch (baud)
	{
		case 9600: speed = B9600; return true;
		case 19200: speed = B19200; return true;
		case 38400: speed = B38400; return true;
		case 57600: speed = B57600; return true;
		case 115200: speed = B115200; return true;
		case 230400: speed = B230400; return true;
		case 460800: speed = B460800; return true;
		case 921600: speed = B921600; return true;
		default: return false;
	}
}

std::string errno_message(const std::string& what)
{
	return what + ": " + std::strerror(errno);
}

} // namespace

SerialPort::~SerialPort()
{
	close();
}

bool SerialPort::open(const std::string& port, int baud, std::string& error)
{
	close();

	speed_t speed;
	if (!to_speed(baud, speed))
	{
		error = "Unsupported baud rate " + std::to_string(baud);
		return false;
	}

	// Non blocking so that open doesn't wait for the carrier detect line
	fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (fd_ < 0)
	{
		error = errno_message("Unable to open " + port);
		return false;
	}

	termios tty{};
	if (::ioctl(fd_, TIOCEXCL) < 0 || ::tcgetattr(fd_, &tty) < 0)
	{
		error = errno_message("Unable to configure " + port);
		close();
		return false;
	}

	::cfmakeraw(&tty);
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (::cfsetispeed(&tty, speed) < 0 || ::cfsetospeed(&tty, speed) < 0 || ::tcsetattr(fd_, TCSANOW, &tty) < 0)
	{
		error = errno_message("Unable to configure " + port);
		close();
		return false;
	}

	// Back to blocking writes, reads are guarded by poll
	const int flags = ::fcntl(fd_, F_GETFL);
	if (flags < 0 || ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK) < 0)
	{
		error = errno_message("Unable to configure " + port);
		close();
		return false;
	}

	// Drop any stale data
	::tcflush(fd_, TCIOFLUSH);
	begin_ = 0;
	end_ = 0;

	return true;
}

void SerialPort::close()
{
	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}
	begin_ = 0;
	end_ = 0;
}

bool SerialPort::write(std::string_view data, std::string& error)
{
	if (!is_open())
	{
		error = "Serial port is not open";
		return false;
	}

	size_t written = 0;
	while (written < data.size())
	{
		const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
		if (n < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			error = errno_message("Serial write failed");
			return false;
		}
		written += n;
	}

	if (::tcdrain(fd_) < 0)
	{
		error = errno_message("Serial drain failed");
		return false;
	}

	return true;
}

SerialPort::ReadResult SerialPort::read_line(std::string_view& line, std::chrono::steady_clock::time_point deadline, std::string& error)
{
	if (!is_open())
	{
		error = "Serial port is not open";
		return ReadResult::ERROR;
	}

	while (true)
	{
		// Return a complete line if one is already buffered
		char* const start = buffer_.data() + begin_;
		const auto* eol = static_cast<const char*>(std::memchr(start, kEol, end_ - begin_));
		if (eol != nullptr)
		{
			line = std::string_view(start, eol - start);
			begin_ = eol - buffer_.data() + 1;
			return ReadResult::LINE;
		}

		// Move the partial line to the front to make room
		if (begin_ > 0)
		{
			std::memmove(buffer_.data(), start, end_ - begin_);
			end_ -= begin_;
			begin_ = 0;
		}

		if (end_ == buffer_.size())
		{
			end_ = 0;
			error = "Received line exceeds the buffer size";
			return ReadResult::ERROR;
		}

		const auto remaining = std::max(deadline - std::chrono::steady_clock::now(), std::chrono::steady_clock::duration::zero());
		const auto remaining_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count();
		const timespec timeout{static_cast<time_t>(remaining_ns / 1000000000), static_cast<long>(remaining_ns % 1000000000)};

		pollfd poll_fd{fd_, POLLIN, 0};
		const int ret = ::ppoll(&poll_fd, 1, &timeout, nullptr);
		if (ret < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			error = errno_message("Serial poll failed");
			return ReadResult::ERROR;
		}

		if (ret == 0)
		{
			return ReadResult::TIMEOUT;
		}

		if (!(poll_fd.revents & POLLIN))
		{
			error = "Serial device disconnected";
			return ReadResult::ERROR;
		}

		const ssize_t n = ::read(fd_, buffer_.data() + end_, buffer_.size() - end_);
		if (n < 0)
		{
			if (errno == EINTR || errno == EAGAIN)
			{
				continue;
			}
			error = errno_message("Serial read failed");
			return ReadResult::ERROR;
		}

		if (n == 0)
		{
			error = "Serial device disconnected";
			return ReadResult::ERROR;
		}

		end_ += n;
	}
}

} // namespace roboteq_motor_controller_driver
