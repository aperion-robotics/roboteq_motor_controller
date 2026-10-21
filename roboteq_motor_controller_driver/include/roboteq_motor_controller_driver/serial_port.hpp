#ifndef ROBOTEQ_SERIAL_PORT_HPP_
#define ROBOTEQ_SERIAL_PORT_HPP_

#include <array>
#include <chrono>
#include <string>
#include <string_view>

namespace roboteq_motor_controller_driver
{

// Minimal Linux serial port (termios) for line based protocols.
// Errors are reported through return values so that it can be used in the control loop without exceptions.
class SerialPort
{

public:
	enum class ReadResult
	{
		LINE,
		TIMEOUT,
		ERROR
	};

	SerialPort() = default;
	SerialPort(const SerialPort&) = delete;
	SerialPort& operator=(const SerialPort&) = delete;
	~SerialPort();

	// Open the port in raw 8N1 mode without flow control
	bool open(const std::string& port, int baud, std::string& error);
	void close();
	bool is_open() const { return fd_ >= 0; }

	// Write all data and wait until it has been transmitted
	bool write(std::string_view data, std::string& error);

	// Read the next line terminated by eol, waiting at most until deadline.
	// The returned line excludes eol and is valid until the next call.
	ReadResult read_line(std::string_view& line, std::chrono::steady_clock::time_point deadline, std::string& error);

private:
	static constexpr char kEol = '\r';

	int fd_{-1};
	std::array<char, 1024> buffer_{};
	size_t begin_{0};
	size_t end_{0};
};

} // namespace roboteq_motor_controller_driver

#endif // ROBOTEQ_SERIAL_PORT_HPP_
