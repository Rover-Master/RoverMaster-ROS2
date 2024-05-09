#include <string>
#include <termios.h>
#include <unistd.h>

namespace serial {

int open(const char *port, int baud = B115200, int blocking = 0);

int flush(int fd);

int write(int fd, std::string data);

int readline(int fd, char **cursor, int size, const char delimiter = '\n');

int readline(int fd, char **cursor, char *const line_ep,
             const char delimiter = '\n');

} // namespace serial
