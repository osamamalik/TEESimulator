/* Adapted from: https://stackoverflow.com/questions/53508059/c-serial-communication-reading-data-works-but-writing-fails */

#ifndef SERIAL_H
#define SERIAL_H

#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

class Serial
{
private:
    int fd;
public:
    Serial(std::string device);
    
    ~Serial()
    {
        close(fd);
    };
    
    int Available();
    void Read(char * buffer, int amountOfBytes);
    void Read(char * bytePtr);
};

#endif
