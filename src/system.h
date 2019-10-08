// ip address
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>


// uptime
#include <sys/sysinfo.h>

// shutdown and reboot
//#include <sys/reboot.h>

void getIfAddress(char* ifname, char* result);
