#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
//#include <stdint.h>
#include <string.h>
//#include <math.h>
//#include <signal.h>
//#include <stdbool.h>

#include "system.h"
void getIfAddress(char* ifname, char* result) {
        int fd;
        struct ifreq ifr;

        // open soket
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        // get an IPv4 IP address
        ifr.ifr_addr.sa_family = AF_INET;
        // get IP address attached to ifname: ex. "eth0" or "wlx0013efcb0cbc"
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
        ioctl(fd, SIOCGIFADDR, &ifr);
        close(fd);
        // convert to human readable
        sprintf(result, "%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
        // debug
        //printf("%s %s\n", ifname, result);
}
