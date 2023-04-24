#include "socketcan.h"
#include <sys/socket.h>
#include <linux/can.h>
#include <sys/types.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>

//#define ip_cmd_set_can0_params ""

SocketCAN::SocketCAN()
{

}

int SocketCAN::InitCAN()
{
//    int
}
