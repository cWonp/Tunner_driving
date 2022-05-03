#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <resolv.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <string>

#define TIME_LOOP_WAIT          30000      // 30ms
#define COUNT_CHECK_LOOP        3          // least 10ms for 1 loop
#define SIZE_ICMP_PACKET        8          // minimum packet size


using namespace std;

struct ICMP_PACKET {
    struct icmphdr hdr;
    char msg[SIZE_ICMP_PACKET - sizeof(struct icmphdr)];
} typedef ICMP_PACKET_t;


/* functions */
unsigned short checksum(void* _data, int _length) {
    unsigned short* p_data;
    unsigned short result;
    unsigned int   sum = 0;

    p_data = (unsigned short*)_data;

    for (sum = 0; _length > 1; _length -= 2) {
        sum += *p_data++;
    }

    if( _length == 1 ) {
        sum += *(unsigned char*)p_data;
    }

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    result = ~sum;

    return result;
}

bool icmp(string _address) {
    const int sock_value = 255;

    int socket_fd;
    int num_sequence = 1;
    int pid = getpid();
    int idx;

    struct sockaddr_in r_addr;
    struct hostent *hname;
    struct sockaddr_in addr_ping,*addr;

    ICMP_PACKET_t pckt;

    int size_packet_msg = sizeof(pckt.msg);

    struct protoent *t_proto = NULL;
    socklen_t len;

    t_proto = getprotobyname("ICMP");

    hname = gethostbyname(_address.c_str());

    bzero(&addr_ping, sizeof(addr_ping));

    addr_ping.sin_family = hname->h_addrtype;
    addr_ping.sin_port   = 0;
    addr_ping.sin_addr.s_addr = *(long*)hname->h_addr;

    addr = &addr_ping;

    if( (socket_fd = socket(PF_INET, SOCK_RAW, t_proto->p_proto)) < 0 ) {
        printf("icmp() socket open failed : [%02d] %s\n", errno, strerror(errno));
        return false;
    }

    if( setsockopt(socket_fd, SOL_IP, IP_TTL, &sock_value, sizeof(sock_value)) != 0) {
        printf("icmp() set TTL option failed : [%02d] %s\n", errno, strerror(errno));
        return false;
    }

    if ( fcntl(socket_fd, F_SETFL, O_NONBLOCK) != 0 ) {
        printf("icmp() request nonblocking I/O failed : [%02d] %s\n", errno, strerror(errno));
        return false;
    }

    for( int loop_cnt = 0 ; loop_cnt < COUNT_CHECK_LOOP ; loop_cnt++ ) {
        len = sizeof(r_addr);
        if( recvfrom(socket_fd, &pckt, sizeof(pckt), 0x00, (struct sockaddr*)&r_addr, &len) > 0 ) {

            close(socket_fd);
            return true;
        }

        bzero(&pckt, sizeof(pckt));
        pckt.hdr.type = ICMP_ECHO;
        pckt.hdr.un.echo.id = pid;

        for( idx = 0; idx < size_packet_msg - 1 ; idx++ ) {
            pckt.msg[idx] = idx + '0';
        }

        pckt.msg[idx] = 0;
        pckt.hdr.un.echo.sequence = num_sequence++;
        pckt.hdr.checksum = checksum(&pckt, sizeof(pckt));

        if( sendto(socket_fd, &pckt, sizeof(pckt), 0, (struct sockaddr*)addr, sizeof(*addr)) <= 0 ) {
            printf("icmp() sendto failed : [%02d] %s\n", errno, strerror(errno));
        }

        usleep(TIME_LOOP_WAIT);
    }

    close(socket_fd);
    return false;
}


/* main function */
int main(int _argc, char *_argv[]) {
    while (true) {
        if( icmp("192.168.1.2") ) {
            printf("icmp status : [success]\n");

        } else {
            printf("icmp status : [failed]\n");
        }

        /* using system : easy but runs another process to run ping commend */
        // int x = system("ping -c1 -s1 -W1 192.168.1.2 > /dev/null 2>&1"); 
        // if (x==0){
        //     printf("icmp status : [success]\n");
        // } else {
        //     printf("icmp status : [failed]\n");
        // }
    }
    return 0;
}
