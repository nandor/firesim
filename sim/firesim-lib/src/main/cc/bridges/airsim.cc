// See LICENSE for license details

// Note: struct_guards just as in the headers
#ifdef AIRSIMBRIDGEMODULE_struct_guard

#include "airsim.h"
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

/* There is no "backpressure" to the user input for sigs. only one at a time
 * non-zero value represents unconsumed special char input.
 *
 * Reset to zero once consumed.
 */

int count = 0;

// COSIM-CODE
// Add these functions due to overloading in the uart class
ssize_t net_write(int fd, const void *buf, size_t count)
{
    return write(fd, buf, count);
}

ssize_t net_read(int fd, void *buf, size_t count)
{
    return read(fd, buf, count);
}
// COSIM-CODE

airsim_t::airsim_t(simif_t *sim, AIRSIMBRIDGEMODULE_struct *mmio_addrs, int airsimno) : bridge_driver_t(sim)
{
    this->mmio_addrs = mmio_addrs;
    this->loggingfd = 0; // unused
    this->connect_synchronizer();
}

airsim_t::~airsim_t()
{
    free(this->mmio_addrs);
    close(this->loggingfd);
}

void airsim_t::connect_synchronizer()
{
    // COSIM-CODE
    // Adapted from: https://www.cs.cmu.edu/afs/cs/academic/class/15213-f99/www/class26/tcpclient.c
    this->hostname = "192.168.0.47";
    // this->portno   = 10100 + uartno;
    this->portno = 60001;

    /* socket: create the socket */
    this->sockfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (this->sockfd < 0)
    {
        perror("ERROR opening socket");
        exit(0);
    }
    printf("Created socket!\n");

    /* gethostbyname: get the server's DNS entry */
    this->server = gethostbyname(hostname);
    if (this->server == NULL)
    {
        fprintf(stderr, "ERROR, no such host as %s\n", hostname);
        exit(0);
    }
    printf("Got server's DNS entry!\n");

    /* build the server's Internet address */
    bzero((char *)&this->serveraddr, sizeof(this->serveraddr));
    this->serveraddr.sin_family = AF_INET;
    bcopy((char *)this->server->h_addr,
          (char *)&this->serveraddr.sin_addr.s_addr, this->server->h_length);
    this->serveraddr.sin_port = htons(this->portno);
    printf("Got server's Internet address!\n");

    /* connect: create a connection with the server */
    while (connect(this->sockfd, (const sockaddr *)&this->serveraddr, sizeof(this->serveraddr)) < 0)
        ;
    // COSIM-CODE
}

void airsim_t::process_packet()
{
    uint32_t cmd;
    uint32_t num_bytes;
    int n;

    cosim_packet_t packet;

    bzero(this->buf, ROBOTICS_COSIM_BUFSIZE);
    n = net_read(this->sockfd, this->buf, ROBOTICS_COSIM_BUFSIZE);
    if (n > 0)
    {
        packet.decode(this->buf);
        printf("[AirSim Driver]: Got packet: ");
        packet.print();
        switch (packet.cmd & 0xFF)
        {
        case CS_GRANT_TOKEN:
            this->grant_cycles();
            break;
        case CS_REQ_CYCLES:
            this->report_cycles();
            break;
        case CS_DEFINE_STEP:
            this->set_step_size(packet.data[0]);
            break;
        }
    }
    //printf("[AirSim Driver]: Finished processing packets\n");
}

void airsim_t::send()
{
    if (data.in.fire())
    {
        write(this->mmio_addrs->in_bits, data.in.bits);
        write(this->mmio_addrs->in_valid, data.in.valid);
    }
    if (data.out.fire())
    {
        write(this->mmio_addrs->out_ready, data.out.ready);
    }
}

void airsim_t::recv()
{
    data.in.ready = read(this->mmio_addrs->in_ready);
    data.out.valid = read(this->mmio_addrs->out_valid);
    if (data.out.valid)
    {
        data.out.bits = read(this->mmio_addrs->out_bits);
        printf("[AirSim Driver]: Got bytes %x\n", data.out.bits);
    }
}

void airsim_t::grant_cycles()
{
    printf("[AirSim Driver]: Granting Cycle\n");
    write(this->mmio_addrs->in_ctrl_bits, 1);
    write(this->mmio_addrs->in_ctrl_valid, true);
}

void airsim_t::report_cycles() 
{
    cosim_packet_t response;
    bzero(this->buf, ROBOTICS_COSIM_BUFSIZE);

    uint32_t cycles = read(this->mmio_addrs->cycle_budget);

    response.init(CS_RSP_CYCLES, 4, (char *) &cycles);
    response.encode(this->buf);
    printf("[AirSim Driver]: Sending cycles packet: ");
    response.print();
    net_write(this->sockfd, this->buf, response.num_bytes + 8);
}

void airsim_t::set_step_size(uint32_t step_size)
{
    printf("[AirSim Driver]: Setting step size to %d!\n", step_size);
    write(this->mmio_addrs->cycle_step, step_size);
}

void airsim_t::tick()
{
    data.out.ready = true;
    data.in.valid = false;
    // printf("[AirSim Driver]: Processing tick\n");
    this->process_packet();
    do
    {
        this->recv();

        if (data.in.ready)
        {
            char inp;
            int readamt;

            if (data.out.fire())
            {
                printf("[AirSim Driver]: Sending data: %x\n", data.out.bits);
                data.in.bits = data.out.bits;
                data.in.valid = true;
            }
        }

        this->send();
        data.in.valid = false;
    } while (data.in.fire() || data.out.fire());
}

cosim_packet_t::cosim_packet_t()
{
    this->cmd = 0x00;
    this->num_bytes = 0;
    this->data = NULL;
}

cosim_packet_t::cosim_packet_t(uint32_t cmd, uint32_t num_bytes, char *data)
{
    this->init(cmd, num_bytes, data);
}

cosim_packet_t::cosim_packet_t(char *buf)
{
    this->decode(buf);
}

cosim_packet_t::~cosim_packet_t()
{
    free(this->data);
}

void cosim_packet_t::print()
{
    printf("cmd: %x, num_bytes: %d, data: [", this->cmd, this->num_bytes);
    for (int i = 0; i < this->num_bytes / 4; i++)
        printf("%d ", this->data[i]);
    printf("]\n");
}

void cosim_packet_t::init(uint32_t cmd, uint32_t num_bytes, char *data)
{
    this->cmd = cmd;
    this->num_bytes = num_bytes;
    if (this->num_bytes > 0)
    {
        this->data = (uint32_t *)malloc(num_bytes);
        memcpy(this->data, data, num_bytes);
    }
}

void cosim_packet_t::encode(char *buf)
{
    memcpy(&buf[0], &(this->cmd), sizeof(uint32_t));
    memcpy(&buf[4], &(this->num_bytes), sizeof(uint32_t));
    if (this->num_bytes > 0)
    {
        memcpy(&buf[8], this->data, this->num_bytes);
    }
}

void cosim_packet_t::decode(char *buf)
{
    memcpy(&(this->cmd), &buf[0], sizeof(uint32_t));
    memcpy(&(this->num_bytes), &buf[4], sizeof(uint32_t));
    if (this->num_bytes > 0)
    {
        this->data = (uint32_t *)malloc(num_bytes);
        memcpy(this->data, &buf[8], num_bytes);
    }
}

#endif // AIRSIMBRIDGEMODULE_struct_guard
