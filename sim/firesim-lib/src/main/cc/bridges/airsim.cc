//See LICENSE for license details

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


airsim_t::airsim_t(simif_t* sim, AIRSIMBRIDGEMODULE_struct * mmio_addrs, int airsimno): bridge_driver_t(sim)
{
    this->mmio_addrs = mmio_addrs;
    this->loggingfd = 0; // unused
    printf("STARTING AIRSIM BRIDGE DRIVER\n");
}

airsim_t::~airsim_t() {
    free(this->mmio_addrs);
    close(this->loggingfd);
}

void airsim_t::send() {
    if (data.in.fire()) {
        write(this->mmio_addrs->in_bits, data.in.bits);
        write(this->mmio_addrs->in_valid, data.in.valid);
    }
    if (data.out.fire()) {
        write(this->mmio_addrs->out_ready, data.out.ready);
    }
}

void airsim_t::recv() {
    data.in.ready = read(this->mmio_addrs->in_ready);
    data.out.valid = read(this->mmio_addrs->out_valid);
    if (data.out.valid) {
        data.out.bits = read(this->mmio_addrs->out_bits);
        printf("AIRSIM DRIVER: GOT BYTES %x\n", data.out.bits);
    }
}

void airsim_t::tick() {
    data.out.ready = true;
    data.in.valid = false;
    do {
        this->recv();

        if (data.in.ready) {
            char inp;
            int readamt;

            if (data.out.fire()) {
                printf("AIRSIM DRIVER: SENDING DATA: %x\n", data.out.bits);
                data.in.bits = data.out.bits;
                data.in.valid = true;
            }
        }

        this->send();
        data.in.valid = false;
    } while(data.in.fire() || data.out.fire());
}

#endif // AIRSIMBRIDGEMODULE_struct_guard
