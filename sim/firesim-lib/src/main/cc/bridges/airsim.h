//See LICENSE for license details
#ifndef __AIRSIM_H
#define __AIRSIM_H

#include "serial.h"
#include <signal.h>

// The definition of the primary constructor argument for a bridge is generated
// by Golden Gate at compile time _iff_ the bridge is instantiated in the
// target. As a result, all bridge driver definitions conditionally remove
// their sources if the constructor class has been defined (the
// <cname>_struct_guard macros are generated along side the class definition.)
//
// The name of this class and its guards are always BridgeModule class name, in
// all-caps, suffixed with "_struct" and "_struct_guard" respectively.

#ifdef AIRSIMBRIDGEMODULE_struct_guard
class airsim_t: public bridge_driver_t
{
    public:
        airsim_t(simif_t* sim, AIRSIMBRIDGEMODULE_struct * mmio_addrs, int airsimno);
        ~airsim_t();
        virtual void tick();
        // Our AIRSIM bridge's initialzation and teardown procedures don't
        // require interaction with the FPGA (i.e., MMIO), and so we don't need
        // to define init and finish methods (we can do everything in the
        // ctor/dtor)
        virtual void init() {};
        virtual void finish() {};
        // Our AIRSIM bridge never calls for the simulation to terminate
        virtual bool terminate() { return false; }
        // ... and thus, never returns a non-zero exit code
        virtual int exit_code() { return 0; }

    private:
        AIRSIMBRIDGEMODULE_struct * mmio_addrs;
        serial_data_t<uint32_t> data;
        int inputfd;
        int outputfd;
        int loggingfd;
        void send();
        void recv();
};
#endif // AIRSIMBRIDGEMODULE_struct_guard

#endif // __AIRSIM_H
