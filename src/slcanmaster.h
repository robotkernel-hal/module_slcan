//! robotkernel module slcanmaster slcanmaster
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of module_slcan.
 *
 * module_slcan is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_slcan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_slcan; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MODULE_PCAN_PCAN_H_
#define _MODULE_PCAN_PCAN_H_

#include <list>
#include <string>
#include <stdint.h>

#include "yaml-cpp/yaml.h"
#include "robotkernel/trigger_base.h"
#include "robotkernel/stream.h"
#include "robotkernel/module_base.h"

//#include "PCANBasic.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "SerialCAN_Defines.h"
#include "SerialCAN.h"
#ifdef __cplusplus
}
#endif

namespace can {
typedef struct __attribute__((__packed__)) frame {
    uint32_t hdr;
    uint8_t  rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} __attribute__((__packed__)) frame_t;
}

//! module_slcanmaster::
namespace module_slcan {

class slcanmaster : 
    public virtual robotkernel::shared_base,
    public robotkernel::module_base,
    public robotkernel::runnable 
{
    public:
        YAML::Node node;

        std::string tty_name;
        int baudrate;
    
        CSerialCAN serial_can;

        std::list<std::string> slave_stream_names;  //!< Name of slave stream devices.
        robotkernel::stream_map_t slave_streams;    //!< Slave stream devices.

        std::shared_ptr<robotkernel::triggerable> trg;  //!< @brief We get triggered by other trigger.
        std::shared_ptr<robotkernel::trigger> send_trigger;
    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        slcanmaster(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~slcanmaster();

        //! second stage init
        virtual void init() override;

        //! module trigger callback
        void tick();

        //! State transition from SAFEOP to PREOP
        virtual void set_state_safeop_2_preop() override;
 
        //! State transition from PREOP to INIT
        virtual void set_state_preop_2_init() override;
 
        //! State transition from INIT to PREOP
        virtual void set_state_init_2_preop() override;
 
        //! State transition from PREOP to SAFEOP
        virtual void set_state_preop_2_safeop() override;

        //! run receiver thread
        virtual void run() override;
};

};    //! module_slcanmaster::

#endif // _MODULE_PCAN_PCAN_H_

