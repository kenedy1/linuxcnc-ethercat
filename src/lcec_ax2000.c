//
//    Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include "lcec.h"
#include "lcec_ax2000.h"

typedef struct {
  lcec_syncs_t syncs;
  lcec_class_ax2_chan_t chan;
} lcec_ax2000_data_t;



void lcec_ax2000_read(struct lcec_slave *slave, long period);
void lcec_ax2000_write(struct lcec_slave *slave, long period);

int lcec_ax2000_preinit(struct lcec_slave *slave) {
  // check if already initialized
 
  // set FSOE conf (this will be used by the corresponding AX5805
  //slave->fsoeConf = &fsoe_conf;

  // set pdo count
  slave->pdo_entry_count = lcec_class_ax2_pdos(slave);

  return 0;
}

int lcec_ax2000_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_ax2000_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read =   lcec_ax2000_read;
  slave->proc_write =  lcec_ax2000_write;
;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_ax2000_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_ax2000_data_t));
  slave->hal_data = hal_data;

  // init subclasses
  if ((err = lcec_class_ax2_init(slave, pdo_entry_regs, &hal_data->chan, 0, "")) != 0) {
    return err;
  }

  // initialize sync info
  lcec_syncs_init(&hal_data->syncs);
    lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
    lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
    lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
      lcec_syncs_add_pdo_info(&hal_data->syncs, 0x1707);
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6062, 0x0,  32); // control-word
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x606b, 0x0,  32); // velo-command
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6074, 0x0,  16); // control-word
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6072, 0x0,  16); // velo-command
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6040, 0x0,  16); // control-word
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x2802, 0x0,  16); // velo-command


    lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
      lcec_syncs_add_pdo_info(&hal_data->syncs, 0x1B07);
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6064, 0x0, 32); // status word
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x35c9, 0x0, 32); // position feedback
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6077, 0x0, 16); // torque feedback
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x60f4, 0x0, 32); // position feedback 2
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6041, 0x0, 16); // diagnostic number
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x2901, 0x0, 16); // torque feedback
        lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x2902, 0x0, 32); // position feedback 2
    
        
  slave->sync_info = &hal_data->syncs.syncs[0];

  return 0;
}

void lcec_ax2000_read(struct lcec_slave *slave, long period) {
  lcec_ax2000_data_t *hal_data = (lcec_ax2000_data_t *) slave->hal_data;

  // check inputs
  lcec_class_ax2_read(slave, &hal_data->chan);
}

void lcec_ax2000_write(struct lcec_slave *slave, long period) {
  lcec_ax2000_data_t *hal_data = (lcec_ax2000_data_t *) slave->hal_data;

  // write outputs
  lcec_class_ax2_write(slave, &hal_data->chan);
}

