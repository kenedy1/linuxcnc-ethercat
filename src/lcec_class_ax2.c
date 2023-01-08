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
#include "lcec_class_enc.h"
#include "lcec_class_ax2.h"

static const lcec_pindesc_t slave_pins[] = {
  { HAL_BIT, HAL_IN, offsetof(lcec_class_ax2_chan_t, enable), "%s.%s.%s.%ssrv-enable" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, enabled), "%s.%s.%s.%ssrv-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, halted), "%s.%s.%s.%ssrv-halted" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, fault), "%s.%s.%s.%ssrv-fault" },
  { HAL_BIT, HAL_IN, offsetof(lcec_class_ax2_chan_t, halt), "%s.%s.%s.%ssrv-halt" },
  { HAL_BIT, HAL_IN, offsetof(lcec_class_ax2_chan_t, drive_off), "%s.%s.%s.%ssrv-drive-off" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_class_ax2_chan_t, velo_cmd), "%s.%s.%s.%ssrv-velo-cmd" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_class_ax2_chan_t, torq_cmd), "%s.%s.%s.%ssrv-torq-cmd" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_class_ax2_chan_t, mtorq_cmd), "%s.%s.%s.%ssrv-mtorq-cmd" },





  { HAL_U32, HAL_OUT, offsetof(lcec_class_ax2_chan_t, status), "%s.%s.%s.%ssrv-status" },
  { HAL_U32, HAL_OUT, offsetof(lcec_class_ax2_chan_t, latch_status), "%s.%s.%s.%ssrv-latch-status" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, latch_pos), "%s.%s.%s.%ssrv-latch-pos" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, torq_fb), "%s.%s.%s.%ssrv-torque-fb" }, 
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ax2_chan_t, follerr_fb), "%s.%s.%s.%ssrv-follerr-fb" },


  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};



static const lcec_pindesc_t slave_params[] = {
  { HAL_FLOAT, HAL_RW, offsetof(lcec_class_ax2_chan_t, scale), "%s.%s.%s.%ssrv-scale" },
  { HAL_FLOAT, HAL_RW, offsetof(lcec_class_ax2_chan_t, scale_fb2), "%s.%s.%s.%ssrv-scale-fb2" },
  { HAL_FLOAT, HAL_RO, offsetof(lcec_class_ax2_chan_t, vel_scale), "%s.%s.%s.%ssrv-vel-scale" },
  { HAL_U32, HAL_RO, offsetof(lcec_class_ax2_chan_t, pos_resolution), "%s.%s.%s.%ssrv-pos-resolution" },
  
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};



static int get_param_flag(struct lcec_slave *slave, int id) {
  LCEC_CONF_MODPARAM_VAL_T *pval;

  pval = lcec_modparam_get(slave, id);
  if (pval == NULL) {
    return 0;
  }

  return pval->bit;
}

int lcec_class_ax2_pdos(struct lcec_slave *slave) {
  int pdo_count = 13;

  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Prbase read error\n");
    return -EIO;
  }


  return pdo_count;
}

int lcec_class_ax2_init(struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs, lcec_class_ax2_chan_t *chan, int index, const char *pfx) {
  lcec_master_t *master = slave->master;
  int err;
  uint8_t idn_buf[4];
  uint32_t idn_pos_resolution = 20;
  uint16_t idn_vel_scale = 5000;
  int16_t idn_vel_exp = 100;
  char enc_pfx[HAL_NAME_LEN];

  // initialize POD entries
/*
SM2: PhysAddr 0x1100, DefaultSize    0, ControlRegister 0x24, Enable 1
  RxPDO 0x1707 ""
    PDO entry 0x6062:00, 32 bit, ""
    PDO entry 0x606b:00, 32 bit, ""
    PDO entry 0x6074:00, 16 bit, ""
    PDO entry 0x6072:00, 16 bit, ""
    PDO entry 0x6040:00, 16 bit, ""
    PDO entry 0x2802:00, 16 bit, ""
SM3: PhysAddr 0x1140, DefaultSize    0, ControlRegister 0x22, Enable 1
  TxPDO 0x1b07 ""
    PDO entry 0x6064:00, 32 bit, ""
    PDO entry 0x35c9:00, 32 bit, ""
    PDO entry 0x6077:00, 16 bit, ""
    PDO entry 0x60f4:00, 32 bit, ""
    PDO entry 0x6041:00, 16 bit, ""
    PDO entry 0x2901:00, 16 bit, ""
    PDO entry 0x2902:00, 32 bit, ""
*/

  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6062, 0x0 , &chan->pos_cmd_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x606b, 0x0 , &chan->vel_cmd_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6074, 0x0 , &chan->trq_cmd_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6072, 0x0 , &chan->mtrq_cmd_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x0 , &chan->ctrl_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2802, 0x0 , &chan->latch_ctrl_pdo_os, NULL);

  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6064, 0x0 , &chan->pos_fbpdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x35c9, 0x0 , &chan->pos_fb2_pdos_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6077, 0x0 , &chan->torque_fb_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60f4, 0x0 , &chan->follerr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x0 , &chan->status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2901, 0x0 , &chan->latchstatus_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2902, 0x0 , &chan->latchpos_pdo_os, NULL);

  // export pins
 if ((err = lcec_pin_newf_list(chan, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, pfx)) != 0) {
    return err;
  }
  

  // set to cyclic synchronous velocity mode
  if (ecrt_slave_config_sdo8(slave->config, 0x6060, 0x00, (uint8_t)0x09 ) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo velo mode\n", master->name, slave->name);
  }

  // initialie encoder
  rtapi_snprintf(enc_pfx, HAL_NAME_LEN, "%senc", pfx);
  if ((err = class_enc_init(slave, &chan->enc, 32, enc_pfx)) != 0) {
    return err;
  }
   

  rtapi_snprintf(enc_pfx, HAL_NAME_LEN, "%senc-fb2", pfx);
    if ((err = class_enc_init(slave, &chan->enc_fb2, 32, enc_pfx)) != 0) {
      return err;
    }
// export params
  if ((err = lcec_param_newf_list(chan, slave_params, LCEC_MODULE_NAME, master->name, slave->name, pfx)) != 0) {
    return err;
  }
 


  // init parameters
  chan->scale = 1.0;
  chan->scale_fb2 = 1.0;
  chan->vel_scale = ((double) idn_vel_scale) * pow(10.0, (double) idn_vel_exp);
  chan->pos_resolution = idn_pos_resolution;

  if (chan->vel_scale > 0.0) {
    chan->vel_output_scale = 60.0 / chan->vel_scale;
  } else {
    chan->vel_output_scale = 0.0;
  }

  return 0;
}

void lcec_class_ax2_check_scales(lcec_class_ax2_chan_t *chan) {
  // check for change in scale value
  if (chan->scale != chan->scale_old) {
    // scale value has changed, test and update it
    if ((chan->scale < 1e-20) && (chan->scale > -1e-20)) {
      // value too small, divide by zero is a bad thing
      chan->scale = 1.0;
    }
    // save new scale to detect future changes
    chan->scale_old = chan->scale;
    // we actually want the reciprocal
    chan->scale_rcpt = 1.0 / chan->scale;
  }

  // check fb2 for change in scale value
  if (chan->scale_fb2 != chan->scale_fb2_old) {
    // scale value has changed, test and update it
    if ((chan->scale_fb2 < 1e-20) && (chan->scale_fb2 > -1e-20)) {
      // value too small, divide by zero is a bad thing
      chan->scale_fb2 = 1.0;
    }
    // save new scale to detect future changes
    chan->scale_fb2_old = chan->scale_fb2;
    // we actually want the reciprocal
    chan->scale_fb2_rcpt = 1.0 / chan->scale_fb2;
  }
}

void lcec_class_ax2_read(struct lcec_slave *slave, lcec_class_ax2_chan_t *chan) {
  lcec_master_t *master = slave->master;
  uint8_t *pd = master->process_data;
  uint32_t pos_cnt;

  // wait for slave to be operational
  if (!slave->state.operational) {
    chan->enc.do_init = 1;
    chan->enc_fb2.do_init = 1;
    *(chan->fault) = 1;
    *(chan->enabled) = 0;
    *(chan->halted) = 0;
    return;
  }

  // check inputs
  lcec_class_ax2_check_scales(chan);

  *(chan->status) = EC_READ_U16(&pd[chan->status_pdo_os]);
  
  // check fault
  *(chan->fault) = 0;
  // check error ETERCAT OK
  if (((*(chan->status) >> 12) & 1) != 1) {
   *(chan->fault) = 1;
  }
  // check hardware error
  if (((*(chan->status) >> 14) & 3) != 0) {
   // *(chan->fault) = 1;
  }

  // check status
  *(chan->enabled) = (((*(chan->status) >> 1) & 2) == 1);
  *(chan->halted) = (((*(chan->status) >> 6) & 1) == 1);


  // update position feedback
  pos_cnt = EC_READ_U32(&pd[chan->pos_fbpdo_os]);
  class_enc_update(&chan->enc, chan->pos_resolution, chan->scale_rcpt, pos_cnt, 0, 0);


  pos_cnt = EC_READ_U32(&pd[chan->pos_fb2_pdos_os]);
  class_enc_update(&chan->enc_fb2, 1, chan->scale_fb2_rcpt, pos_cnt, 0, 0);
  
  
  *(chan->follerr_fb) = ((double) EC_READ_S32(&pd[chan->follerr_pdo_os])) / 5000;

  pos_cnt = EC_READ_U32(&pd[chan->latchpos_pdo_os]);
  *(chan->latch_pos) = pos_cnt;

  *(chan->torq_fb) = ((double) EC_READ_S16(&pd[chan->torque_fb_pdo_os])) * 10;
}

void lcec_class_ax2_write(struct lcec_slave *slave, lcec_class_ax2_chan_t *chan) {
  lcec_master_t *master = slave->master;
  uint8_t *pd = master->process_data;
  uint16_t ctrl,lctrl;
  double velo_cmd_raw;

  // write outputs
  ctrl = 0;
  if  (slave->state.operational)  ctrl |= (1 << 1);


  if (*(chan->enable)) {
    if (!(*(chan->halt))) {
      ctrl |= (1 << 0); // halt/restart
    }
    ctrl |= (1 << 2); // enable
    if (!(*(chan->drive_off))) {
      ctrl |= (1 << 3); // drive on
    }
  }
  EC_WRITE_U16(&pd[chan->ctrl_pdo_os], ctrl);

  // set velo command
  velo_cmd_raw = *(chan->velo_cmd) * chan->scale * chan->vel_output_scale;
  if (velo_cmd_raw > (double)0x7fffffff) {
    velo_cmd_raw = (double)0x7fffffff;
  }
  if (velo_cmd_raw < (double)-0x7fffffff) {
    velo_cmd_raw = (double)-0x7fffffff;
  }
  EC_WRITE_S32(&pd[chan->vel_cmd_pdo_os], (int32_t)velo_cmd_raw);

//torque cmd

 EC_WRITE_S16(&pd[chan->trq_cmd_pdo_os], (int16_t)1000);

 EC_WRITE_S16(&pd[chan->mtrq_cmd_pdo_os], (int16_t)1000);
  lctrl = 0;
 EC_WRITE_U16(&pd[chan->latch_ctrl_pdo_os],  lctrl);

}

