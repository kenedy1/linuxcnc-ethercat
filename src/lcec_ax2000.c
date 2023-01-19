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

#define LATCH_ENABLED 1

#define AX2000_FAULT_AUTORESET_DELAY_NS 100000000LL
#define OPMODE_POSITION 0x08
#define OPMODE_VELOCITY 0x09
#define OPMODE_TORQUE   0x0A
#define OPMODE_HOING    0x06

#define TORQUE_SCALE  3280           //max driver curent 6A
#define VELO_SCALE_RPM  (double)(2^32(128*4000*60)) // velo scale 1 RPM
#define POS_SCALE_1T    2^20      // pos scale 1 turn


//latch control world mask
#define  LATCH_CWB_EN_EXT_1_P   0x0001
#define  LATCH_CWB_RD_EXT_1_P   0x0100
//latch status worl mask
#define  LATCH_VALID_SWB_EXTLT_1_P    0x0001
#define  LATCH_ACK_VAL_SWB_EXTLT_1_P  0x0100
#define  LATCH_SWB_DI1                0x8000

typedef struct {
  hal_float_t *pos_fb;
  hal_float_t *opmode_fb;
  hal_float_t *vel_rpm_cmd;

  hal_bit_t   *stat_switch_on_ready;
  hal_bit_t   *stat_switched_on;
  hal_bit_t   *stat_op_enabled;
  hal_bit_t   *stat_fault;
  hal_bit_t   *stat_volt_enabled;
  hal_bit_t   *stat_quick_stoped;
  hal_bit_t   *stat_switch_on_disabled;
  hal_bit_t   *stat_warning;
  hal_bit_t   *stat_remote;
  hal_bit_t   *stat_at_speed;

  hal_bit_t   *quick_stop;
  hal_bit_t   *enable;
  hal_bit_t   *fault_reset;
  hal_bit_t   *halt;

  hal_s32_t   *mode_op_display;

  hal_float_t *act_current;
  hal_u32_t   *warn_code;
  hal_u32_t   *error_code;
  hal_float_t *foll_err;

  hal_u32_t   *cmd_torq;
  hal_u32_t   *max_torq;

  hal_bit_t   auto_fault_reset;
  hal_float_t vel_scale;

  double      vel_scale_old;
  double      vel_scale_rcpt;

  unsigned int fb_pos_pdo_os;
  unsigned int fb_torq_pdo_os;
  unsigned int fb_follerr_pdo_os;
  unsigned int fb_opmode_pdo_os;
  unsigned int gap;
  unsigned int status_pdo_os;
  unsigned int latch_status_pdo_os;
  unsigned int fb_pos_lat_pdo_os;

  unsigned int cmd_pos_pdo_os;
  unsigned int cmd_vel_pdo_os;
  unsigned int cmd_torq_pdo_os;
  unsigned int max_torq_pdo_os;
  unsigned int cmd_opmode_pdo_os;
  unsigned int gap1;
  unsigned int controlW_pdo_os;
  unsigned int lat_controlW_pdo_os;


  hal_bit_t enable_old;
  hal_bit_t internal_fault;

  long long auto_fault_reset_delay;

} lcec_ax2000_data_t;
// HAL pins 
static const lcec_pindesc_t slave_pins[] = {
  { HAL_FLOAT, HAL_OUT,   offsetof(lcec_ax2000_data_t, pos_fb),           "%s.%s.%s.pos-fb" },
  { HAL_32,    HAL_OUT,   offsetof(lcec_ax2000_data_t, opmode_fb),       "%s.%s.%s.opmode-fb" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_switch_on_ready), "%s.%s.%s.stat-switch-on-ready" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_switched_on),     "%s.%s.%s.stat-switched-on" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_op_enabled),      "%s.%s.%s.stat-op-enabled" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_fault),           "%s.%s.%s.stat-fault" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_volt_enabled),    "%s.%s.%s.stat-volt-enabled" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_quick_stoped),    "%s.%s.%s.stat-quick-stoped" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_switch_on_disabled), "%s.%s.%s.stat-switch-on-disabled" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_warning),         "%s.%s.%s.stat-warning" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_remote),          "%s.%s.%s.stat-remote" },
  { HAL_BIT,   HAL_OUT,   offsetof(lcec_ax2000_data_t, stat_at_speed),        "%s.%s.%s.stat-at-speed" },
  { HAL_FLOAT, HAL_OUT,   offsetof(lcec_ax2000_data_t, act_current),          "%s.%s.%s.act-current" },
  { HAL_U32,   HAL_OUT,   offsetof(lcec_ax2000_data_t, warn_code),            "%s.%s.%s.warn-code" },
  { HAL_U32,   HAL_OUT,   offsetof(lcec_ax2000_data_t, error_code),           "%s.%s.%s.error-code" },
  { HAL_FLOAT, HAL_OUT,   offsetof(lcec_ax2000_data_t, foll_err),           "%s.%s.%s.foll-err" },
  { HAL_BIT,   HAL_IN,    offsetof(lcec_ax2000_data_t, quick_stop),           "%s.%s.%s.quick-stop" },
  { HAL_BIT,   HAL_IN,    offsetof(lcec_ax2000_data_t, enable),               "%s.%s.%s.enable" },
  { HAL_BIT,   HAL_IN,    offsetof(lcec_ax2000_data_t, fault_reset),          "%s.%s.%s.fault-reset" },
  { HAL_BIT,   HAL_IN,    offsetof(lcec_ax2000_data_t, halt),                 "%s.%s.%s.halt" },
  { HAL_FLOAT, HAL_IN,    offsetof(lcec_ax2000_data_t, vel_rpm_cmd),          "%s.%s.%s.vel-rpm-cmd" },
  { HAL_U32,   HAL_IN,    offsetof(lcec_ax2000_data_t, cmd_torq),          "%s.%s.%s.cmd-torq" },
  { HAL_U32,   HAL_IN,    offsetof(lcec_ax2000_data_t, max_torq),        "%s.%s.%s.max-torq" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};
// HAL parameters
static const lcec_pindesc_t slave_params[] = {
  { HAL_BIT,    HAL_RW, offsetof(lcec_ax2000_data_t, auto_fault_reset),  "%s.%s.%s.auto-fault-reset" },
  { HAL_FLOAT,  HAL_RW, offsetof(lcec_ax2000_data_t, vel_scale),         "%s.%s.%s.vel-scale" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};


//from servo 0x1b08
static ec_pdo_entry_info_t lcec_ax2000_in[] = {
  {0x6064, 0x00, 32}, // Position fedbeck     DINT
  {0x6077, 0x00, 16}, // Torque feedback      INT
  {0x60f4, 0x00, 32}, // Following error     DINT
  {0x6061, 0x00, 8},  // Operantion mode     USINT
  {0x0000, 0x00, 8},  // Gap
  {0x6041, 0x00, 16}, // Status word         UINT
  {0x2901, 0x00, 16}, // Latch status        UINT
  {0x2902, 0x00, 32}  // Latch position      DINT
};
//to servo 0x1708
static ec_pdo_entry_info_t lcec_ax2000_out[] = {
  {0x6062, 0x00, 32},   // Cmd position         DINT
  {0x606b, 0x00, 32},   // Cmd Velocity         DINT
  {0x6074, 0x00, 16},   // Cmd Torq             INT
  {0x6072, 0x00, 16},   // Cmd Torq max         UINT
  {0x6060, 0x00, 8},    // Operat mode cmd      USINT
  {0x0000, 0x00, 8},    // Gap
  {0x6040, 0x00,  16},  // Control word       UINT
  {0x2802, 0x00,  16}   // Control latch word  UINT 
 
};

static ec_pdo_info_t lcec_ax2000_pdos_out[] = {
     {0x1708, 8, lcec_ax2000_out}
};

static ec_pdo_info_t lcec_ax2000_pdos_in[] = {
     {0x1b08, 8, lcec_ax2000_in}
};



static ec_sync_info_t lcec_ax2000_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,EC_WD_ENABLE},
    {1, EC_DIR_INPUT,  0, NULL,EC_WD_ENABLE},
    {2, EC_DIR_OUTPUT, 1, lcec_ax2000_pdos_out,EC_WD_ENABLE},  //0x1707
    {3, EC_DIR_INPUT,  1, lcec_ax2000_pdos_in,EC_WD_ENABLE},  //0x1b07
    {0xff}
};

void lcec_ax2000_check_scales(lcec_ax2000_data_t *hal_data);

void lcec_ax2000_read (struct lcec_slave *slave, long period);
void lcec_ax2000_write(struct lcec_slave *slave, long period);

//init 
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

  // initialize sync info  
  slave->sync_info = lcec_ax2000_syncs;

  //init PDO entries
  //0x1b08
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6064, 0x00, &hal_data->fb_pos_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6077, 0x00, &hal_data->fb_torq_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60f4, 0x00, &hal_data->fb_follerr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6061, 0x00, &hal_data->fb_opmode_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x0000, 0x00, &hal_data->gap, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2901, 0x00, &hal_data->latch_status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2902, 0x00, &hal_data->fb_pos_lat_pdo_os, NULL);
  //0x1708
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6062, 0x00, &hal_data->cmd_pos_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x606b, 0x00, &hal_data->cmd_vel_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6074, 0x00, &hal_data->cmd_torq_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6072, 0x00, &hal_data->max_torq_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6060, 0x00, &hal_data->cmd_opmode_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x0000, 0x00, &hal_data->ga1, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->controlW_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2802, 0x00, &hal_data->lat_controlW_pdo_os, NULL);

  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  
  // initialize variables
  hal_data->enable_old = 0;
  hal_data->internal_fault = 0;

  hal_data->auto_fault_reset = 1;
  hal_data->vel_scale = 1.0;
  hal_data->vel_scale_old = hal_data->vel_scale + 1.0;
  hal_data->vel_scale_rcpt = 1.0;
  return 0;
}

 void lcec_ax2000_check_scales(lcec_ax2000_data_t *hal_data) {
  // check for change in scale value
  if (hal_data->vel_scale != hal_data->vel_scale_old) {
    // scale value has changed, test and update it
    if ((hal_data->vel_scale < 1e-20) && (hal_data->vel_scale > -1e-20)) {
      // value too small, divide by zero is a bad thing
      hal_data->vel_scale = 1.0;
    }
    // save new scale to detect future changes
    hal_data->vel_scale_old = hal_data->vel_scale;
    // we actually want the reciprocal
    hal_data->vel_scale_rcpt = 1.0 / hal_data->vel_scale;
  }
}

void lcec_ax2000_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ax2000_data_t *hal_data = (lcec_ax2000_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status,error,lstatus;
  int8_t opmode_in;
  int32_t speed_raw;
  int32_t pos_fb , pos_fb2;
  double rpm;

  // check for change in scale value
  lcec_ax2000_check_scales(hal_data);

  // read current
  *(hal_data->act_current) = (double) EC_READ_U16(&pd[hal_data->fb_torq_pdo_os]) / 100.0;
  // follerr
  *(hal_data->foll_err) = (double) EC_READ_S32(&pd[hal_data->fb_follerr_pdo_os]) / 10.0;
  // read warn and error code
  pos_fb    = EC_READ_S32(&pd[hal_data->fb_pos_pdo_os]);
  *(hal_data->pos_fb)= (double) pos_fb / 10000;

  // pos_fb2    = EC_READ_S32(&pd[hal_data->fb_pos2_pdo_os]);
  *(hal_data->opmode_fb)= (double) pos_fb2 / 10000;

  *(hal_data->error_code) = 0; //low byte
  *(hal_data->warn_code) =  8; //high byte

  // wait for slave to be operational
  if (!slave->state.operational) {
    *(hal_data->stat_switch_on_ready)    = 0;
    *(hal_data->stat_switched_on)        = 0;
    *(hal_data->stat_op_enabled)         = 0;
    *(hal_data->stat_fault)              = 1;
    *(hal_data->stat_volt_enabled)       = 0;
    *(hal_data->stat_quick_stoped)       = 0;
    *(hal_data->stat_switch_on_disabled) = 0;
    *(hal_data->stat_warning)            = 0;
    *(hal_data->stat_remote)             = 0;
    *(hal_data->stat_at_speed)           = 0;
    return;
  }

  // read Modes of Operation
  opmode_in = EC_READ_S8(&pd[hal_data->fb_opmode_pdo_os]);

  // read status word
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  *(hal_data->stat_switch_on_ready)    = (status >> 0) & 1;
  *(hal_data->stat_switched_on)        = (status >> 1) & 1;
  *(hal_data->stat_op_enabled)         = (status >> 2) & 1;
//  *(hal_data->stat_fault)             = (status >> 3) & 1;
  hal_data->internal_fault             = (status >> 3) & 0x01;
  *(hal_data->stat_volt_enabled)       = (status >> 4) & 1;
  *(hal_data->stat_quick_stoped)       = (status >> 5) & 1;
  *(hal_data->stat_switch_on_disabled) = (status >> 6) & 1;
  *(hal_data->stat_warning)            = (status >> 7) & 1;
  *(hal_data->stat_remote)             = (status >> 9) & 1;
  *(hal_data->stat_at_speed)           = (status >> 10) & 0x01;

  // set fault if op mode is wrong
  // if (opmode_in != OPMODE_VELOCITY) {
  //  hal_data->internal_fault  = 1;
  //  rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "AX2000 slave %s.%s not sending velo mode\n", master->name, slave->name);
  // }
  lstatus = EC_READ_U16(&pd[hal_data->latch_status_pdo_os]);
  // update fault output
  if (hal_data->auto_fault_reset_delay > 0) {
    hal_data->auto_fault_reset_delay -= period;
    *(hal_data->stat_fault) = 0;
  } else {
    *(hal_data->stat_fault) = hal_data->internal_fault;
  }

  // fb position
  speed_raw = EC_READ_S32(&pd[hal_data->fb_pos_lat_pdo_os]);
 // rpm = (double)speed_raw * hal_data->vel_scale_rcpt;
 // *(hal_data->vel_fb_rpm) = rpm;
 // *(hal_data->vel_fb_rpm_abs) = fabs(rpm);


}

void lcec_ax2000_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ax2000_data_t *hal_data = (lcec_ax2000_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control, lcontrol;
  double speed_raw;
  int8_t opmode;
  int enable_edge;

  // check for change in scale value
  lcec_ax2000_check_scales(hal_data);

  //set drive OP mode
  opmode = OPMODE_VELOCITY;
  EC_WRITE_S8(&pd[hal_data->mode_op_pdo_os], (int8_t)opmode);

  // check for enable edge
  enable_edge = *(hal_data->enable) && !hal_data->enable_old;
  hal_data->enable_old = *(hal_data->enable);


  // write control register
  control = (!*(hal_data->fault_reset) << 2); // quick stop

  if (*(hal_data->stat_fault)) {
    if (*(hal_data->fault_reset)) {
      control |= (1 << 7); // fault reset
    }
    if (hal_data->auto_fault_reset && enable_edge) {
      hal_data->auto_fault_reset_delay = AX2000_FAULT_AUTORESET_DELAY_NS;
      control |= (1 << 7); // fault reset
    }
  } else {
    if (*(hal_data->enable)) {
      control |= (1 << 1); // enable voltage
      if (*(hal_data->stat_switch_on_ready)) {
        control |= (1 << 0); // switch on
        if (*(hal_data->stat_switched_on)) {
          control |= (1 << 3); // enable op
        }
      }
    }
    //set velo control bits
    if (*(hal_data->stat_op_enabled)) {
      control |= (1 << 4); // rfg enable
      control |= (1 << 5); // rfg unlock
      control |= (1 << 6); // rfg use ref
    }
  }

  //halt
  control |= (*(hal_data->halt) << 8); // halt
  EC_WRITE_U16(&pd[hal_data->controlW_pdo_os], control);
  

  //write ramp times
  EC_WRITE_S32(&pd[hal_data->cmd_pos_pdo_os], 0);
  EC_WRITE_S16(&pd[hal_data->cmd_torq_pdo_os], *(hal_data->cmd_torq));
  EC_WRITE_S16(&pd[hal_data->max_torq_pdo_os], *(hal_data->max_torq));

  // set RPM
  speed_raw = *(hal_data->vel_rpm_cmd) * hal_data->vel_scale;
  if (speed_raw > (double)0x7fff) {
    speed_raw = (double)0x7fff;
  }
  if (speed_raw < (double)-0x7fff) {
    speed_raw = (double)-0x7fff;
  }
  EC_WRITE_S32(&pd[hal_data->cmd_vel_pdo_os], (int32_t)speed_raw);


  

  lcontrol = 0;
  EC_WRITE_U16(&pd[hal_data->lat_controlW_pdo_os], lcontrol);
}

