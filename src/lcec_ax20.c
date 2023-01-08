//
//    Copyright (C) 2011 Jan Cedrych <c.honza@volny.cz>
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
#include "lcec_ax20.h"

#include "lcec_class_enc.h"

#define ax20_RPS_FACTOR (4294967296.0 / (128.0 * 4000.0))

typedef struct
{
  

  // Digital inputs bits
  // hal_bit_t *Hdin[4];
  // hal_bit_t *Hdin_not[4];

  // Control word bits
  hal_bit_t *enable;
  hal_bit_t *inhibit;
  hal_bit_t *ctrl_bit2;
  hal_bit_t *setpoint_enable;
  hal_bit_t *clr_fault;
  hal_bit_t *homing_start;
  hal_bit_t *save;

  // Status word bits
  hal_bit_t *switch_on_not_ready;
  hal_bit_t *switch_on_disabled;
  hal_bit_t *switch_on_ready;
  hal_bit_t *switched_on;
  hal_bit_t *enabled;
  hal_bit_t *fault;
  hal_bit_t *fault_react_active;
  hal_bit_t *quick_stop_active;

  hal_bit_t *warning;
  hal_bit_t *followinq_err;
  hal_bit_t *ref_point_set;
  hal_bit_t *in_pos;
  hal_bit_t *on_lim_switch;
  hal_bit_t *ethercat_ok;
  hal_bit_t *homing_error;
  hal_bit_t *motion_task_active;

  hal_bit_t *Hpos_fb_index_enable;
  hal_bit_t *Hpos_extenc_index_enable;
  hal_bit_t *Hpos_latch;


  hal_float_t *Hpos_fb_raw;
  hal_float_t *Hpos_fb;
   
  hal_float_t *Hpos_fb_index_delta;

  hal_float_t *Hpos_folloving_err;

  hal_float_t *Hpos_extenc_raw;
  hal_float_t *Hpos_extenc;
  hal_float_t *Hpos_extenc_index_delta;

  hal_float_t *Hpos_latched;


  hal_float_t *Hvel_fb;

  hal_float_t *Hvel_extenc;

  hal_float_t *Hcur_fb;
  

  

  
  hal_float_t *Hcur_cmd;
  hal_float_t *Hvel_cmd;
  hal_float_t *Hmtrq_cmd;
  hal_float_t *Hpos_cmd;

  hal_u32_t *Hpos_fb_cnt_raw_hi;
  hal_u32_t *Hpos_fb_cnt_raw_lo;
  hal_s32_t *Hpos_fb_index_delta_count;

  hal_u32_t *Hpos_extenc_cnt_raw_hi;
  hal_u32_t *Hpos_extenc_cnt_raw_lo;
  hal_s32_t *Hpos_extenc_index_delta_count;


  hal_s32_t *Hpos_latched_count;


  hal_s32_t *Hpos_cmd_pdo_val;
  hal_s32_t *Hvel_cmd_pdo_val;
  hal_s32_t *trq_cmd_pdo_val;
  hal_s32_t *mtrq_cmd_pdo_val;





  int pos_fb_srv_cnt_last;
  long long pos_fb_index_cnt;
  long long pos_fb_cnt_last;
  long long pos_fb_cnt;

  

  hal_s32_t pos_fb_home_raw;
  hal_float_t Ppos_fb_scale;
  hal_float_t pos_fb_scale_old;

 

  int pos_extenc_srv_cnt_last;
  long long pos_extenc_index_cnt;
  long long pos_extenc_cnt_last;
  long long pos_extenc_cnt;

  hal_s32_t pos_extenc_home_raw;
 
  hal_float_t Ppos_extenc_scale;
  hal_float_t pos_extenc_scale_old;

//PDOS  1B07  fb
  unsigned int pos_pdo_os;  //32
  unsigned int pos2_pdo_os; //32
  unsigned int tork_pdo_os; //16
  unsigned int follerr_pdo_os; //32
  unsigned int status_pdo_os; //16
  unsigned int latchstatus_pdo_os; //16
  unsigned int latchpos_pdo_os;  //32  22 B total
//  pdos 1707 cnd 
  unsigned int pos_cmd_pdo_os;  //32
  unsigned int vel_cmd_pdo_os; //32
  unsigned int trq_cmd_pdo_os; //16
  unsigned int mtrq_cmd_pdo_os; //16
  unsigned int control_cmd_pdo_os; //16
  unsigned int latchcontrol_cmd_pdo_os; //16   16 B total

  hal_u32_t Pstatus_val;

/*
  hal_u32_t Platch_status_val;
  hal_u32_t latch_status_val_last;
  hal_u32_t Platch_control_val;
  hal_u32_t latch_control_val_last;

 
  hal_bit_t pos_latch_last;
*/

  hal_u32_t Pcontrol_cmd_val;
 

  double prbase_val;
  int prbase_mask;
  hal_u32_t drv_peak_current;
  //int do_init;
  //lcec_class_enc_data_t enc;
  //lcec_class_enc_data_t extenc;

} lcec_ax20_data_t;

static const lcec_pindesc_t slave_pins[] = {
 
  //control word
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, enable), "%s.%s.%s.srv_enable" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, inhibit), "%s.%s.%s.srv_inhibit" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, ctrl_bit2), "%s.%s.%s.srv_ctrl_bit2" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, setpoint_enable), "%s.%s.%s.srv_setpoint_enable" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, clr_fault), "%s.%s.%s.srv_clr_fault" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, homing_start), "%s.%s.%s.srv_homing_start" },
  { HAL_BIT, HAL_IN, offsetof(lcec_ax20_data_t, save), "%s.%s.%s.srv_save" },
  //status word
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, switch_on_not_ready), "%s.%s.%s.srv_switch_on_not_ready" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, switch_on_disabled), "%s.%s.%s.srv_switch_on_disabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, switch_on_ready), "%s.%s.%s.srv_switch_on_ready" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, switched_on), "%s.%s.%s.srv_switched_on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, enabled), "%s.%s.%s.srv_enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, fault), "%s.%s.%s.srv_fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, fault_react_active), "%s.%s.%s.srv_fault_react_active" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, quick_stop_active), "%s.%s.%s.srv_quick_stop_active" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, warning), "%s.%s.%s.srv_warning" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, followinq_err), "%s.%s.%s.srv_followinq_err" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, ref_point_set), "%s.%s.%s.srv_ref_point_set" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, in_pos), "%s.%s.%s.srv_in_pos" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, on_lim_switch), "%s.%s.%s.srv_on_lim_switch" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, ethercat_ok), "%s.%s.%s.srv_ethercat_ok" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, homing_error), "%s.%s.%s.srv_homing_error" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ax20_data_t, motion_task_active), "%s.%s.%s.srv_motion_task_active" },

  { HAL_BIT, HAL_IO, offsetof(lcec_ax20_data_t, Hpos_fb_index_enable), "%s.%s.%s.srv_pos_fb_index_enable" },
  { HAL_BIT, HAL_IO, offsetof(lcec_ax20_data_t, Hpos_extenc_index_enable), "%s.%s.%s.srv_pos_extenc_index_enable" },
  { HAL_BIT, HAL_IO, offsetof(lcec_ax20_data_t, Hpos_latch), "%s.%s.%s.srv_pos_latch" },


  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb_raw), "%s.%s.%s.srv_pos_fb_raw" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb), "%s.%s.%s.srv_pos_fb" },

  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb_index_delta), "%s.%s.%s.srv_pos_fb_index_delta" },

  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_folloving_err), "%s.%s.%s.srv_pos_folloving_err" },

  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_extenc_raw), "%s.%s.%s.srv_pos_extenc_raw" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_extenc_index_delta), "%s.%s.%s.srv_pos_extenc_index_delta" },

   { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_latched), "%s.%s.%s.srv_pos_latched" },
   { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hvel_fb), "%s.%s.%s.srv_vel_fb" },

   { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hvel_extenc), "%s.%s.%s.srv_vel_extenc" },

   { HAL_FLOAT, HAL_OUT, offsetof(lcec_ax20_data_t, Hcur_fb), "%s.%s.%s.srv_cur_fb" },

  // float inputs
   { HAL_FLOAT, HAL_IN, offsetof(lcec_ax20_data_t, Hcur_cmd), "%s.%s.%s.srv_cur_cmd" },
   { HAL_FLOAT, HAL_IN, offsetof(lcec_ax20_data_t, Hvel_cmd), "%s.%s.%s.srv_vel_cmd" },
   { HAL_FLOAT, HAL_IN, offsetof(lcec_ax20_data_t, Hmtrq_cmd), "%s.%s.%s.srv_mtrq_cmd" },
   { HAL_FLOAT, HAL_IN, offsetof(lcec_ax20_data_t, Hpos_cmd), "%s.%s.%s.srv_pos_cmd" },



  { HAL_U32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb_cnt_raw_hi), "%s.%s.%s.srv_pos_fb_cnt_raw_hi" },
  { HAL_U32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb_cnt_raw_lo), "%s.%s.%s.srv_pos_fb_cnt_raw_lo" },
  { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_fb_index_delta_count), "%s.%s.%s.srv_pos_fb_index_delta_count" },

  { HAL_U32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_extenc_cnt_raw_hi), "%s.%s.%s.srv_pos_extenc_cnt_raw_hi" },
  { HAL_U32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_extenc_cnt_raw_lo), "%s.%s.%s.srv_pos_extenc_cnt_raw_lo" },
  { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_extenc_index_delta_count), "%s.%s.%s.srv_pos_extenc_index_delta_count" },

  { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_latched_count), "%s.%s.%s.srv_pos_latched_count" },

   { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, Hpos_cmd_pdo_val), "%s.%s.%s.srv_pos_cmd_pdo_val" },
   { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, Hvel_cmd_pdo_val), "%s.%s.%s.srv_vel_cmd_pdo_val" },
   { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, trq_cmd_pdo_val), "%s.%s.%s.srv_trq_cmd_pdo_val" },
   { HAL_S32, HAL_OUT, offsetof(lcec_ax20_data_t, mtrq_cmd_pdo_val), "%s.%s.%s.srv_mtrq_cmd_pdo_val" },

  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};


static const lcec_pindesc_t slave_params[] = {
  { HAL_FLOAT, HAL_RW, offsetof(lcec_ax20_data_t, Ppos_fb_scale),     "%s.%s.%s.Ppos_fb_scale" },
  { HAL_FLOAT, HAL_RW, offsetof(lcec_ax20_data_t, Ppos_extenc_scale), "%s.%s.%s.Ppos_extenc_scale" },
  { HAL_U32, HAL_RO, offsetof(lcec_ax20_data_t,   Pstatus_val),       "%s.%s.%s.srv-Pstatus_val" },
  //{ HAL_U32, HAL_RO, offsetof(lcec_ax20_data_t,   Platch_status_val), "%s.%s.%s.srv-Platch_status_val" },
  { HAL_U32, HAL_RO, offsetof(lcec_ax20_data_t,   Pcontrol_cmd_val),  "%s.%s.%s.srv-Pcontrol_cmd_val" },
 // { HAL_U32, HAL_RO, offsetof(lcec_ax20_data_t,   Platch_control_val),"%s.%s.%s.srv-Platch_control_val" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

/*           IN PDOS                      */
static ec_pdo_entry_info_t lcec_ax20_in[] = {
    {0x6064, 0x00, 32}, // Position actual value
    {0x35c9, 0x00, 32}, // Position actual value 2
    {0x6077, 0x00, 16}, // Torque actual value
    {0x60f4, 0x00, 32}, // Following error
    {0x6041, 0x00, 16}, // Status word
    {0x2901, 0x00, 16}, // Latch status word
    {0x2902, 0x00, 32}  // Latch position
};

static ec_pdo_info_t lcec_ax20_pdos_in[] = {
    {0x1B07, 7, lcec_ax20_in}};
/**           OUT PDOS  *       ***********************/
static ec_pdo_entry_info_t lcec_ax20_out[] = {
    {0x6062, 0x00, 32}, // Position actual value
    {0x606b, 0x00, 32}, // Velocity demand value
    {0x6074, 0x00, 16}, // Torque demand value
    {0x6072, 0x00, 16}, // Max torque
    {0x6040, 0x00, 16}, // Control word
    {0x2802, 0x00, 16}  // Latch Control word
};
static ec_pdo_info_t lcec_ax20_pdos_out[] = {
    {0x1707, 6, lcec_ax20_out}};

static ec_sync_info_t lcec_ax20_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_ax20_pdos_out},
    {3, EC_DIR_INPUT, 1, lcec_ax20_pdos_in},
    {0xff}};

void lcec_ax20_read(struct lcec_slave *slave, long period);
void lcec_ax20_write(struct lcec_slave *slave, long period);

int lcec_ax20_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs)
{
  lcec_master_t *master = slave->master;
  lcec_ax20_data_t *hal_data;
  int err;
  int i;
  uint32_t tu;
  int8_t ti;
  int prbase;
  uint8_t sdo_buf[4];

  char name[HAL_NAME_LEN + 1];

  // initialize callbacks
  slave->proc_read = lcec_ax20_read;
  slave->proc_write = lcec_ax20_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_ax20_data_t))) == NULL)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  else
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "HAL mem alocated\n");

  memset(hal_data, 0, sizeof(lcec_ax20_data_t));
  slave->hal_data = hal_data;

  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "starting writing sdos\n");

  // set to cyclic synchronous velocity mode
  if (ecrt_slave_config_sdo8(slave->config, 0x6060, 0x00, (uint8_t)0x09 ) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo velo mode\n", master->name, slave->name);
  }

  // set interpolation time period
  tu = master->app_time_period;
  ti = -9;
  while ((tu % 10) == 0 || tu > 255)
  {
    tu /= 10;
    ti++;
  }


  if (ecrt_slave_config_sdo8(slave->config, 0x60C2, 0x01, (uint8_t)tu) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo ipol time period units\n", master->name, slave->name);
  }
  if (ecrt_slave_config_sdo8(slave->config, 0x60C2, 0x02, ti) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo ipol time period index\n", master->name, slave->name);
  }
  
/*
    if (ecrt_slave_config_sdo8(slave->config, 0x1C12, 0x00, 0x00) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping access\n", master->name, slave->name);
    }
    if (ecrt_slave_config_sdo16(slave->config, 0x1C12, 0x01, 0x1706) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping value\n", master->name, slave->name);
    }
    if (ecrt_slave_config_sdo8(slave->config, 0x1C12, 0x00, 0x01) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping access\n", master->name, slave->name);
    }
    if (ecrt_slave_config_sdo8(slave->config, 0x1C13, 0x00, 0x00) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping access\n", master->name, slave->name);
    }
    if (ecrt_slave_config_sdo16(slave->config, 0x1C13, 0x01, 0x1B07) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping value\n", master->name, slave->name);
    }
    if (ecrt_slave_config_sdo8(slave->config, 0x1C13, 0x00, 0x01) != 0) {
      rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo fixed pdo mapping access\n", master->name, slave->name);
    }

  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "writing sdos finished - no sdo\n");
*/
  // read prbase value
  if (lcec_read_sdo(slave, 0x35D1, 0x01, sdo_buf, 4))
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Prbase read error\n");
    return -EIO;
  }

  prbase = EC_READ_S32(sdo_buf);

  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Prbase %d\n", prbase);

  hal_data->prbase_val = (double)(1 << prbase);
  hal_data->prbase_mask = (1 << prbase) - 1;

  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Prbase value %d\n", (int)hal_data->prbase_val);

  slave->sync_info = lcec_ax20_syncs;


  // initialize POD entries
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6064, 0x00, &hal_data->pos_pdo_os, NULL); //32
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x35C9, 0x00, &hal_data->pos2_pdo_os, NULL); //32
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6077, 0x00, &hal_data->tork_pdo_os, NULL); //16
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60F4, 0x00, &hal_data->follerr_pdo_os, NULL); //32
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_pdo_os, NULL); //16
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2901, 0x00, &hal_data->latchstatus_pdo_os, NULL); //16
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2902, 0x00, &hal_data->latchpos_pdo_os, NULL); //32

  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6062, 0x00, &hal_data->pos_cmd_pdo_os, NULL); //32
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x606b, 0x00, &hal_data->vel_cmd_pdo_os, NULL); //32
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6074, 0x00, &hal_data->trq_cmd_pdo_os, NULL); //16 
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6072, 0x00, &hal_data->mtrq_cmd_pdo_os, NULL); //16
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->control_cmd_pdo_os, NULL); //16
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x2802, 0x00, &hal_data->latchcontrol_cmd_pdo_os, NULL); //16
  
  
  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }
  /*
  // init subclasses
  if ((err = class_enc_init(slave, &hal_data->enc, 32, "enc")) != 0) {
    return err;
  }
  if ((err = class_enc_init(slave, &hal_data->extenc, 32, "extenc")) != 0) {
    return err;
  }
 */
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init start finish\n");
  /* for (i = 0; i < 4; i++)
  {
    *(hal_data->Hdin[i]) = 0;
    *(hal_data->Hdin_not[i]) = 0;
  } */

  hal_data->pos_fb_srv_cnt_last = 0;
  hal_data->pos_fb_index_cnt = 0;
  hal_data->pos_fb_cnt_last = 0;
  hal_data->pos_fb_cnt = 0;
  //*(hal_data->Hpos_fb_cnt_raw_hi) = 0;
  //*(hal_data->Hpos_fb_cnt_raw_lo) = 0;
  hal_data->pos_fb_home_raw = 0;
  //*(hal_data->Hpos_fb_raw) = 0;
  //*(hal_data->Hpos_fb) = 0;
  hal_data->Ppos_fb_scale = 0;
  hal_data->pos_fb_scale_old = 0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 4 finish\n");
 //*(hal_data->Hpos_fb_index_delta_count) = 0;
 //*(hal_data->Hpos_fb_index_delta) = 0;

 /*(hal_data->Hpos_folloving_err) = 0;

 //*(hal_data->Hvel_fb) = 0;
*/
  hal_data->pos_extenc_srv_cnt_last = 0;
  hal_data->pos_extenc_index_cnt = 0;
  hal_data->pos_extenc_cnt_last = 0;
  hal_data->pos_extenc_cnt = 0;
  /*(hal_data->Hpos_extenc_cnt_raw_hi) = 0;
  *(hal_data->Hpos_extenc_cnt_raw_lo) = 0;
  */
  hal_data->pos_extenc_home_raw = 0;
  /*(hal_data->Hpos_extenc_raw) = 0.0;
  *(hal_data->Hpos_extenc) = 0.0;
  */
  hal_data->Ppos_extenc_scale = 1.0;
  hal_data->pos_extenc_scale_old = 2.0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 5 finish\n");
  /*(hal_data->Hpos_extenc_index_delta_count) = 0;
  *(hal_data->Hpos_extenc_index_delta) = 0.0;
  *(hal_data->Hvel_extenc) = 0.0;
  *(hal_data->Hpos_latched_count) = 0;
  *(hal_data->Hpos_latched) = 0.0;
  *(hal_data->Hcur_fb) = 0.0;
*/
  hal_data->Pstatus_val = 0;
  //hal_data->Platch_status_val = 0;
 // hal_data->latch_status_val_last = 0;
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 6 finish\n");
  /*(hal_data->Hpos_cmd_pdo_val) = 0.0;
  *(hal_data->Hvel_cmd_pdo_val) = 0.0;
  *(hal_data->trq_cmd_pdo_val) = 0.0;
  *(hal_data->mtrq_cmd_pdo_val) = 0.0;
  */
  hal_data->Pcontrol_cmd_val = 0;
  // hal_data->Platch_control_val = 0;
  //hal_data->latch_control_val_last = 0;
  //hal_data->pos_latch_last = 0.0;
  hal_data->prbase_val = 0;
  hal_data->prbase_mask = 0;
  hal_data->drv_peak_current = 6;
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 7 finish\n");
  return 0;
}

void lcec_ax20_check_scales(lcec_ax20_data_t *hal_data)
{

  if (hal_data->Ppos_fb_scale != hal_data->pos_fb_scale_old)
  {
    if ((hal_data->Ppos_fb_scale < 1e-20) && (hal_data->Ppos_fb_scale > -1e-20))
    {
      hal_data->Ppos_fb_scale = 1.0;
    }
    hal_data->pos_fb_scale_old = hal_data->Ppos_fb_scale;
  }

  if (hal_data->Ppos_extenc_scale != hal_data->pos_extenc_scale_old)
  {
    if ((hal_data->Ppos_extenc_scale < 1e-20) && (hal_data->Ppos_extenc_scale > -1e-20))
    {
      hal_data->Ppos_extenc_scale = 1.0;
    }
    hal_data->pos_extenc_scale_old = hal_data->Ppos_extenc_scale;
  }

}

void lcec_ax20_read(struct lcec_slave *slave, long period)
{
  lcec_master_t *master = slave->master;
  lcec_ax20_data_t *hal_data = (lcec_ax20_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  int i;

  int pos_cnt;
  long long pos_cnt_diff;
  int16_t curent;

  if (!slave->state.operational)
  {
    return;
  }
  
  lcec_ax20_check_scales(hal_data);

  // update position counter
  pos_cnt = EC_READ_S32(&pd[hal_data->pos_pdo_os]);
  pos_cnt_diff = (long long)pos_cnt - (long long)hal_data->pos_fb_srv_cnt_last;
  if (pos_cnt_diff > 2147483648LL)
    pos_cnt_diff -= 4294967296LL;
  if (pos_cnt_diff < -2147483648LL)
    pos_cnt_diff += 4294967296LL;
  hal_data->pos_fb_cnt += pos_cnt_diff;
  hal_data->pos_fb_srv_cnt_last = pos_cnt;

  if (*hal_data->Hpos_fb_index_enable == true)
  {
    if ((hal_data->pos_fb_cnt & ~((long long)hal_data->prbase_mask)) > (hal_data->pos_fb_cnt_last & ~((long long)hal_data->prbase_mask)))
    {
      *hal_data->Hpos_fb_index_enable = false;
      hal_data->pos_fb_index_cnt = hal_data->pos_fb_cnt & ~((long long)hal_data->prbase_mask);
      *hal_data->Hpos_fb_index_delta_count = hal_data->pos_fb_cnt - hal_data->pos_fb_index_cnt;
      *hal_data->Hpos_fb_index_delta = *hal_data->Hpos_fb_index_delta_count * hal_data->Ppos_extenc_scale / hal_data->prbase_val;
    }
    if ((hal_data->pos_fb_cnt & ~((long long)hal_data->prbase_mask)) < (hal_data->pos_fb_cnt_last & ~((long long)hal_data->prbase_mask)))
    {
      *hal_data->Hpos_fb_index_enable = false;
      hal_data->pos_fb_index_cnt = hal_data->pos_fb_cnt_last & ~((long long)hal_data->prbase_mask);
      *hal_data->Hpos_fb_index_delta_count = hal_data->pos_fb_cnt - hal_data->pos_fb_index_cnt;
      *hal_data->Hpos_fb_index_delta = *hal_data->Hpos_fb_index_delta_count * hal_data->Ppos_extenc_scale / hal_data->prbase_val;
    }
  }

  *hal_data->Hvel_fb = ((double)(hal_data->pos_fb_cnt - hal_data->pos_fb_cnt_last) * hal_data->Ppos_fb_scale / hal_data->prbase_val) / ((double)(master->app_time_period) / 1e9);

  hal_data->pos_fb_cnt_last = hal_data->pos_fb_cnt;

  *(hal_data->Hpos_fb_cnt_raw_hi) = (hal_data->pos_fb_cnt >> 32) & 0xffffffff;
  *(hal_data->Hpos_fb_cnt_raw_lo) = (hal_data->pos_fb_cnt) & 0xffffffff;
  *(hal_data->Hpos_fb_raw) = (double)(hal_data->pos_fb_cnt) * hal_data->Ppos_fb_scale / hal_data->prbase_val;
  *(hal_data->Hpos_fb) = (double)(hal_data->pos_fb_cnt - hal_data->pos_fb_index_cnt) * hal_data->Ppos_fb_scale / hal_data->prbase_val;

  pos_cnt = EC_READ_S32(&pd[hal_data->follerr_pdo_os]);
  *(hal_data->Hpos_folloving_err) = (double)(pos_cnt)*  hal_data->Ppos_fb_scale / hal_data->prbase_val;

  pos_cnt = EC_READ_S32(&pd[hal_data->pos2_pdo_os]);
  pos_cnt_diff = (long long)pos_cnt - (long long)hal_data->pos_extenc_srv_cnt_last;
  if (pos_cnt_diff > 2147483648LL)
    pos_cnt_diff -= 4294967296LL;
  if (pos_cnt_diff < -2147483648LL)
    pos_cnt_diff += 4294967296LL;
  hal_data->pos_extenc_cnt += pos_cnt_diff;
  hal_data->pos_extenc_srv_cnt_last = pos_cnt;

  if (*hal_data->Hpos_extenc_index_enable == true)
  {
    if ((hal_data->pos_extenc_cnt & ~((long long)hal_data->prbase_mask)) > (hal_data->pos_extenc_cnt_last & ~((long long)hal_data->prbase_mask)))
    {
      *hal_data->Hpos_extenc_index_enable = false;
      hal_data->pos_extenc_index_cnt = hal_data->pos_extenc_cnt & ~((long long)hal_data->prbase_mask);
      *hal_data->Hpos_extenc_index_delta_count = hal_data->pos_extenc_cnt - hal_data->pos_extenc_index_cnt;
      *hal_data->Hpos_extenc_index_delta = *hal_data->Hpos_extenc_index_delta_count * hal_data->Ppos_extenc_scale / hal_data->prbase_val;
    }
    if ((hal_data->pos_extenc_cnt & ~((long long)hal_data->prbase_mask)) < (hal_data->pos_extenc_cnt_last & ~((long long)hal_data->prbase_mask)))
    {
      *hal_data->Hpos_extenc_index_enable = false;
      hal_data->pos_extenc_index_cnt = hal_data->pos_extenc_cnt_last & ~((long long)hal_data->prbase_mask);
      *hal_data->Hpos_extenc_index_delta_count = hal_data->pos_extenc_cnt - hal_data->pos_extenc_index_cnt;
      *hal_data->Hpos_extenc_index_delta = *hal_data->Hpos_extenc_index_delta_count * hal_data->Ppos_extenc_scale / hal_data->prbase_val;
    }
  }

  *(hal_data->Hvel_extenc) = ((double)(hal_data->pos_extenc_cnt - hal_data->pos_extenc_cnt_last) * hal_data->Ppos_fb_scale / hal_data->prbase_val) / ((double)(master->app_time_period) / 1e9);

  hal_data->pos_extenc_cnt_last = hal_data->pos_extenc_cnt;

  *(hal_data->Hpos_extenc_cnt_raw_hi) = (hal_data->pos_extenc_cnt >> 32) & 0xffffffff;
  *(hal_data->Hpos_extenc_cnt_raw_lo) = (hal_data->pos_extenc_cnt) & 0xffffffff;
  *(hal_data->Hpos_extenc_raw) = (double)(hal_data->pos_extenc_cnt) * hal_data->Ppos_extenc_scale / hal_data->prbase_val;
  *(hal_data->Hpos_extenc) = (double)(hal_data->pos_extenc_cnt - hal_data->pos_extenc_index_cnt) * hal_data->Ppos_extenc_scale / hal_data->prbase_val;

  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  hal_data->Pstatus_val = status;

  *(hal_data->warning) = (status >> 7) & 0x01;
  *(hal_data->followinq_err) = (status >> 8) & 0x01;
  *(hal_data->ref_point_set) = (status >> 9) & 0x01;
  *(hal_data->in_pos) = (status >> 10) & 0x01;
  *(hal_data->on_lim_switch) = (status >> 11) & 0x01;
  *(hal_data->ethercat_ok) = (status >> 12) & 0x01;
  *(hal_data->homing_error) = (status >> 13) & 0x01;
  *(hal_data->motion_task_active) = (status >> 15) & 0x01;

  *(hal_data->switch_on_not_ready) = (status & 0x40) == 0x40;
  *(hal_data->switch_on_disabled) = (status & 0x40) == 0x40;
  *(hal_data->switch_on_ready) = (status & 0x01) == 0x01;
  *(hal_data->switched_on) = (status & 0x23) == 0x23;
  *(hal_data->enabled) = (status & 0x27) == 0x27;
  *(hal_data->fault) = (status & 0x28) == 0x28;
  *(hal_data->fault_react_active) = (status & 0x0F) == 0x0F;
  *(hal_data->quick_stop_active) = (status & 0x03) == 0x03;

  curent = EC_READ_S16(&pd[hal_data->tork_pdo_os]);
  *(hal_data->Hcur_fb) = (double)(curent) / 3280 * hal_data->drv_peak_current;

  status = EC_READ_U16(&pd[hal_data->latchstatus_pdo_os]);
  pos_cnt = EC_READ_S32(&pd[hal_data->latchpos_pdo_os]);
  /*
  for (i = 0; i < 4; i++)
  {
    
    *(hal_data->Hdin[3 - i]) = (status & (0x1000 << i)) == (0x1000 << i);
    *(hal_data->Hdin_not[3 - i]) = !*(hal_data->Hdin[3 - i]);
    
  }
  
  
      if ((hal_data->Platch_status_val & 0x001f) != (hal_data->latch_control_val_last & 0x001f)){
          switch((hal_data->Platch_status_val & 0x001f)){
              case 0x0010:
                  *(hal_data->Hpos_latched_count) = pos_cnt;
                  *(hal_data->Hpos_latched) = (double)(pos_cnt) * hal_data->Ppos_fb_scale / hal_data->prbase_val;
                  break;
          default:
              break;
          }
      }
      switch((hal_data->Platch_status_val & 0x0F00)){
          case 0x0500:
              *(hal_data->Hpos_latched_count) = pos_cnt;
              *(hal_data->Hpos_latched) = (double)(pos_cnt) * hal_data->Ppos_fb_scale / hal_data->prbase_val;
              break;
      default:
          break;
      }
      if((hal_data->latch_status_val_last & 0x0F00) != (status & 0x0F00)){
      }
      hal_data->latch_status_val_last = hal_data->Platch_status_val;
*/  

 
}

void lcec_ax20_write(struct lcec_slave *slave, long period)
{
  lcec_master_t *master = slave->master;
  lcec_ax20_data_t *hal_data = (lcec_ax20_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control , lcontrol = 0;
  int16_t tork;
  int16_t mtork;
  int32_t speed;
  int32_t position;
  
  lcec_ax20_check_scales(hal_data);

  control = 0x00;

  if (*hal_data->enable)
  {
    control |= (1 << 0);
  }
  if (*hal_data->inhibit)
  {
    control |= (1 << 1);
  }
  if (*hal_data->ctrl_bit2)
  {
    control |= (1 << 2);
  }
  if (*hal_data->setpoint_enable)
  {
    control |= (1 << 3);
  }
  if (*hal_data->clr_fault)
  {
    control |= (1 << 7);
  }
  if (*hal_data->homing_start)
  {
    control |= (1 << 11);
  }
  if (*hal_data->save)
  {
    control |= (1 << 12);
  }
  EC_WRITE_U16(&pd[hal_data->control_cmd_pdo_os], control);
  hal_data->Pcontrol_cmd_val = control;

  speed = *(hal_data->Hvel_cmd) / hal_data->Ppos_fb_scale * ax20_RPS_FACTOR;
  EC_WRITE_S32(&pd[hal_data->vel_cmd_pdo_os], speed);
  *(hal_data->Hvel_cmd_pdo_val) = speed;

  // position = (long long)((*hal_data->Hpos_cmd) / hal_data->Ppos_fb_scale * hal_data->prbase_val) + hal_data->pos_fb_index_cnt;
  position = (long long)(*(hal_data->Hpos_cmd) / hal_data->Ppos_fb_scale * hal_data->prbase_val);
  EC_WRITE_S32(&pd[hal_data->pos_cmd_pdo_os], position);
  *hal_data->Hpos_cmd_pdo_val = position;

  EC_WRITE_U16(&pd[hal_data->latchcontrol_cmd_pdo_os], lcontrol);
   // hal_data->Platch_control_val = lcontrol;
 

  *(hal_data->trq_cmd_pdo_val) = (int16_t) ( *(hal_data->Hcur_cmd)  *  3280 / hal_data->drv_peak_current);
  tork = *(hal_data->trq_cmd_pdo_val);
  EC_WRITE_S16(&pd[hal_data->trq_cmd_pdo_os], tork);

  *(hal_data->mtrq_cmd_pdo_val) = (int16_t) ( *(hal_data->Hmtrq_cmd)  *  3280 / hal_data->drv_peak_current);
  mtork = *(hal_data->mtrq_cmd_pdo_val);
  EC_WRITE_S16(&pd[hal_data->mtrq_cmd_pdo_os], mtork);

}
