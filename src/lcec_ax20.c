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

#define ax20_RPS_FACTOR (4294967296.0 / (128.0 * 4000.0))

typedef struct
{
  int do_init;

  // Digital inputs bits
  hal_bit_t *Hdin[4];
  hal_bit_t *Hdin_not[4];

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

  hal_float_t *Hcur_cmd;
  hal_float_t *Hvel_cmd;
  hal_float_t *Htrq_cmd;
  hal_float_t *Hmtrq_cmd;
  hal_float_t *Hpos_cmd;


  int pos_fb_srv_cnt_last;
  long long pos_fb_index_cnt;
  long long pos_fb_cnt_last;
  long long pos_fb_cnt;
  hal_u32_t *Hpos_fb_cnt_raw_hi;
  hal_u32_t *Hpos_fb_cnt_raw_lo;
  hal_s32_t pos_fb_home_raw;
  hal_float_t *Hpos_fb_raw;
  hal_float_t *Hpos_fb;
  hal_float_t Ppos_fb_scale;
  hal_float_t pos_fb_scale_old;

  hal_s32_t *Hpos_fb_index_delta_count;
  hal_float_t *Hpos_fb_index_delta;

  hal_float_t *Hpos_folloving_err;

  hal_float_t *Hvel_fb;

  int pos_extenc_srv_cnt_last;
  long long pos_extenc_index_cnt;
  long long pos_extenc_cnt_last;
  long long pos_extenc_cnt;
  hal_u32_t *Hpos_extenc_cnt_raw_hi;
  hal_u32_t *Hpos_extenc_cnt_raw_lo;
  hal_s32_t pos_extenc_home_raw;
  hal_float_t *Hpos_extenc_raw;
  hal_float_t *Hpos_extenc;
  hal_float_t Ppos_extenc_scale;
  hal_float_t pos_extenc_scale_old;

  hal_s32_t *Hpos_extenc_index_delta_count;
  hal_float_t *Hpos_extenc_index_delta;

  hal_float_t *Hvel_extenc;

  hal_s32_t *Hpos_latched_count;
  hal_float_t *Hpos_latched;

  hal_float_t *Hcur_fb;

  unsigned int pos_pdo_os;
  unsigned int pos2_pdo_os;
  unsigned int tork_pdo_os;
  unsigned int follerr_pdo_os;
  unsigned int status_pdo_os;
  unsigned int latchstatus_pdo_os;
  unsigned int latchpos_pdo_os;

  unsigned int pos_cmd_pdo_os;
  unsigned int vel_cmd_pdo_os;
  unsigned int trq_cmd_pdo_os;
  unsigned int mtrq_cmd_pdo_os;
  unsigned int control_cmd_pdo_os;
  unsigned int latchcontrol_cmd_pdo_os;

  hal_u32_t Pstatus_val;
  hal_u32_t Platch_status_val;
  hal_u32_t latch_status_val_last;

  hal_s32_t *Hpos_cmd_pdo_val;
  hal_s32_t *Hvel_cmd_pdo_val;
  hal_s32_t *trq_cmd_pdo_val;
  hal_s32_t *mtrq_cmd_pdo_val;
  hal_u32_t control_cmd_val;
  hal_u32_t Platch_control_val;
  hal_u32_t latch_control_val_last;

  hal_bit_t *Hpos_latch;
  hal_bit_t pos_latch_last;

  double prbase_val;
  int prbase_mask;

  int drv_peak_current;

  hal_bit_t *Hpos_fb_index_enable;
  hal_bit_t *Hpos_extenc_index_enable;

} lcec_ax20_data_t;

/*           IN PDOS                      */
static ec_pdo_entry_info_t lcec_ax20_in[] = {
    {0x6064, 0x00, 32}, // Position actual value
    {0x35C9, 0x00, 32}, // Position actual value 2
    {0x6077, 0x00, 16}, // Torque actual value
    {0x60F4, 0x00, 32}, // Following error
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
  if (ecrt_slave_config_sdo8(slave->config, 0x6060, 0x00, 9) != 0)
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

  // initialize sync info
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
  
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "PDO init finish\n");
  /*
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos_fb-cnt_raw-hi", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_u32_new(name, HAL_OUT, &(hal_data->Hpos_fb_cnt_raw_hi), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-cnt-raw-lo", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_u32_new(name, HAL_OUT, &(hal_data->Hpos_fb_cnt_raw_lo), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-raw", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_fb_raw), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_fb), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-vel-fb", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hvel_fb), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-scale", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_param_float_new(name, HAL_RW, &(hal_data->Ppos_fb_scale), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting param %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-following-err", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_folloving_err), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-cnt-raw-hi", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_u32_new(name, HAL_OUT, &(hal_data->Hpos_extenc_cnt_raw_hi), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-cnt-raw-lo", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_u32_new(name, HAL_OUT, &(hal_data->Hpos_extenc_cnt_raw_lo), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-raw", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_extenc_raw), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_extenc), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-scale", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_param_float_new(name, HAL_RW, &(hal_data->Ppos_extenc_scale), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting param %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-vel-extenc", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hvel_extenc), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-cur-fb", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hcur_fb), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-cmd", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_IN, &(hal_data->Hpos_cmd), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-vel-cmd", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_IN, &(hal_data->Hvel_cmd), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-trq-cmd", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_IN, &(hal_data->Htrq_cmd), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-mtrq-cmd", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_IN, &(hal_data->Hmtrq_cmd), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-cur-cmd", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_IN, &(hal_data->Hcur_cmd), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  // Adding Digital inputs bits
  for (i = 0; i < 4; i++)
  {
    rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-Hdin%d", LCEC_MODULE_NAME, master->name, slave->name, i + 1);
    if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->Hdin[i]), comp_id)) != 0)
    {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
      return err;
    }
    rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-Hdin%d-not", LCEC_MODULE_NAME, master->name, slave->name, i + 1);
    if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->Hdin_not[i]), comp_id)) != 0)
    {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
      return err;
    }
  }

  // Adding Control word bits
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-enable", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->enable), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-inhibit", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->inhibit), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-ctrl-bit2", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->ctrl_bit2), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-setpoint-enable", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->setpoint_enable), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-clr-fault", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->clr_fault), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-homing-start", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->homing_start), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-save", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IN, &(hal_data->save), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  // Adding Status word bits
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-sw-on-not-ready", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->switch_on_not_ready), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-sw-on-disabled", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->switch_on_disabled), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-sw-on-ready", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->switch_on_ready), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-switched-on", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->switched_on), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-enabled", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->enabled), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-fault", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->fault), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-fault-react-active", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->fault_react_active), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-quick-stop-active", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->quick_stop_active), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-warning", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->warning), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-follwing-err-present", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->followinq_err), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-ref-point-set", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->ref_point_set), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-in-position", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->in_pos), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-on-limit-switch", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->on_lim_switch), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-ethercat-ok", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->ethercat_ok), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-homing-error", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->homing_error), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-motion-task-active", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_OUT, &(hal_data->motion_task_active), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-index-enable", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IO, &(hal_data->Hpos_fb_index_enable), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-index-enable", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IO, &(hal_data->Hpos_extenc_index_enable), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  // Debug, will be removed
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-dbg-status-pdo-val", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_param_u32_new(name, HAL_RO, &(hal_data->Pstatus_val), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-dbg-latchstatus-pdo-val", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_param_u32_new(name, HAL_RO, &(hal_data->Platch_status_val), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-dbg-Hpos_cmd-pdo-val", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_s32_new(name, HAL_OUT, &(hal_data->Hpos_cmd_pdo_val), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-dbg-Hvel_cmd-pdo-val", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_s32_new(name, HAL_OUT, &(hal_data->Hvel_cmd_pdo_val), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-dbg-latchcontrol_cmd-pdo-val", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_param_u32_new(name, HAL_RO, &(hal_data->Platch_control_val), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-latched-count", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_s32_new(name, HAL_OUT, &(hal_data->Hpos_latched_count), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-index-delta-count", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_s32_new(name, HAL_OUT, &(hal_data->Hpos_fb_index_delta_count), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-fb-index-delta", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_fb_index_delta), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-index-delta-count", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_s32_new(name, HAL_OUT, &(hal_data->Hpos_extenc_index_delta_count), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-extenc-index-delta", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_extenc_index_delta), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }

  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-latched", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_float_new(name, HAL_OUT, &(hal_data->Hpos_latched), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_snprintf(name, sizeof(name), "%s.%s.%s.srv-pos-latch", LCEC_MODULE_NAME, master->name, slave->name);
  if ((err = hal_pin_bit_new(name, HAL_IO, &(hal_data->Hpos_latch), comp_id)) != 0)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s failed\n", name);
    return err;
  }
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "HAL init finish\n");
  */
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init start finish\n");
  for (i = 0; i < 4; i++)
  {
    *(hal_data->Hdin[i]) = 0;
    *(hal_data->Hdin_not[i]) = 0;
  }
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 1 finish\n");
  *(hal_data->enable) = 0;
  *(hal_data->inhibit) = 0;
  *(hal_data->ctrl_bit2) = 0;
  *(hal_data->setpoint_enable) = 0;
  *(hal_data->clr_fault) = 0;
  *(hal_data->homing_start) = 0;
  *(hal_data->save) = 0;

  *(hal_data->switch_on_not_ready) = 0;
  *(hal_data->switch_on_disabled) = 0;
  *(hal_data->switch_on_ready) = 0;
  *(hal_data->switched_on) = 0;
  *(hal_data->enabled) = 0;
  *(hal_data->fault) = 0;
  *(hal_data->fault_react_active) = 0;
  *(hal_data->quick_stop_active) = 0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 2 finish\n");
  *(hal_data->warning) = 0;
  *(hal_data->followinq_err) = 0;
  *(hal_data->ref_point_set) = 0;
  *(hal_data->in_pos) = 0;
  *(hal_data->on_lim_switch) = 0;
  *(hal_data->ethercat_ok) = 0;
  *(hal_data->homing_error) = 0;
  *(hal_data->motion_task_active) = 0;

 rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 3 finish\n");

  *(hal_data->Hcur_cmd) = 0.0;
  *(hal_data->Hvel_cmd) = 0.0;
  *(hal_data->Htrq_cmd) = 0.0;
  *(hal_data->Hmtrq_cmd) = 0.0;
  *(hal_data->Hpos_cmd) = 0.0;

  hal_data->pos_fb_srv_cnt_last = 0;
  hal_data->pos_fb_index_cnt = 0;
  hal_data->pos_fb_cnt_last = 0;
  hal_data->pos_fb_cnt = 0;
  *(hal_data->Hpos_fb_cnt_raw_hi) = 0;
  *(hal_data->Hpos_fb_cnt_raw_lo) = 0;
  hal_data->pos_fb_home_raw = 0;
  *(hal_data->Hpos_fb_raw) = 0;
  *(hal_data->Hpos_fb) = 0;
  hal_data->Ppos_fb_scale = 0;
  hal_data->pos_fb_scale_old = 0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 4 finish\n");
 *(hal_data->Hpos_fb_index_delta_count) = 0;
 *(hal_data->Hpos_fb_index_delta) = 0;

 *(hal_data->Hpos_folloving_err) = 0;

 *(hal_data->Hvel_fb) = 0;

  hal_data->pos_extenc_srv_cnt_last = 0;
  hal_data->pos_extenc_index_cnt = 0;
  hal_data->pos_extenc_cnt_last = 0;
  hal_data->pos_extenc_cnt = 0;
  *(hal_data->Hpos_extenc_cnt_raw_hi) = 0;
  *(hal_data->Hpos_extenc_cnt_raw_lo) = 0;
  hal_data->pos_extenc_home_raw = 0;
  *(hal_data->Hpos_extenc_raw) = 0.0;
  *(hal_data->Hpos_extenc) = 0.0;
  hal_data->Ppos_extenc_scale = 1.0;
  hal_data->pos_extenc_scale_old = 2.0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 5 finish\n");
  *(hal_data->Hpos_extenc_index_delta_count) = 0;
  *(hal_data->Hpos_extenc_index_delta) = 0.0;

  *(hal_data->Hvel_extenc) = 0.0;


  *(hal_data->Hpos_latched_count) = 0;
  *(hal_data->Hpos_latched) = 0.0;

  *(hal_data->Hcur_fb) = 0.0;

    hal_data->Pstatus_val = 0;
   hal_data->Platch_status_val = 0;
    hal_data->latch_status_val_last = 0;
rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 6 finish\n");
  *(hal_data->Hpos_cmd_pdo_val) = 0.0;
  *(hal_data->Hvel_cmd_pdo_val) = 0.0;
  *(hal_data->trq_cmd_pdo_val) = 0.0;
  *(hal_data->mtrq_cmd_pdo_val) = 0.0;
  hal_data->control_cmd_val = 0;
  hal_data->Platch_control_val = 0;
  hal_data->latch_control_val_last = 0;


  *(hal_data->Hpos_latch) = 0.0;
  hal_data->pos_latch_last = 0.0;

  hal_data->prbase_val = 0;
  hal_data->prbase_mask = 0;

  hal_data->drv_peak_current = 6;
  
  *(hal_data->Hpos_fb_index_enable) = 0;
  *(hal_data->Hpos_extenc_index_enable) = 0;

  
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Init 7 finish\n");
  return 0;
}

void lcec_ax20_check_scales(lcec_ax20_data_t *hal_data)
{
/*
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
  */
}

void lcec_ax20_read(struct lcec_slave *slave, long period)
{
  lcec_master_t *master = slave->master;
  lcec_ax20_data_t *hal_data = (lcec_ax20_data_t *)slave->hal_data;
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
  /*
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

  *hal_data->Hpos_fb_cnt_raw_hi = (hal_data->pos_fb_cnt >> 32) & 0xffffffff;
  *hal_data->Hpos_fb_cnt_raw_lo = (hal_data->pos_fb_cnt) & 0xffffffff;
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

  for (i = 0; i < 4; i++)
  {
    *(hal_data->Hdin[3 - i]) = (status & (0x1000 << i)) == (0x1000 << i);
    *(hal_data->Hdin_not[3 - i]) = !*(hal_data->Hdin[3 - i]);
  }
  /*
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

  return;
}

void lcec_ax20_write(struct lcec_slave *slave, long period)
{
  lcec_master_t *master = slave->master;
  lcec_ax20_data_t *hal_data = (lcec_ax20_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control , lcontrol;
  int16_t tork;
  int16_t mtork;
  int32_t speed;
  int32_t position;
  /*
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
  hal_data->control_cmd_val = control;

  speed = (*hal_data->Hvel_cmd) / hal_data->Ppos_fb_scale * ax20_RPS_FACTOR;
  EC_WRITE_S32(&pd[hal_data->vel_cmd_pdo_os], speed);
  *hal_data->Hvel_cmd_pdo_val = speed;

  // position = (long long)((*hal_data->Hpos_cmd) / hal_data->Ppos_fb_scale * hal_data->prbase_val) + hal_data->pos_fb_index_cnt;
  position = (long long)((*hal_data->Hpos_cmd) / hal_data->Ppos_fb_scale * hal_data->prbase_val);
  EC_WRITE_S32(&pd[hal_data->pos_cmd_pdo_os], position);
  *hal_data->Hpos_cmd_pdo_val = position;

  EC_WRITE_U16(&pd[hal_data->latchcontrol_cmd_pdo_os], lcontrol);
  hal_data->Platch_control_val = lcontrol;

  tork =(*hal_data->trq_cmd_pdo_val);
  EC_WRITE_S16(&pd[hal_data->trq_cmd_pdo_os], tork);
  mtork =(*hal_data->mtrq_cmd_pdo_val);
  EC_WRITE_S16(&pd[hal_data->mtrq_cmd_pdo_os], mtork);

  */
  return;
}
