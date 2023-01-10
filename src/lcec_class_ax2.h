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
#ifndef _LCEC_CLASS_AX2_H_
#define _LCEC_CLASS_AX2_H_

#include "lcec.h"
#include "lcec_class_enc.h"

#define AX2_STS_MASK       0xEFF  /* mask to remove manufacturer special bits */
#define AX2_STS_SW_ON_DIS  0b1000000  /* switched on disabled */
#define AX2_STS_RDY_SW_ON  0b0000001/* ready to switch on   */
#define AX2_STS_SW_ON_ENA  0b0100011 /* switched on enabled  */
#define AX2_STS_ERROR      0b0101000  /* error                */
#define AX2_STS_ENABLED    0b0100111  /* error                */

#define AX2_CMD_ENA_QSTOP  0x00   /* enable quick stop   */
#define AX2_CMD_DIS_QSTOP  0x06   /* disable quick stop  */
#define AX2_CMD_ENA_SWION  0x07   /* enable switched  on */
#define AX2_CMD_ENA_OP     0x0F   /* enable operation    */
#define AX2_CMD_CLR_ERROR  0x80   /* clear error         */



typedef struct {
  //drv cmd
  hal_bit_t *enable;
  hal_bit_t *halt;
  hal_bit_t *drv_on;

  //drv state
  hal_bit_t *sw_on_dis;
  hal_bit_t *rdy_to_on;
  hal_bit_t *sw_on;
  hal_bit_t *enabled;
  hal_bit_t *fault;
  
  

  hal_float_t *velo_cmd;
  hal_float_t *torq_cmd;
  hal_float_t *mtorq_cmd;

  int fb2_enabled;
  int diag_enabled;

  hal_u32_t *status;
  hal_u32_t *latch_status;
  hal_float_t *latch_pos;
  hal_float_t *torq_fb;
  hal_float_t *follerr_fb;
  
/*
  unsigned int status_pdo_os;
  unsigned int pos_fb_pdo_os;
  unsigned int pos_fb2_pdo_os;
  unsigned int torque_fb_pdo_os;
  unsigned int diag_pdo_os;
  unsigned int ctrl_pdo_os;
  unsigned int vel_cmd_pdo_os;
*/
//  pdos 1707 cnd 
  unsigned int pos_cmd_pdo_os;  //32
  unsigned int vel_cmd_pdo_os; //32
  unsigned int trq_cmd_pdo_os; //16
  unsigned int mtrq_cmd_pdo_os; //16
  unsigned int ctrl_pdo_os; //16
  unsigned int latch_ctrl_pdo_os; //16   16 B total
//PDOS  1B07  fb
  unsigned int pos_fbpdo_os;  //32
  unsigned int pos_fb2_pdos_os; //32
  unsigned int torque_fb_pdo_os; //16
  unsigned int follerr_pdo_os; //32
  unsigned int status_pdo_os; //16
  unsigned int latchstatus_pdo_os; //16
  unsigned int latchpos_pdo_os;  //32  22 B total


  hal_float_t scale;
  hal_float_t scale_fb2;
  hal_float_t vel_scale;
  hal_u32_t   pos_resolution;

  lcec_class_enc_data_t enc;
  lcec_class_enc_data_t enc_fb2;

  double scale_old;
  double scale_rcpt;
  double scale_fb2_old;
  double scale_fb2_rcpt;

  double vel_output_scale;

  int toggle;

} lcec_class_ax2_chan_t;

int lcec_class_ax2_pdos(struct lcec_slave *slave);
int lcec_class_ax2_init(struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs, lcec_class_ax2_chan_t *chan, int index, const char *pfx);
void lcec_class_ax2_read(struct lcec_slave *slave, lcec_class_ax2_chan_t *chan);
void lcec_class_ax2_write(struct lcec_slave *slave, lcec_class_ax2_chan_t *chan);

#endif

