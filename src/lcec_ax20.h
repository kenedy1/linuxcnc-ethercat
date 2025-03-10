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

#ifndef LCEC_AX20_H
#define LCEC_AX20_H

#include "lcec.h"

#define LCEC_AX20_VID LCEC_BECKHOFF_VID
#define LCEC_AX20_PID 0x07d06014

#define LCEC_AX20_PDOS 13

int lcec_ax20_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs);

#endif // LCEC_AX20_H
