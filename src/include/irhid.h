/*
 * irhid  --- USB HID remote control keyboard
 *            uses TIMER2
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __IRHID_H__
#define __IRHID_H__

void irhid_init(void);
void irhid_tasks(void);

#endif
