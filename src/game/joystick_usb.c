/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB game controllers, on the joystick side. The USB devices
 *          themselves are in usb_hid.c, where the axes and buttons here
 *          must stay in step with the model table.
 *
 *          Copyright 2021-2025 Jasmine Iwanek.
 */
#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

/* USB pads never answer on a game port. */
void *
joystick_usb_init(void)
{
    return NULL;
}

void
joystick_usb_close(UNUSED(void *priv))
{
}

uint8_t
joystick_usb_read(UNUSED(void *priv))
{
    return 0xff;
}

void
joystick_usb_write(UNUSED(void *priv))
{
}

int
joystick_usb_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    return AXIS_NOT_PRESENT;
}

void
joystick_usb_a0_over(UNUSED(void *priv))
{
}

const joystick_t joystick_usb_gamepad = {
    .name          = "USB gamepad (2-axis, 10-button)",
    .internal_name = "usb_gamepad",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8", "Button 9", "Button 10" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_device, &gamepad_usb_2_device }
};

const joystick_t joystick_usb_gamepad_6axis = {
    .name          = "USB gamepad (6-axis, 11-button)",
    .internal_name = "usb_gamepad_6axis",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 6,
    .button_count  = 11,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "Left stick X", "Left stick Y", "Right stick X", "Right stick Y", "Left trigger", "Right trigger" },
    .button_names  = { "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "Left stick", "Right stick", "Guide" },
    .pov_names     = { "D-pad" },
    .usb_device    = { &gamepad_usb_6axis_device, &gamepad_usb_6axis_2_device }
};

const joystick_t joystick_sidewinder_pad_usb = {
    .name          = "SideWinder Game Pad USB version 1.0",
    .internal_name = "sidewinder_pad_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "A", "B", "C", "X", "Y", "Z", "L", "R", "Start", "Select" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_sidewinder_pad_device, &gamepad_sidewinder_pad_2_device }
};

const joystick_t joystick_sidewinder_precision_pro_usb = {
    .name          = "Microsoft SideWinder Precision Pro (USB)",
    .internal_name = "sidewinder_precision_pro_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 9,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Rudder", "Throttle" },
    .button_names  = { "Trigger", "Thumb", "Top", "Top 2", "Base", "Base 2", "Base 3", "Base 4", "Shift" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_sidewinder_precision_pro_device, &gamepad_sidewinder_precision_pro_2_device }
};

const joystick_t joystick_sidewinder_freestyle_pro_usb = {
    .name          = "Microsoft SideWinder Freestyle Pro (USB)",
    .internal_name = "sidewinder_freestyle_pro_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 3,
    .button_count  = 10,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Throttle" },
    .button_names  = { "A", "B", "C", "X", "Y", "Z", "L", "R", "Start", "Shift" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_sidewinder_freestyle_pro_device, &gamepad_sidewinder_freestyle_pro_2_device }
};

const joystick_t joystick_tm_rage3d_usb = {
    .name          = "ThrustMaster Rage 3D USB Game Pad",
    .internal_name = "tm_rage3d_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_tm_rage3d_device, &gamepad_tm_rage3d_2_device }
};

const joystick_t joystick_tm_top_gun_usb = {
    .name          = "ThrustMaster Top Gun USB Joystick",
    .internal_name = "tm_top_gun_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 4,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_tm_top_gun_device, &gamepad_tm_top_gun_2_device }
};

const joystick_t joystick_tm_top_gun_rev2_usb = {
    .name          = "ThrustMaster Top Gun USB Joystick (rev2)",
    .internal_name = "tm_top_gun_usb_rev2",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 4,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_tm_top_gun_rev2_device, &gamepad_tm_top_gun_rev2_2_device }
};

const joystick_t joystick_nec_pk_gp201_usb = {
    .name          = "NEC PK-GP201 PlayStick",
    .internal_name = "nec_pk_gp201",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 4,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_nec_pk_gp201_device, &gamepad_nec_pk_gp201_2_device }
};

const joystick_t joystick_nec_pk_gp301_usb = {
    .name          = "NEC PK-GP301 Driving Wheel",
    .internal_name = "nec_pk_gp301",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 3,
    .button_count  = 4,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "Wheel", "Accelerator", "Brake" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_nec_pk_gp301_device, &gamepad_nec_pk_gp301_2_device }
};

const joystick_t joystick_logitech_wingman_extreme_usb = {
    .name          = "Logitech USB WingMan Extreme Joystick",
    .internal_name = "logitech_wingman_extreme_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 6,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_logitech_wingman_extreme_device, &gamepad_logitech_wingman_extreme_2_device }
};

const joystick_t joystick_logitech_wingman_extreme_throttle_usb = {
    .name          = "Logitech USB WingMan Extreme Joystick with Throttle",
    .internal_name = "logitech_wingman_extreme_throttle_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 7,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Rudder", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6", "Button 7" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_logitech_wingman_extreme_throttle_device, &gamepad_logitech_wingman_extreme_throttle_2_device }
};

const joystick_t joystick_genius_joystick_usb = {
    .name          = "Genius USB Joystick",
    .internal_name = "genius_joystick_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 3,
    .button_count  = 6,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_genius_joystick_device, &gamepad_genius_joystick_2_device }
};

const joystick_t joystick_genius_gamepad_usb = {
    .name          = "Genius USB Game Pad",
    .internal_name = "genius_gamepad_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_genius_gamepad_device, &gamepad_genius_gamepad_2_device }
};

const joystick_t joystick_guillemot_jet_leader_usb = {
    .name          = "Guillemot Jet Leader USB Joystick",
    .internal_name = "guillemot_jet_leader_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 6,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Rudder", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_guillemot_jet_leader_device, &gamepad_guillemot_jet_leader_2_device }
};

const joystick_t joystick_logitech_wingman_formula_yellow_usb = {
    .name          = "Logitech WingMan Formula (Yellow)(USB)",
    .internal_name = "logitech_wingman_formula_yellow_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 6,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "Wheel", "Pedals" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_logitech_wingman_formula_yellow_device, &gamepad_logitech_wingman_formula_yellow_2_device }
};

const joystick_t joystick_logitech_wingman_extreme_digital_3d_usb = {
    .name          = "Logitech WingMan Extreme Digital 3D (USB)",
    .internal_name = "logitech_wingman_extreme_digital_3d_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 7,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Rudder", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6", "Button 7" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_logitech_wingman_extreme_digital_3d_device, &gamepad_logitech_wingman_extreme_digital_3d_2_device }
};

const joystick_t joystick_logitech_wingman_gamepad_extreme_usb = {
    .name          = "Logitech WingMan Gamepad Extreme (USB)",
    .internal_name = "logitech_wingman_gamepad_extreme_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 11,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "A", "B", "C", "X", "Y", "Z", "L", "R", "Start", "Mode", "Select" },
    .pov_names     = { "Hat" },
    .usb_device    = { &gamepad_logitech_wingman_gamepad_extreme_device, &gamepad_logitech_wingman_gamepad_extreme_2_device }
};

const joystick_t joystick_logitech_wingman_gamepad_usb = {
    .name          = "Logitech WingMan Gamepad (USB)",
    .internal_name = "logitech_wingman_gamepad_usb",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 11,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "A", "B", "C", "X", "Y", "Z", "L", "R", "Start", "Mode", "Select" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_logitech_wingman_gamepad_device, &gamepad_logitech_wingman_gamepad_2_device }
};

const joystick_t joystick_usb_gamepad_2axis_8button = {
    .name          = "USB gamepad (2-axis, 8-button)",
    .internal_name = "usb_gamepad_2axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_2axis_8button_device, &gamepad_usb_2axis_8button_2_device }
};

const joystick_t joystick_usb_gamepad_3axis_8button = {
    .name          = "USB gamepad (3-axis, 8-button)",
    .internal_name = "usb_gamepad_3axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 3,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y", "Throttle" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_3axis_8button_device, &gamepad_usb_3axis_8button_2_device }
};

const joystick_t joystick_usb_gamepad_4axis_8button = {
    .name          = "USB gamepad (4-axis, 8-button)",
    .internal_name = "usb_gamepad_4axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "Left stick X", "Left stick Y", "Right stick X", "Right stick Y" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_4axis_8button_device, &gamepad_usb_4axis_8button_2_device }
};

const joystick_t joystick_usb_joystick_2axis_8button = {
    .name          = "USB joystick (2-axis, 8-button)",
    .internal_name = "usb_joystick_2axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 2,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_joystick_2axis_8button_device, &gamepad_usb_joystick_2axis_8button_2_device }
};

const joystick_t joystick_usb_joystick_3axis_8button = {
    .name          = "USB joystick (3-axis, 8-button)",
    .internal_name = "usb_joystick_3axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 3,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_joystick_3axis_8button_device, &gamepad_usb_joystick_3axis_8button_2_device }
};

const joystick_t joystick_usb_joystick_4axis_8button = {
    .name          = "USB joystick (4-axis, 8-button)",
    .internal_name = "usb_joystick_4axis_8button",
    .init          = joystick_usb_init,
    .close         = joystick_usb_close,
    .read          = joystick_usb_read,
    .write         = joystick_usb_write,
    .read_axis     = joystick_usb_read_axis,
    .a0_over       = joystick_usb_a0_over,
    .axis_count    = 4,
    .button_count  = 8,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Rudder", "Throttle" },
    .button_names  = { "Trigger", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6",
                       "Button 7", "Button 8" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_usb_joystick_4axis_8button_device, &gamepad_usb_joystick_4axis_8button_2_device }
};
