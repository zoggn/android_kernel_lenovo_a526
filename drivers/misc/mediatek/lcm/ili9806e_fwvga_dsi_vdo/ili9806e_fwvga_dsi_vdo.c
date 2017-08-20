// ---------------------------------------------------------------------------
//  Thanks LazyCODEr for reverse this driver :)
// ---------------------------------------------------------------------------
#if defined(BUILD_LK)
#include <string.h>
#else
#include <linux/string.h>
#endif


#if defined(BUILD_LK)
#include "cust_gpio_usage.h"
#include <platform/mt_gpio.h>
#else
#include "cust_gpio_usage.h"
#include <mach/mt_gpio.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define GPIO_LCM_ID	GPIO19


#define LCM_ID_ILI9806E 									(0x980604)


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
static bool first_run = 1;
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define write_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_init1[] = {

{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x01 }},
{ 0x08, 0x01, {0x10 }},
{ 0x21, 0x01, {0x01 }},
{ 0x30, 0x01, {0x01 }},
{ 0x31, 0x01, {0x02 }},
{ 0x40, 0x01, {0x12 }},
{ 0x41, 0x01, {0x33 }},
{ 0x42, 0x01, {0x01 }},
{ 0x43, 0x01, {0x09 }},
{ 0x44, 0x01, {0x06 }},
{ 0x50, 0x01, {0x78 }},
{ 0x51, 0x01, {0x78 }},
{ 0x52, 0x01, {0x00 }},
{ 0x53, 0x01, {0x1f }},
{ 0x57, 0x01, {0x50 }},
{ 0x60, 0x01, {0x07 }},
{ 0x61, 0x01, {0x00 }},
{ 0x62, 0x01, {0x07 }},
{ 0x63, 0x01, {0x00 }},
{ 0xa0, 0x01, {0x00 }},
{ 0xa1, 0x01, {0x16 }},
{ 0xa2, 0x01, {0x1d }},
{ 0xa3, 0x01, {0x14 }},
{ 0xa4, 0x01, {0x0e }},
{ 0xa5, 0x01, {0x1e }},
{ 0xa6, 0x01, {0x0e }},
{ 0xa7, 0x01, {0x09 }},
{ 0xa8, 0x01, {0x05 }},
{ 0xa9, 0x01, {0x0f }},
{ 0xaa, 0x01, {0x0b }},
{ 0xab, 0x01, {0x02 }},
{ 0xac, 0x01, {0x11 }},
{ 0xad, 0x01, {0x2d }},
{ 0xae, 0x01, {0x28 }},
{ 0xaf, 0x01, {0x00 }},
{ 0xc0, 0x01, {0x00 }},
{ 0xc1, 0x01, {0x0d }},
{ 0xc2, 0x01, {0x1c }},
{ 0xc3, 0x01, {0x06 }},
{ 0xc4, 0x01, {0x05 }},
{ 0xc5, 0x01, {0x0a }},
{ 0xc6, 0x01, {0x09 }},
{ 0xc7, 0x01, {0x0a }},
{ 0xc8, 0x01, {0x02 }},
{ 0xc9, 0x01, {0x04 }},
{ 0xca, 0x01, {0x02 }},
{ 0xcb, 0x01, {0x08 }},
{ 0xcc, 0x01, {0x01 }},
{ 0xcd, 0x01, {0x28 }},
{ 0xce, 0x01, {0x24 }},
{ 0xcf, 0x01, {0x00 }},
{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x06 }},
{ 0x00, 0x01, {0x21 }},
{ 0x01, 0x01, {0x06 }},
{ 0x02, 0x01, {0x00 }},
{ 0x03, 0x01, {0x00 }},
{ 0x04, 0x01, {0x16 }},
{ 0x05, 0x01, {0x16 }},
{ 0x06, 0x01, {0x80 }},
{ 0x07, 0x01, {0x02 }},
{ 0x08, 0x01, {0x07 }},
{ 0x09, 0x01, {0x00 }},
{ 0x0a, 0x01, {0x00 }},
{ 0x0b, 0x01, {0x00 }},
{ 0x0c, 0x01, {0x16 }},
{ 0x0d, 0x01, {0x16 }},
{ 0x0e, 0x01, {0x00 }},
{ 0x0f, 0x01, {0x00 }},
{ 0x10, 0x01, {0x77 }},
{ 0x11, 0x01, {0xf0 }},
{ 0x12, 0x01, {0x00 }},
{ 0x13, 0x01, {0x00 }},
{ 0x14, 0x01, {0x00 }},
{ 0x15, 0x01, {0xc0 }},
{ 0x16, 0x01, {0x08 }},
{ 0x17, 0x01, {0x00 }},
{ 0x18, 0x01, {0x00 }},
{ 0x19, 0x01, {0x00 }},
{ 0x1a, 0x01, {0x00 }},
{ 0x1b, 0x01, {0x00 }},
{ 0x1c, 0x01, {0x00 }},
{ 0x1d, 0x01, {0x00 }},
{ 0x20, 0x01, {0x01 }},
{ 0x21, 0x01, {0x23 }},
{ 0x22, 0x01, {0x45 }},
{ 0x23, 0x01, {0x67 }},
{ 0x24, 0x01, {0x01 }},
{ 0x25, 0x01, {0x23 }},
{ 0x26, 0x01, {0x45 }},
{ 0x27, 0x01, {0x67 }},
{ 0x30, 0x01, {0x11 }},
{ 0x31, 0x01, {0x22 }},
{ 0x32, 0x01, {0x11 }},
{ 0x33, 0x01, {0x00 }},
{ 0x34, 0x01, {0x66 }},
{ 0x35, 0x01, {0x88 }},
{ 0x36, 0x01, {0x22 }},
{ 0x37, 0x01, {0x22 }},
{ 0x38, 0x01, {0xaa }},
{ 0x39, 0x01, {0xcc }},
{ 0x3a, 0x01, {0xbb }},
{ 0x3b, 0x01, {0xdd }},
{ 0x3c, 0x01, {0x22 }},
{ 0x3d, 0x01, {0x22 }},
{ 0x3e, 0x01, {0x22 }},
{ 0x3f, 0x01, {0x22 }},
{ 0x40, 0x01, {0x22 }},
{ 0x52, 0x01, {0x10 }},
{ 0x53, 0x01, {0x10 }},
{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x07 }},
{ 0x17, 0x01, {0x22 }},
{ 0x02, 0x01, {0x77 }},
{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x00 }},
{ REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_init2[] = {
{ 0x11, 0x01, {0x00 }},
{ REGFLAG_DELAY, 120, {}},
{ 0x29, 0x01, {0x00 }},
{ REGFLAG_DELAY, 10, {}},
{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x08 }},
{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x00 }},
{ REGFLAG_DELAY, 1, {}},
{ 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x00 }},
{ 0x28, 0x01, {0x00 }},
{ REGFLAG_DELAY, 10, {}},
{ 0x10, 0x01, {0x00 }},
{ REGFLAG_DELAY, 120, {}},
{ REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_compare_id_in_setting[] = {

    { 0xff, 0x05, {0xff, 0x98, 0x06, 0x04, 0x00 }},
    { REGFLAG_DELAY, 10, {}},
    { REGFLAG_END_OF_TABLE, 0x00, {}},
    
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
memset(params, 0, sizeof(LCM_PARAMS));

params->type = 2;
params->dsi.LANE_NUM = 2;
params->dsi.data_format.format = 2;
params->dsi.PS = 2;
params->dsi.vertical_sync_active = 2;
params->width = FRAME_WIDTH;
params->dsi.horizontal_sync_active = 10;
params->height = FRAME_HEIGHT;
params->dsi.horizontal_backporch = 60;
params->dsi.mode = 1;
params->dsi.horizontal_frontporch = 200;
params->dsi.data_format.color_order = 0;
params->dsi.data_format.trans_seq = 0;
params->dsi.data_format.padding = 0;
params->dsi.packet_size = 256;
params->dsi.intermediat_buffer_num = 0;
params->dsi.vertical_backporch = 20;
params->dsi.vertical_frontporch = 20;
params->dsi.vertical_active_line = FRAME_HEIGHT;
params->dsi.horizontal_active_pixel = FRAME_WIDTH;
params->dsi.PLL_CLOCK = 250;
params->dsi.ssc_range = 0;
params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	push_table(lcm_init1, sizeof(lcm_init1) / sizeof(struct LCM_setting_table), 1);
	push_table(lcm_init2, sizeof(lcm_init2) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}



static void lcm_resume(void)
{
	lcm_init();  
}

static void lcm_esd_recover(void)
{
	lcm_init();
}

static unsigned int lcm_esd_check(void)
{

  int result;
  unsigned char outdata[2];
  int data[4];
  result = 0;
  if ( first_run )
  {
    result = 1;
    first_run = false;
  }
  else
  {
    UDELAY(600);
    data[0] = 0x63902;
    data[1] = 0x698FFFF;
    data[2] = 4;
    dsi_set_cmdq(data, 3, 1);
    data[0] = 0x13700;
    dsi_set_cmdq(data, 1, 1);
    read_reg_v2(10, &outdata, 2);
    data[0] = 0x63902;
    data[1] = 0x698FFFF;
    data[2] = 0x804;
    dsi_set_cmdq(data, 3, 1);
    if ( outdata != 0x9C )
      result = 1;
  }
  return result;
}

// ---------------------------------------------------------------------------
//  Get LCM ID Information
// ---------------------------------------------------------------------------
static unsigned int lcm_compare_id()
{
  int v0;

  mt_set_gpio_mode(GPIO_LCM_ID, 0);
  mt_set_gpio_dir(GPIO_LCM_ID, 0);
  mt_set_gpio_pull_enable(GPIO_LCM_ID, 0);
  v0 = mt_get_gpio_in(GPIO_LCM_ID);
  return v0 == 1;

} 

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_fwvga_dsi_vdo_lcm_drv =
{
    .name			= "ili9806e_fwvga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
        .esd_check     =  lcm_esd_check,
        .esd_recover   = lcm_esd_recover,
};

