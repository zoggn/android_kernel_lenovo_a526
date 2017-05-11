#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/string.h>
#include "lcm_drv.h"

#define FRAME_WIDTH 		(480)
#define FRAME_HEIGHT 		(800)
#define REGFLAG_DELAY 		0XFE
#define REGFLAG_END_OF_TABLE  	0xFFF  // END OF REGISTERS MARKER
#define LCM_ID 			0x79
static unsigned int lcm_compare_id(void);
// ---------------------------------------------------------------------------
// Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
// ---------------------------------------------------------------------------
// Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)  lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)  lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xB9,0x03,{0xFF,0x83,0x79}},
    {0xB1,0x10,{0x44,0x18,0x18,0x31,0x31,0x50,0xD0,0xEE,0x54,0x80,
    		0x38,0x38,0xF8,0x33,0x32,0x22                     }},
    {0xB2,0x09,{0x82,0x3C,0x0a,0x04,0x00,0x50,0x11,0x42,0x1D      }},
    {0xB4,0x0A,{0x01,0x6E,0x01,0x6E,0x01,0x6E,0x22,0x80,0x23,0x80 }},
    {0xCC,0x01,{0x02						  }},
    {0xD2,0x01,{0x33						  }},
    {0xD3,0x1F,{0x00,0x07,0x00,0x3C,0x01,0x10,0x10,0x32,0x10,0x03,
	        0x00,0x03,0x03,0x30,0x03,0x30,0x00,0x08,0x00,0x08,
    		0x17,0x11,0x01,0x01,0x17,0x01,0x01,0x17,0x08,0x00,
    		0x14						  }},
    {0xD5,0x20,{0x18,0x18,0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,
        	0x18,0x18,0x20,0x21,0x18,0x18,0x1A,0x1A,0x18,0x18,
    		0x1B,0x1B,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
    		0x18,0x18					  }},
    {0xE0,0x2A,{0x00,0x00,0x07,0x0D,0x0F,0x3F,0x25,0x32,0x09,0x0F,
        	0x11,0x1A,0x11,0x15,0x18,0x16,0x16,0x09,0x13,0x14,
        	0x19,0x00,0x00,0x06,0x0E,0x0E,0x3F,0x25,0x32,0x0A,
    		0x10,0x11,0x1A,0x10,0x14,0x17,0x14,0x15,0x06,0x11,
    		0x11,0x16					  }},
    {0xB6,0x02,{0x85,0x85					  }},
    {0x11,1,{0x00}}, // Sleep-Out
    {REGFLAG_DELAY, 150, },
    {0x29,1,{0x00}},
    {REGFLAG_DELAY, 100, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
//---------------------------------

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update) {
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
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

// ---------------------------------------------------------------------------
// LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_get_params(LCM_PARAMS *params)
{
	//code from LK
    memset(params, 0, sizeof(LCM_PARAMS));
    params->type  			= LCM_TYPE_DSI;
    params->width 			= FRAME_WIDTH;
    params->height 			= FRAME_HEIGHT;
    params->dbi.te_mode 		= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity 	= LCM_POLARITY_RISING;
    params->dsi.mode  			= SYNC_PULSE_VDO_MODE;
    params->dsi.LANE_NUM 		= LCM_TWO_LANE;
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq  	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format  	= LCM_DSI_FORMAT_RGB888;
    params->dsi.packet_size		= 256;
    params->dsi.intermediat_buffer_num 	= 2;
    params->dsi.PS			= LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count  		= 480*3;
    params->dsi.vertical_sync_active 	= 3;
    params->dsi.vertical_backporch 	= 7;
    params->dsi.vertical_frontporch 	= 6;
    params->dsi.vertical_active_line 	= FRAME_HEIGHT;
    params->dsi.horizontal_sync_active 	= 36;
    params->dsi.horizontal_backporch 	= 36;
    params->dsi.horizontal_frontporch 	= 36;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.pll_div1		= 1;
    params->dsi.pll_div2		= 1;
    params->dsi.fbk_div 		= 30;
    params->dsi.ssc_disable		= 1;
    params->dsi.lcm_int_te_monitor	= FALSE;
    params->dsi.lcm_int_te_period	= 1;
    params->dsi.lcm_ext_te_monitor	= FALSE;
    params->dsi.noncont_clock		= TRUE;
    params->dsi.noncont_clock_period	= 2;
}
static void lcm_init(void)
{
    hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
    hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(150);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(3);
    SET_RESET_PIN(1);
    MDELAY(150);
}
static void lcm_resume(void)
{
    printk("hwj huaxian lcd resume!!!");
    lcm_init();
}
static void lcm_update( unsigned int x, unsigned int y,
			unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;
    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);
    unsigned int data_array[16];
    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    data_array[3]= 0x00053902;
    data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[5]= (y1_LSB);
    data_array[6]= 0x002c3909;
    dsi_set_cmdq(&data_array, 7, 0);
}
static unsigned int lcm_compare_id(void)
{
	return 1;
}
// ---------------------------------------------------------------------------
// Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8379c_dsi_vdo_hlt_lcm_drv =
{
    .name  = "hx8379c_dsi_vdo_hlt",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .compare_id = lcm_compare_id,
};
