/* touchscreen/ektf2k_mtk.c - ELAN EKTF2K touchscreen driver
 * for MTK65xx serial platform.
 *
 * Copyright (C) 2012 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * 2012.5.14 Release for MTK platform, driver version: mtk0003
 */

//modified by luosen ��������ͻ�������Զ�����TP�̼����ظ�ΪTP_FIRMWARE_UPDATE
//#define TP_FIRMWARE_UPDATE
//#define ISP_IN_DRIVER //comment by assusdan 14.02.2016
//#define ISP_IN_DRIVER_NOJUDGE_FOR_HRC  // ckt LiuHuojun 2013.1.14 18:04 ���жϣ�ǿ��д�뻪�𴨹̼�
//#define ISP_IN_DRIVER_NOJUDGE_FOR_DM    // ckt LiuHuojun 2013.1.14 18:05 ���жϣ�ǿ��д�������̼�
//#define ISP_IN_DRIVER_NOJUDGE_FOR_LC    // ckt guoyi 2013.6.7 11:55 ���жϣ�ǿ��д�������̼�

#include <linux/module.h>
#include <linux/input.h>
#include "tpd.h"
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>

#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include "tpd_custom_ektf2k.h"
//#include <mach/mt6573_boot.h> /*Set your boot */
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
//#include "cust_gpio_usage.h"

#define PACKET_SIZE		18
//added by luosen ����ʵ��CTP�����ӽ�����������
//#define TP_REPLACE_ALSPS
//end
//#define TP_REPLACE_ALSPS_TEST
/*  */
#define IAP_PORTION 1
#if IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x2a, 0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old
uint8_t *fw_data_p;


/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
//#include "hua_14fa_4110.i"
};

#endif
static int update_flage = 0;
static int is_down = 0;
extern struct tpd_device *tpd;

struct i2c_client *ektf_i2c_client = NULL;
struct task_struct *ektf_thread = NULL;
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;  
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
int NEW_FW_VERSION=0x00;
int BC_VERSION = 0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
uint8_t RECOVERY=0x00;
char TPFW[32]; // ckt LiuHuojun 2013.3.15 14:05 ���ⲿ��ȡ�̼��汾��Ϣ
int TPFW_ID=0; //ckt LiuHuojun 20130826 �жϵ�ǰID��,���Զ�����ʧ��ʱ���Բο�ǰһ�ε�ID���Զ�ѡ�񳧼ҹ̼�����

// ckt LiuHuojun 2013.3.20 10:17 ָ����Ҫ�����Ĺ̼�ID��/�汾������
//���𴨵�����ID/VERΪ0x14f0/0x0b03
//����������ID/VERΪ 0x14f1/0x0b13
/*
static int FW_Infor[3][3]=
{//ID / VER / WHICH
#if 1 
   //HRC
  	{0x14f0,  0xc103,  0},

   //DM
  	{0x14f1,  0xc103,  1},

#endif
  	{0,  0,  0},
};
*/
static int FW_AllInfor[3][3]=
{//ID / VER / WHICH �������������İ汾�Ŷ�Ӧ
   //HRC
  	{0x14fa,  0x4110,  0},

   //DM
  	{0x14f7,  0x4104,  1},

  	{0,  0,  0},
};
extern void show_ektf2k_name(void);


#if defined(ISP_IN_DRIVER)
static uint8_t file_fw_data_hrc[] = {
    #include "hua_14fa_4110.i"
};
static uint8_t file_fw_data_dm[] = {
    #include "dian_14f7_4104.i"
};


#if 0//defined(ISP_IN_DRIVER_NOJUDGE_FOR_HRC)
static uint8_t file_fw_data_fixed_hrc[] = {
	#include "hua_14fa_4104.i"
};
//#elif defined(ISP_IN_DRIVER_NOJUDGE_FOR_DM)
static uint8_t file_fw_data_fixed_dm[] = {
	#include "dian_14f7_4104.i"
};
#endif
#endif


#if defined(__CTP_ESD_ORIGIN__)||defined(ISP_IN_DRIVER)


static struct workqueue_struct *elan_wq = NULL;
static struct work_struct work;
#endif


static DECLARE_WAIT_QUEUE_HEAD(waiter);
#define __CTP_ESD__

#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:17:58
static wait_queue_head_t esd_wait_queue;
static int esdflag = 0;
DEFINE_SEMAPHORE(sem_suspend);
static struct task_struct *ctp_esd_recovery_task = NULL;
static bool esd_kthread_pause = false;
bool is_suspended = false;
#endif /* __CTP_ESD__ */

static void tpd_eint_interrupt_handler(void);
static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y);

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

#if defined(TP_REPLACE_ALSPS)
int ckt_tp_replace_ps_mod_on(void);
int ckt_tp_replace_ps_mod_off(void);
int  ckt_tp_replace_ps_enable(int enable);
u16  ckt_get_tp_replace_ps_value(void);
int ckt_tp_replace_ps_state= 0;
int ckt_tp_replace_ps_close = 0;
#endif
int IAPReset();
int Update_FW_One_update(struct i2c_client *client);
int Update_FW_One(struct i2c_client *client, int recovery);
static struct elan_ktf2k_ts_data *private_ts;
static int __hello_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int ektf_tpd_resume(struct i2c_client *client);

#define CMD_W_PKT			0x54  


static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static int finger_num = 0;
static BOOTMODE bootMode = NORMAL_BOOT;  //  add for TP button,by tyd-lg

#if defined(__CTP_ESD_ORIGIN__)
static struct timer_list elan_timer;
#endif

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#define TPD_OK 0
// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)


#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)

//#define TP_DEBUG_MSG

#if defined(TP_DEBUG_MSG)
#define pr_tp(format, args...) printk("<0>" format, ##args)
#define pr_k(format, args...) printk("<0>" format, ##args)
#define pr_ch(format, args...)                      \
    printk("<0>" "%s <%d>,%s(),cheehwa_print:\n\t"  \
           format,__FILE__,__LINE__,__func__, ##args)

#else
#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, args...)  do {} while (0)
#undef pr_k(format, args...)
#define pr_k(format, args...)  do {} while (0)
#endif

struct touch_info
{
    unsigned short y[3];
    unsigned short x[3];
    unsigned short p[3];
	u8 key_val;
    unsigned short count;
};

typedef struct
{
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short pos_x2;
    unsigned short pos_y2;
    unsigned short temp2;
    unsigned short temp;
    short dst_x;
    short dst_y;
    unsigned char checksum;
} SHORT_TOUCH_STATE;


static const struct i2c_device_id tpd_id[] = {{"ekft2k", 0}, {}}; //{{TPD_DEVICE, 0}, {}};
//unsigned short force[] = {0, 0x2a, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };


static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = "ekft2k", //.name = TPD_DEVICE,
        .owner = THIS_MODULE,
    },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tpd_id,
    .detect = tpd_detect,
    //.address_data = &addr_data,
};

#ifdef ISP_IN_DRIVER
#define IAPRESTART 5
struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct early_suspend early_suspend;
	int intr_gpio;
    // Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
    // For Firmare Update 
	struct miscdevice firmware;
	struct hrtimer timer;
};

//static struct elan_ktf2k_ts_data *private_ts;

enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,	
};
extern void mtk_wdt_restart(enum wk_wdt_type type);

//static uint8_t I2C_DATA = 0x2a;        /*I2C devices address*/


#define PAGERETRY  10

static int request_firmware(struct i2c_client *client);

enum {
	PageSize		= 132,
	PageNum			= 249,
	ACK_Fail		= 0x00,
	ACK_OK	    = 0xAA,
	ACK_REWRITE = 0x55,
};

enum {
	E_FD = -1,
};
// For Firmware Update 
int  IAPReset()
{
			int res;

	   		mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
 			mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
    			mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
			mdelay(10);
		//	#if !defined(EVB)
    			mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//		#endif
	   		mdelay(10);
    			mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
			return 1;
}
int elan_iap_open(struct inode *inode, struct file *filp){ 

	pr_k("[ELAN]into elan_iap_open\n");
		if (private_ts == NULL)  pr_k("private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;

    pr_k("[ELAN]into elan_iap_write\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
#ifdef MTK6589_DMA    
    ret = elan_i2c_dma_send_data(private_ts->client, tmp, count);
#else    
    ret = i2c_master_send(private_ts->client, tmp, count);
#endif    
    kfree(tmp);
    return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;

    pr_k("[ELAN]into elan_iap_read\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;
#ifdef MTK6589_DMA
    ret = elan_i2c_dma_recv_data(private_ts->client, tmp, count);
#else    
    ret = i2c_master_recv(private_ts->client, tmp, count);
#endif  
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;
	
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	pr_k("[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;
	pr_k("[ELAN]into elan_iap_ioctl\n");
	pr_k("cmd value %x\n",cmd);
	
	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			private_ts->client->addr &= I2C_MASK_FLAG; 
			private_ts->client->addr |= I2C_ENEXT_FLAG;
			//file_fops_addr = 0x15;
			break;   
		case IOCTL_MAJOR_FW_VER:            
			break;        
		case IOCTL_MINOR_FW_VER:            
			break;        
		case IOCTL_RESET:

	   		mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
 			mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
    			mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
			mdelay(20);
		//	#if !defined(EVB)
    				mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		//	#endif
		        mdelay(20);
    			mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
 			mdelay(20);

			break;
		case IOCTL_IAP_MODE_LOCK:
			if(work_lock==0)
			{
				pr_k("[elan]%s %x=IOCTL_IAP_MODE_LOCK\n", __func__,IOCTL_IAP_MODE_LOCK);
				work_lock=1;
				update_flage = 1;
				disable_irq(CUST_EINT_TOUCH_PANEL_NUM);
				mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
				//cancel_work_sync(&private_ts->work);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(work_lock==1)
			{			
				work_lock=0;
				update_flage = 0;
				enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
				mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_ROUGH_CALIBRATE:
			return elan_ktf2k_ts_rough_calibrate(private_ts->client);
		case IOCTL_I2C_INT:
			put_user(mt_get_gpio_in(GPIO_CTP_EINT_PIN),ip);
			pr_k("[elan]GPIO_CTP_EINT_PIN = %d\n", mt_get_gpio_in(GPIO_CTP_EINT_PIN));

			break;	
		case IOCTL_RESUME:
			ektf_tpd_resume(private_ts->client);
			break;	
		case IOCTL_CIRCUIT_CHECK:
			return circuit_ver;
			break;
		case IOCTL_POWER_LOCK:
			power_lock=1;
			break;
		case IOCTL_POWER_UNLOCK:
			power_lock=0;
			break;
#if IAP_PORTION		
		case IOCTL_GET_UPDATE_PROGREE:
			update_progree=(int __user)arg;
			break; 

		case IOCTL_FW_UPDATE:
			//RECOVERY = IAPReset(private_ts->client);
			RECOVERY=0;
			Update_FW_One_update(private_ts->client);
#endif
		case IOCTL_BC_VER:
			__fw_packet_handler(private_ts->client);
			return BC_VERSION;
			break;
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
        .open =         elan_iap_open,    
        .write =        elan_iap_write,    
        .read = 	elan_iap_read,    
        .release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
 };
static int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd, int cmd_len)
{
	int len = 0;
    
	len = i2c_master_send(private_ts->client, isp_cmd, cmd_len);
	if (len != cmd_len) {
		pr_tp(TPD_DEVICE "[elan] ERROR: EnterISPMode fail! len=%d", len);
		return -1;
	} else
		pr_tp(TPD_DEVICE "[elan] IAPMode write data successfully!");
    
	return 0;
}

static int WritePage(uint8_t *szPage, int byte)
{
	int len = 0;
    
	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		pr_tp(TPD_DEVICE "[elan] ERROR: write page error, write error. err=%d", len);
	}
    
	return 0;
}

static int GetAckData(struct i2c_client *client)
{
	int len = 0;
	char buff[2] = {0};
    
	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		pr_tp(TPD_DEVICE "[elan] ERROR: read data error, write 50 times error. len=%d\r", len);
	}
	
	pr_tp(TPD_DEVICE "[elan]%s buf[0] = 0x%x, buf[1] = 0x%x\n", __func__, buff[0], buff[1]);
    
	if (buff[0] == 0xaa )
		pr_tp(TPD_DEVICE "[elan] 0x%2x,0x%2x", buff[0], buff[1]);
	else 
		pr_tp(TPD_DEVICE "[elan] GetAckData ERROR 0x%2x,0x%2x", buff[0], buff[1]);
    
	if (buff[0] == 0xaa ) {
		return ACK_OK;
	} else if (buff[0] == 0x55) {
		return ACK_REWRITE;
	} else {
		return ACK_Fail;
	}
    
	return 0;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(void)
{
	char buff[4] = {0},len = 0;
	//WaitIAPVerify(1000000);
	//len = read(fd, buff, sizeof(buff));
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) 
	{
		pr_tp(TPD_DEVICE "[ELAN] CheckIapMode ERROR: read data error,len=%d\r\n", len);
		return -1;
	}
	else
	{
		
		if (buff[0] == 0x55 && buff[1] == 0xaa && buff[2] == 0x33 && buff[3] == 0xcc)
		{
			//pr_tp(TPD_DEVICE "[ELAN] CheckIapMode is 55 aa 33 cc\n");
			return 0;
		}
        else if (buff[0] == 0x55 && buff[1] == 0x55 && buff[2] == 0x80 && buff[3] == 0x80) {
            len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
            pr_tp(TPD_DEVICE "[ELAN] Recovery Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
            return 0;
        }
		else// if ( j == 9 )
		{
			pr_tp(TPD_DEVICE "[ELAN] Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
			pr_tp(TPD_DEVICE "[ELAN] ERROR:  CheckIapMode error\n");
			return -1;
		}
	}
	pr_tp(TPD_DEVICE "\n");	
}

static void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];
    
	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}
    
	page_tatol = page + 249 * (ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));
    
	if ((page) == (249))
		percent = 100;
    
	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;
    
	pr_tp(TPD_DEVICE "\rprogress %s| %d%%", str, percent);
    
	if (page == (249))
		pr_tp(TPD_DEVICE "\n");
}


int Update_FW_One_update(struct i2c_client *client)
{
	int res = 0,ic_num = 0;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;

	int restartCnt = 0, checkCnt = 0; // For IAP_RESTART
	//uint8_t recovery_buffer[4] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;

	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34

	uint8_t recovery_buffer[4] = {0};

IAP_RESTART:	

	data=I2C_DATA[0];//Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

//	if(recovery != 0x80)
//	{
		pr_k("[ELAN] Firmware upgrade normal mode !\n");

		IAPReset();
	        mdelay(200);	

		res = EnterISPMode(private_ts->client, isp_cmd, sizeof(isp_cmd));	 //enter ISP mode

	res = i2c_master_recv(private_ts->client, recovery_buffer, 4);   //55 aa 33 cc 
	pr_k("[ELAN] recovery byte data:%x,%x,%x,%x \n",recovery_buffer[0],recovery_buffer[1],recovery_buffer[2],recovery_buffer[3]);			

        mdelay(10);
#if 0
		//Check IC's status is IAP mode(55 aa 33 cc) or not
		res = CheckIapMode();	 //Step 1 enter ISP mode
		if (res == -1) //CheckIapMode fail
		{	
			checkCnt ++;
			if (checkCnt >= 5)
			{
				pr_k("[ELAN] ERROR: CheckIapMode %d times fails!\n", IAPRESTART);
				return E_FD;
			}
			else
			{
				pr_k("[ELAN] CheckIapMode retry %dth times! And restart IAP~~~\n\n", checkCnt);
				goto IAP_RESTART;
			}
		}
		else
			pr_k("[ELAN]  CheckIapMode ok!\n");
#endif
//	} else
//		pr_k("[ELAN] Firmware upgrade recovery mode !\n");
	// Send Dummy Byte	
	pr_k("[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		pr_k("[ELAN] dummy error code = %d\n",res);
	}	
	mdelay(50);


	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
PAGE_REWRITE:
#if 1 
		// 8byte mode
		//szBuff = fw_data + ((iPage-1) * PageSize); 
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
	//			pr_k("[ELAN] byte %d\n",byte_count);	
	//			pr_k("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;

				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 8);
			}
			else
			{
	//			pr_k("byte %d\n",byte_count);
	//			pr_k("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 0 // 132byte mode		
		szBuff = file_fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
#if 1
		if(iPage==249 || iPage==1)
		{
			mdelay(300); 			 
		}
		else
		{
			mdelay(50); 			 
		}
#endif	
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) 
		{
			mdelay(50); 
			pr_k("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE ) 
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					pr_k("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
					pr_k("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					curIndex = curIndex - PageSize;
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
					pr_k("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
					pr_k("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{       pr_k("  data : 0x%02x ",  data);  
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

		mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	//if (IAPReset() > 0)
	pr_k("[ELAN] Update ALL Firmware successfully!\n");
	return 0;
}

int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 0;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0,j;
	uint8_t data;
	//struct timeval tv1, tv2;
	int restartCnt = 0, checkCnt = 0; // For IAP_RESTART
	//uint8_t recovery_buffer[4] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34
//	uint8_t fw_chose=0; 
	//uint8_t *fw_data_p;// ckt LiuHuojun 2013.1.14 16:05 ����FW ID�ж��ǻ��𴨻��ǵ�������
	  
	dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
	IAP_RESTART:	
	//reset
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(20);
	
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	//msleep(300);    // waiting for Re-Calibration (100m ~ 300m second)
	msleep(500);  //LiuHuojun 

	__hello_packet_handler(private_ts->client);
	

	data=0x15;//Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	if(RECOVERY!= 0x80)
	{
	  	pr_tp(TPD_DEVICE "[ELAN] Firmware upgrade normal mode !\n");
	
			res = EnterISPMode(private_ts->client, isp_cmd, sizeof(isp_cmd));	 //enter ISP mode


	//res = i2c_master_recv(private_ts->client, recovery_buffer, 4);   //55 aa 33 cc 
	//pr_tp(TPD_DEVICE "[ELAN] recovery byte data:%x,%x,%x,%x \n",recovery_buffer[0],recovery_buffer[1],recovery_buffer[2],recovery_buffer[3]);		

	//mdelay(10);

	} 
	else
		pr_tp(TPD_DEVICE "[ELAN] Firmware upgrade recovery mode !\n");

	#if 1
	//Chech IC's status is 55 aa 33 cc
	msleep(5); //LiuHuojun 20130528 �ر�log���޷�����������
	res = CheckIapMode();	 //Step 1 enter ISP mode
	if (res == -1) //CheckIapMode fail
	{	
	checkCnt ++;
	if (checkCnt >= 5)
	{
	pr_tp(TPD_DEVICE "[ELAN] ERROR: CheckIapMode times fails!\n");
	return E_FD;
	}
	else
	{
	pr_tp(TPD_DEVICE "[ELAN] CheckIapMode retry %dth times! And restart IAP~~~\n\n", checkCnt);
	goto IAP_RESTART;
	}
	}
	else
	pr_tp(TPD_DEVICE "[ELAN]  CheckIapMode ok!\n");
#endif
	
	// Send Dummy Byte	
	//pr_tp(TPD_DEVICE "[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);

	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
	pr_tp(TPD_DEVICE "[ELAN] dummy error code = %d\n",res);
	}	
	mdelay(50);

    mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//ckt guoyi add for reset wdt 2013-8-10, MTK ALPS00868952

	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
	PAGE_REWRITE:
#if 1 // 8byte mode
	// 8 bytes
	//szBuff = fw_data + ((iPage-1) * PageSize); 
	for(byte_count=1;byte_count<=17;byte_count++)
	{
	if(byte_count!=17)
	{		
	//			pr_tp(TPD_DEVICE "[ELAN] byte %d\n",byte_count);	
	//			pr_tp(TPD_DEVICE "curIndex =%d\n",curIndex);
			szBuff = fw_data_p + curIndex;
	curIndex =  curIndex + 8;

	//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
	res = WritePage(szBuff, 8);
	}
	else
	{
	//			pr_tp(TPD_DEVICE "byte %d\n",byte_count);
	//			pr_tp(TPD_DEVICE "curIndex =%d\n",curIndex);
			szBuff = fw_data_p + curIndex;
	curIndex =  curIndex + 4;
	//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
	res = WritePage(szBuff, 4); 
	}
	} // end of for(byte_count=1;byte_count<=17;byte_count++)
#else 
	// 132byte mode		
	//szBuff = file_fw_data + curIndex;
  szBuff = fw_data_p + curIndex;
	curIndex =  curIndex + PageSize;
	res = WritePage(szBuff, PageSize);
#endif

#if 1
	if(iPage==249 || iPage==1)
	{
		mdelay(300); 			 
	}
	else
	{
		mdelay(50); 			 
	}
#endif	
	res = GetAckData(private_ts->client);

	if (ACK_OK != res) 
	{
			mdelay(50); 
			pr_tp(TPD_DEVICE "[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE ) 
			{
			rewriteCnt = rewriteCnt + 1;
			if (rewriteCnt == PAGERETRY)
			{
			pr_tp(TPD_DEVICE "[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
			return E_FD;
			}
			else
			{
			pr_tp(TPD_DEVICE "[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
			curIndex = curIndex - PageSize;
			goto PAGE_REWRITE;
			}
			}
			else
			{
			restartCnt = restartCnt + 1;
			if (restartCnt >= 5)
			{
			pr_tp(TPD_DEVICE "[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
			return E_FD;
			}
			else
			{
			pr_tp(TPD_DEVICE "[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
			goto IAP_RESTART;
			}
			}
	}
	else
		{       
		//pr_tp(TPD_DEVICE "  data : 0x%02x ",  data);  
		rewriteCnt=0;
		print_progress(iPage,ic_num,i);
	}

	mdelay(10);
	mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//ckt guoyi add for reset wdt 2013-8-10, MTK ALPS00868952
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

    

    for(j=0;FW_AllInfor[j][0]!=0;j++)
    {
         if(FW_AllInfor[j][0] == TPFW_ID)
         {
             FW_VERSION = FW_AllInfor[j][1];
             FW_ID = TPFW_ID;
             break;
         }
    }

	return 0;
}

static int request_firmware(struct i2c_client *client)
{
	int ret, i;
	u8 buf[4];
	int temp1, temp2;
	//uint8_t *fw_data_p;
	pr_tp(TPD_DEVICE "[elan]: Check Firmware Version.\n");

	//update no into esd
	update_flage =  1;
	//lock work function
//	disable_irq(CUST_EINT_TOUCH_PANEL_NUM);
//	cancel_work_sync(&private_ts->work);
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		mdelay(20);
    
	pr_tp(TPD_DEVICE "[elan]: ISP Procedure Step 1: Reset TP ----------------------\n");
#if 1

	if(FW_ID == 0x14fa)
	{
		fw_data_p = file_fw_data_hrc;
		printk("[elan]: file_fw_data_hrc\n");
	}
	else if(FW_ID == 0x14f7)
	{
		fw_data_p = file_fw_data_dm;
		printk("[elan]: file_fw_data_dm\n");
	}
	else
	{
		fw_data_p = file_fw_data_hrc;
		printk("[elan]: unknow fw id ,use default fw\n");
	}

	temp1 = (fw_data_p[0x7bd0]);
	temp2 = (int)(fw_data_p[0x7bd0+1])<<8 ;
	NEW_FW_VERSION = temp1 + temp2;
	printk("[elan]: FW_VERSION =0x%x, New_FW_VER =0x%x, FW_ID =0x%x \n",FW_VERSION, NEW_FW_VERSION, FW_ID);

#else
#if defined(ISP_IN_DRIVER_NOJUDGE_FOR_HRC)
  if((FW_VERSION != 0x0d01 || FW_ID != 0x14f0) || RECOVERY == 0x80)
#elif defined(ISP_IN_DRIVER_NOJUDGE_FOR_DM)
  if((FW_VERSION != 0x0d01 || FW_ID != 0x14f1) || RECOVERY == 0x80)
#else

   for(i=0;FW_Infor[i][0]!=0;i++)
   	{
       if(FW_Infor[i][0]==FW_ID && FW_Infor[i][1]==FW_VERSION)
       	break;
   	}
#endif
#endif
 //  if(0 != FW_Infor[i][0] || RECOVERY == 0x80)
 if(FW_VERSION != NEW_FW_VERSION || RECOVERY == 0x80)
{
	///////////////////////////////////////////
	Update_FW_One(client, RECOVERY);

    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(20);
    
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(500);    // waiting for Re-Calibration (100m ~ 300m second)
    
    ret = i2c_master_recv(ektf_i2c_client, buf, sizeof(buf));
    
    if(ret != 4)
    {
        pr_tp(TPD_DEVICE "[elan] %s Received package error.\n", __func__);
        return false;
    }
    
	pr_tp(TPD_DEVICE "[elan] hello packet: %x %x %x %x %x\n", buf[0],buf[1], buf[2],buf[3], buf[4]);
    
    //mdelay(50);
// End TP Initial
    pr_tp(TPD_DEVICE "[elan] Update ALL Firmware successfully!!!!!!!");
    //mdelay(2000);


    
 		}   
 	else{
			pr_tp(TPD_DEVICE "[elan] +++++++++++++++++FW_VERSION is same+++++++++++++++++++++++++++");
 	}
	
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    update_flage =  0;
	return 0;
}

#endif	//End Elan ISP_IN_DRIVER


static  void tpd_down(int x, int y, int p) 
{
    // input_report_abs(tpd->dev, ABS_PRESSURE, p);
    if (RECOVERY_BOOT != get_boot_mode())
    {
        input_report_key(tpd->dev, BTN_TOUCH, 1);
    }
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //pr_tp(TPD_DEVICE "D[%4d %4d %4d] ", x, y, p);
	 pr_tp(TPD_DEVICE "[elan]: Touch Down[%4d %4d %4d]\n", x, y, p);
	 input_mt_sync(tpd->dev);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
       tpd_button(x, y, 1);  
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         msleep(50);
		 pr_tp(TPD_DEVICE "D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }

 static  void tpd_up(int x, int y,int *count)
{
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
    //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //pr_tp(TPD_DEVICE "U[%4d %4d %4d] ", x, y, 0);		  
		   pr_tp(TPD_DEVICE "[elan]: Touch Up[%4d %4d %4d]\n", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
        tpd_button(x, y, 0); 
     }   		 
}

static bool ektf2k_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    
    ret = i2c_master_recv(ektf_i2c_client, pbt_buf, dw_lenth);

    if(ret <= 0)
    {
        pr_tp(TPD_DEVICE "[elan] %s Received package error.\n", __func__);
        return false;
    }

    return true;
}

static int tpd_touchinfo(struct touch_info *cinfo)
{
    SHORT_TOUCH_STATE ShortTouchState;
    char buf[PACKET_SIZE] = {0};
    unsigned int  temp = 0;
	int index = 0;
	int i, j = 0;
	u16 x, y;	
	//u16 idx;
	u16 fbits = 0;

    ektf2k_i2c_read(buf, 8);
	if ((buf[0] == 0x6D) || (buf[0] == 0x5D))	//for five finger
	{
		mdelay(1);
		ektf2k_i2c_read(buf + 8, 8);
		mdelay(1);
		ektf2k_i2c_read(buf + 16, 2);
 	}

//added by luosen 
#if defined(TP_REPLACE_ALSPS)
		//face close turn off lcd led
		if(buf[0] ==0xFA && buf[1] ==0xCE && buf[2] ==0xAA && buf[3] ==0xAA){
			ckt_tp_replace_ps_close = 1;
			pr_tp(TPD_DEVICE "face close0n\n");
		return 0;
		}

		//face away turn on lcd led
		if(buf[0] ==0xFA && buf[1] ==0xCE && buf[2] ==0x55 && buf[3] ==0x55){
			ckt_tp_replace_ps_close = 0;
			pr_tp(TPD_DEVICE "face away0n\n");
		return 0;
		}
#endif
//end


#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:20:41
	if ( 
		    (buf[0] == 0x52) 
		    && 
		    ((buf[1] &0xf0) == 0xf0)
// �� �� 2012��12��03�� 10:24:18		    && 
// �� �� 2012��12��03�� 10:24:18		    (buf[2] == 0x11)
// �� �� 2012��12��03�� 10:24:18		    && 
// �� �� 2012��12��03�� 10:24:18		    (buf[3] == 0xe1)
		)
	{
		esdflag=1;
		pr_tp(TPD_DEVICE "=============esd wakeup\n");
		wake_up_interruptible(&esd_wait_queue);
		return 0;
	}
#endif /* __CTP_ESD__ */

	// james: elan debug message
	pr_tp(TPD_DEVICE "[elan_luosen] package: ");
	for (i = 0; i < PACKET_SIZE; i++)
		pr_tp(TPD_DEVICE "%x ", buf[i]);
	pr_tp(TPD_DEVICE "\n");
	
	//get the number of the touch points
// for 5 fingers	
    if ((buf[0] == 0x5D) || (buf[0] == 0x6D))
    {
    	finger_num = 5;
        point_num = buf[1] & 0x07; 
        pr_tp(TPD_DEVICE "[elan] finger_num = %d,point_num =%d\n", finger_num, point_num);
        for (i = 0; i < point_num; i++) 
		{		
				elan_ktf2k_ts_parse_xy(&buf[ 2 + 3*i ], &x, &y);
				x = x * 480 / X_RESOLUTION;
				y = y * 800 / Y_RESOLUTION;
                if (x >= 480) x = 479;
                if (y >= 800) y = 799;

				cinfo->x[ i ] = x;
				cinfo->y[ i ] = y;
				pr_tp(TPD_DEVICE "[elan] 5finger's cinfo->x[%d] = %d, cinfo->y[%d] = %d\n", i, cinfo->x[i], i, cinfo->y[i]);
		} // end for
		cinfo->key_val = buf[17] & 0xfe;
		pr_tp(TPD_DEVICE "[ELAN] KEY value =  %d\n", cinfo->key_val);
    }
    else if(buf[0] == 0x5a)
    {
// for 2 fingers  
	
	finger_num = 2;    
      	point_num = buf[7] & 0x03;
     	switch (point_num)
		{
			case 0:
				point_num=0;
				break;
			case 3:
				point_num=2;
				break;
			case 2:
				point_num=1;
				j = 1;
				break;
			default:
				point_num=1;
		} 
// ����8bit��������1�ĸ���,�������ȼ� �� �� 2012��11��29�� 17:13:59
// �� �� 2012��11��29�� 17:13:41		point_num = (point_num & 0x55) + ((point_num >> 1) & 0x55);
// �� �� 2012��11��29�� 17:13:41		point_num = (point_num & 0x33) + ((point_num >> 2) & 0x33);
// �� �� 2012��11��29�� 17:13:41		point_num = (point_num & 0x0F) + ((point_num >> 4) & 0x0F);

		pr_tp(TPD_DEVICE "[elan] point_num =%d\n",point_num);      		
		for(i = 0; i < point_num; i++, j++)
		{
			elan_ktf2k_ts_parse_xy(&buf[ 1 + 3*j ], &x, &y);
			x = x * 480 / X_RESOLUTION;
			y = y * 800 / Y_RESOLUTION;
            if (x >= 480) x = 479;
            if (y >= 800) y = 799;
		if(x<=0) x=1;
		if(y<=0) y=1;
			cinfo->x[ i ] = x;
			cinfo->y[ i ] = y;
			pr_tp(TPD_DEVICE"[elan] cinfo->x[%d] = %d, cinfo->y[%d] = %d\n", i, cinfo->x[i], i, cinfo->y[i]);
		}
		
		is_down = buf[7] & 0x03;
		cinfo->key_val = buf[7] & 0xfc;
		pr_tp(TPD_DEVICE "[ELAN] KEY value =  %d\n", cinfo->key_val);
		pr_tp(TPD_DEVICE "[ELAN] is_down =  %d\n", is_down);
	}
	else{
		 pr_tp(TPD_DEVICE "unknow paket buf[0] %x\n", buf[0]);
	}
		
	return 1;

}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];
	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

// james: ts_work_fun
static int touch_event_handler(void *unused)
{
    struct touch_info cinfo;
    int touch_state = 3;
    int button_state = 0;
    unsigned long time_eclapse;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
	int last_key = 0;
	int key;
	int index = 0;

    int tmp_x = 0;
    int tmp_y = 0;

    do
    {
        //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        if(tpd_touchinfo(&cinfo))
        {
           pr_tp(TPD_DEVICE "[elan]touch_event_handler X = %d,Y = %d, point_num = %d\n", cinfo.x[0], cinfo.y[0], point_num);
           if(point_num > 0 && button_state == 0)
            {
				for(index =0 ; index <point_num ; index ++)
                {
   					tpd_down(cinfo.x[index], cinfo.y[index], index+1);
					pr_tp(TPD_DEVICE "[elan] %d : X = %d,Y = %d, point_num = %d\n",index,  cinfo.x[index], cinfo.y[index], point_num);
			
                }
                pr_k("msg_press >=2 points--->\n");
                input_sync(tpd->dev);
            }
            else if(point_num == 0)
            {
#ifdef TPD_HAVE_BUTTON
		pr_tp(TPD_DEVICE "switch key button_state =%d  key_val =%d\n",button_state, cinfo.key_val);

#if 1 //ckt guoyi 2013-5-7 add for TPD vitual button
                if (0 == button_state)
                {
                    switch(cinfo.key_val)
                        {
                        case 0x04:
                            tmp_x = 100;
                            tmp_y = 1050;
                            tpd_down(tmp_x, tmp_y, 1);
                            button_state = cinfo.key_val;
                            break;
        
                        case 0x08:
                            tmp_x = 300;
                            tmp_y = 1050;
                            tpd_down(tmp_x, tmp_y, 1);
                            button_state = cinfo.key_val;
                            break;
        
                        case 0x10:
                            tmp_x = 500;
                            tmp_y = 1050;
                            tpd_down(tmp_x, tmp_y, 1);
                            button_state = cinfo.key_val;
                            break;
        
                        default:
                            tpd_up(cinfo.x[0], cinfo.y[0], 0);
                            pr_tp("NO TOUCH_KEY");
                            break;
                        }
                }
                else
                {
                    tpd_up(tmp_x, tmp_y, 0);
                    tmp_x = 0;
                    tmp_y = 0;
                    button_state = 0;
                }
#else
	 	if (button_state == 0) {
			
			switch (cinfo.key_val) {
			    case 0x04:
					pr_tp(TPD_DEVICE "[elan]KEY back 1\n");
		                        input_report_key(tpd->dev, tpd_keys_local[0], 1);
		                        button_state = tpd_keys_local[0];
					break;
				case 0x08:
					pr_tp(TPD_DEVICE "[elan]KEY home 1\n");
		                        input_report_key(tpd->dev, tpd_keys_local[1], 1);
		                        button_state = tpd_keys_local[1];
					break;
				case 0x10:
					pr_tp(TPD_DEVICE "[elan]KEY menu 1\n");
		                        input_report_key(tpd->dev, tpd_keys_local[2], 1);
		                        button_state = tpd_keys_local[2];
					break;
					default:
								tpd_up(cinfo.x[0], cinfo.y[0], 0);
								//input_sync(tpd->dev);
								break;
			}
		}
		else if(button_state != 0)
		{
			if(button_state == tpd_keys_local[0])
			{
				pr_tp(TPD_DEVICE "[elan]KEY back 0\n");
		                input_report_key(tpd->dev, tpd_keys_local[0], 0);
		                button_state = 0;
			}
			else if(button_state == tpd_keys_local[1])
			{
				pr_tp(TPD_DEVICE "[elan]KEY home 0\n");
		                input_report_key(tpd->dev, tpd_keys_local[1], 0);
		                button_state = 0;
			}
			else if(button_state == tpd_keys_local[2])
			{
				pr_tp(TPD_DEVICE "[elan]KEY menu 0\n");
		                input_report_key(tpd->dev, tpd_keys_local[2], 0);
		                button_state = 0;
			}	

		}
#endif						

#endif

		   //tpd_up(cinfo.x[0], cinfo.y[0], 0);
                pr_ch("[elan]release --->\n");
                //input_mt_sync(tpd->dev);
              if ( tpd != NULL && tpd->dev != NULL )
                input_sync(tpd->dev);

            }
        }
/*
        pr_tp(TPD_DEVICE  "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, FW_VERSION);
	pr_tp(TPD_DEVICE  "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, FW_ID);
	pr_tp(TPD_DEVICE  "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, X_RESOLUTION, Y_RESOLUTION);
	pr_tp(TPD_DEVICE  "[elan] %s: bootcode version: 0x%4.4x\n",
			__func__, BC_VERSION);
*/
    }
    while(!kthread_should_stop());

    return 0;
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
    pr_tp("luosen TPD int\n");
    //rintk("luosentp ektf g_fw_err=%d\n",g_fw_err);
#if defined(__CTP_ESD_ORIGIN__)
 mod_timer(&elan_timer,jiffies+5*HZ);
 #endif
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}


#if defined(__CTP_ESD_ORIGIN__)
///add esd
static void elan_timer_function(unsigned long v)
{
	pr_tp(TPD_DEVICE "esd timer in\n");
	queue_work(elan_wq, &work);
	mod_timer(&elan_timer,jiffies+5*HZ);
}
#endif


#if defined(TP_REPLACE_ALSPS_TEST)
static ssize_t elan_ktf2k_face_mod_on(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int res = 0;
	//struct elan_ktf2k_ts_data *ts = private_ts;
	char buf_on[] = {0x54, 0xC1, 0x00, 0x01};

	res = i2c_master_send(ektf_i2c_client, buf_on, sizeof(buf_on));
	if (res != sizeof(buf_on)) {
		pr_tp(TPD_DEVICE "[elan] turn on face mod faild\n");
	} else {
		pr_tp(TPD_DEVICE "[elan] turn on face mod ok\n");
	}
	return res;
}

static DEVICE_ATTR(faceon, S_IRUGO, elan_ktf2k_face_mod_on, NULL);

static ssize_t elan_ktf2k_face_mod_off(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char buf_off[] = {0x54, 0xC0, 0x00, 0x01};
	ssize_t ret = 0;
	ret = i2c_master_send(ektf_i2c_client, buf_off, sizeof(buf_off));
	if (ret != sizeof(buf_off)) {
		pr_tp(TPD_DEVICE "[elan] turn off face mod faild\n");
	} else {
		pr_tp(TPD_DEVICE "[elan] turn off face mod ok\n");
	}
	return ret;
}
static DEVICE_ATTR(faceoff, S_IRUGO, elan_ktf2k_face_mod_off, NULL);
static struct kobject *android_touch_kobj;
static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_tp(TPD_DEVICE KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_faceon.attr);
	if (ret) {
		pr_tp(TPD_DEVICE KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_faceoff.attr);
	if (ret) {
		pr_tp(TPD_DEVICE KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

#endif


#if defined(__CTP_ESD_ORIGIN__)
static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};
	int rc;
	if ((i2c_master_send(ektf_i2c_client, cmd, 4)) != 4) {
		pr_tp(TPD_DEVICE "[elan esd]%s: i2c_master_send failed\n", __func__);

		pr_tp(TPD_DEVICE "[elan esd]%s:reset tp now\n");
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
		msleep(20);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
		msleep(100);
		return -EINVAL;
	}
	else{
		pr_tp(TPD_DEVICE "[elan esd]%s:normal \n", __func__);
	}
}
#endif



#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:22:28
bool CTP_EsdCheck(struct i2c_client *client)
{
    int reg_val[4] = {0x52,0x00,0x00,0x00};

	uint8_t id[] = {0x53, 0xf0, 0x00, 0x01};
	int len=0;
	
	
	len = i2c_master_send(client, id, 4);
	
	if (len != sizeof(id)/sizeof(id[0])) {
		 pr_tp(TPD_DEVICE "================ ERROR: read id command error len=%d  %s %x\n", len,client->name,client->addr);
			return true;
	}
	else
	{
        long ret = wait_event_interruptible_timeout(esd_wait_queue, 
                                            esdflag!=0,
                                            2*HZ);
		esdflag=0;
		if(ret==0)// timeout �� �� 2012��11��29�� 15:32:00
		{
			 pr_tp(TPD_DEVICE "================ timeout\n");
			return true;
			
		}
		else 
		{
			 pr_tp(TPD_DEVICE "================ ok\n");
		}
	}

	return 0;
}

bool CTP_EsdRecover(void)
{
	pr_tp(TPD_DEVICE "CTP_EsdRecover \n");

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);

    msleep(200);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return true;
}
static int ctp_esd_recovery_kthread(void *data)
{
	//struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE };
	//sched_setscheduler(current, SCHED_RR, &param);

	struct i2c_client *client=data;
	
    pr_tp(TPD_DEVICE "enter esd_recovery_kthread()\n");
    for( ;; ) {

//		pr_tp(TPD_DEVICE "======================%s", client->name);
		
        if (kthread_should_stop())
            break;

        pr_tp(TPD_DEVICE "sleep start in esd_recovery_kthread()\n");
        msleep(2000);       //2s
        pr_tp(TPD_DEVICE "sleep ends in esd_recovery_kthread()\n");

        if(!esd_kthread_pause)
        {
            if(is_suspended || update_flage == 1)
            {
                pr_tp(TPD_DEVICE "is_suspended in esd_recovery_kthread()\n");
                continue;
            }

            if (down_interruptible(&sem_suspend)) {
                pr_tp(TPD_DEVICE TPD_DEVICE "can't get sem_suspend in esd_recovery_kthread()\n");
                continue;
            }


          	if(is_suspended || update_flage == 1)
            {
                up(&sem_suspend);
                pr_tp(TPD_DEVICE "is_suspended in esd_recovery_kthread()\n");
                continue;
            }
                
           ///execute ESD check and recover flow
           pr_tp(TPD_DEVICE "CTP_EsdCheck starts\n");
           if(CTP_EsdCheck(client))
           {    
                if(!esd_kthread_pause)
                {
                    pr_tp(TPD_DEVICE "CTP_EsdRecover starts\n");
                    CTP_EsdRecover();
                    pr_tp(TPD_DEVICE "CTP_EsdRecover ends\n");
// �� �� 2012��11��26�� 11:07:12                    if(CTP_EsdCheck(client))
// �� �� 2012��11��26�� 11:07:12                    {
// �� �� 2012��11��26�� 11:07:12                        pr_tp(TPD_DEVICE "OOps! CTP_EsdRecover dose not work\n");
// �� �� 2012��11��26�� 11:07:12                        esd_kthread_pause = TRUE;
// �� �� 2012��11��26�� 11:07:12                    }
                }
                else
                {
                    /// this is error handling for some special case
                }
           }

           pr_tp(TPD_DEVICE "DISP_EsdCheck ends\n");

           up(&sem_suspend);
       }
    }
    pr_tp(TPD_DEVICE "exit esd_recovery_kthread()\n");
    return 0;
}
#endif /* __CTP_ESD__ */


static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
		//pr_tp(TPD_DEVICE "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	pr_tp(TPD_DEVICE  "[elan]%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != 0x52)
			return -EINVAL;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		pr_tp(TPD_DEVICE  "[elan] %s: Int poll failed!\n", __func__);
		RECOVERY=0x80;
		return RECOVERY;
		//return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	pr_tp(TPD_DEVICE "[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
             RECOVERY=0x80;
	     return RECOVERY; 
	}
	if(buf_recv[2] == 0x00){
				RECOVERY=0x80;
	     return RECOVERY; 
	}
			

	return 0;
}

//LiuHuojun add 20130916 for tp fw id
extern int get_rtc_spare_tpfwid_value(void);
extern int set_rtc_spare_tpfwid_value(int val);
int GetAndSet_TPFW_ID_info(int fw_id)
{ //commented by assusdan
    //TPFW_ID = get_rtc_spare_tpfwid_value();
    //pr_tp(TPD_DEVICE "[elan] get_rtc_spare_fg_value:[0x%x],FW_ID=0x%x\n",TPFW_ID,FW_ID);
   // if(0!=FW_ID)
   // {
    //    TPFW_ID = FW_ID;
    //    set_rtc_spare_tpfwid_value(FW_ID);
    ////}
    return 1;
}
//add by sen.luo for chipinfo 2013.09.24 start
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client =ektf_i2c_client;
//	char strbuf[256];
	if(NULL == client)
	{
		printk("i2c client is null!!\n");
		return 0;
	}
	if(FW_ID == 0x14fa)
		{
       return sprintf(buf,"[HRC ektf2232] ID:0x%x VER:0x%x\n",FW_ID, FW_VERSION);
	       }
	else if(FW_ID == 0x14f7)
		{
	return sprintf(buf,"[DM ektf2232] ID:0x%x VER:0x%x\n",FW_ID, FW_VERSION);	
		}
	else
		{
	return sprintf(buf,"[unknown] ID:0x%x VER:0x%x\n",FW_ID, FW_VERSION);		
		}
	//return sprintf(buf, "%s\n", strbuf);        
}

static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static struct driver_attribute *ektf2k_attr_list[] = {
    &driver_attr_chipinfo,
  //  &driver_attr_als,
 //   &driver_attr_ps,       
  //  &driver_attr_config,
 //   &driver_attr_status,
  //  &driver_attr_pscalibrate,
};

static int ektf2k_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ektf2k_attr_list)/sizeof(ektf2k_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ektf2k_attr_list[idx])))
		{            
			printk("driver_create_file (%s) = %d\n", ektf2k_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int ektf2k_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ektf2k_attr_list)/sizeof(ektf2k_attr_list[0]));
	if (!driver)
	return -EINVAL;
	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ektf2k_attr_list[idx]);
	}
	return err;
}
//add by sen.luo for chipinfo 2013.09.24 end
static int __fw_packet_handler(struct i2c_client *client)
{
	
//	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[] = {0x53, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};

pr_tp(TPD_DEVICE KERN_INFO "[elan] %s: n",
			__func__);
// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
//	ts->fw_ver = major << 8 | minor;
	FW_VERSION = major << 8 | minor;
// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	//ts->fw_id = major << 8 | minor;
	FW_ID = major << 8 | minor;
// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	//ts->x_resolution =minor;
	X_RESOLUTION = minor;
	
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	//ts->y_resolution =minor;
	Y_RESOLUTION = minor;

// Bootcode version
	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	//ts->bc_ver = major << 8 | minor;
	BC_VERSION = major << 8 | minor;
	
	pr_tp(TPD_DEVICE  "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, FW_VERSION);
	pr_tp(TPD_DEVICE  "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, FW_ID);
	pr_tp(TPD_DEVICE  "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, X_RESOLUTION, Y_RESOLUTION);
	pr_tp(TPD_DEVICE  "[elan] %s: bootcode version: 0x%4.4x\n",
			__func__, BC_VERSION);
	return 0;
}


static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;
	//int count = 3
	
//retry:
    
	rc = __hello_packet_handler(client);
	pr_tp(TPD_DEVICE "[elan] hellopacket's rc = %d\n",rc);

	mdelay(10);
	if (rc != 0x80){
	    rc = __fw_packet_handler(client);
	    if (rc < 0)
		    pr_tp(TPD_DEVICE "[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
	    pr_tp(TPD_DEVICE "[elan] %s: firmware checking done.\n", __func__);
//Check for FW_VERSION, if 0x0000 means FW update fail!
	    if ( FW_VERSION == 0x00)
	    {
		rc = 0x80;
		pr_tp(TPD_DEVICE "[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
	    }
      }
	GetAndSet_TPFW_ID_info(FW_ID);
	return rc;
}

struct platform_driver ektf2k_tp_driver;

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK;
    int fw_err;
    int ret;
    char buf[PACKET_SIZE] = {0};
#ifdef ISP_IN_DRIVER
    static struct elan_ktf2k_ts_data ts;
#endif

    //   char data;
    //    client->timing = 400;
    ektf_i2c_client = client;
#ifdef ISP_IN_DRIVER
	private_ts = &ts;
    private_ts->client = client;
    //private_ts->addr = 0x2a;
#endif

#if 0
    /*LDO enable*/
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif
//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
                   hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
                   hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
                   hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 

         msleep(10);
// Reset Touch Pannel
pr_tp(TPD_DEVICE "ELAN enter tpd_probe_ ,the i2c addr=0x%x", client->addr);
         mt_set_gpio_mode( GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO );
         mt_set_gpio_dir( GPIO_CTP_RST_PIN, GPIO_DIR_OUT );
         mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
         mdelay(50);
         mt_set_gpio_out( GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
         mdelay(300);
// End Reset Touch Pannel       
         
	
#ifdef HAVE_TOUCH_KEY
int retry;
	for(retry = 0; retry < 2; retry++)
	{
		input_set_capability(tpd->dev,EV_KEY,tpd_keys_local[retry]);
	}
#endif

	client->addr |= I2C_ENEXT_FLAG;
	client->timing =  100;
	
// Setup Interrupt Pin
         mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
         mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
         mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
         mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
        // mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
         mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
         mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
         //mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);                    
         mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
         mdelay(100);
// End Setup Interrupt Pin 
   	fw_err = elan_ktf2k_ts_setup(client); 
	if (fw_err < 0) {
		pr_tp(TPD_DEVICE KERN_INFO "[elan] No Elan chip inside\n");
		return -1;
	}
    
    ret = i2c_master_recv(ektf_i2c_client, buf, 8);

    if(ret <= 0)
    {
        pr_tp(TPD_DEVICE "[elan] %s Received package error.\n", __func__);
        return false;
    }
	
	
    tpd_load_status = 1;
#if 0 /*RESET RESOLUTION*/
	input_set_abs_params(ts->input_dev, ABS_X, 0,  X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, Y_RESOLUTION, 0, 0);
#endif 
    ektf_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if(IS_ERR(ektf_thread))
    {
        retval = PTR_ERR(ektf_thread);
        pr_tp(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
    }



    pr_tp(TPD_DEVICE " ELAN Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef ISP_IN_DRIVER
// Firmware Update
	// MISC
  	ts.firmware.minor = MISC_DYNAMIC_MINOR;
  	ts.firmware.name = "elan-iap";
  	ts.firmware.fops = &elan_touch_fops;
  	ts.firmware.mode = S_IRWXUGO; 
   	
  	if (misc_register(&ts.firmware) < 0)
  		pr_k("[elan] misc_register failed!!");
  	else
  	  pr_k("[elan] misc_register finished!!"); 
// End Firmware Update	
#endif
    
#if defined(__CTP_ESD_ORIGIN__)
      //for esd
	init_timer(&elan_timer);
	elan_timer.function = elan_timer_function;
	elan_timer.expires = jiffies + 30*HZ;
	add_timer(&elan_timer);
	elan_wq = create_singlethread_workqueue("elan_wq");
	if (!elan_wq) {
		pr_tp(TPD_DEVICE KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
		int err = -ENOMEM;
		//goto err_create_wq_failed;
	}

	INIT_WORK(&work, elan_ktf2k_ts_work_func);
	//end esd
#endif
#ifdef TP_FIRMWARE_UPDATE
	
  int err;
	err = request_firmware(client);
	if(err <0 )
		pr_tp(TPD_DEVICE "[elan] firmware upgraded failed\n");
	else
		pr_tp(TPD_DEVICE "[elan] The firmware upgraded completed\n");
		
#endif
#if defined(TP_REPLACE_ALSPS_TEST)
elan_ktf2k_touch_sysfs_init();
#endif

#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:23:20
   ctp_esd_recovery_task = kthread_create(
    			   ctp_esd_recovery_kthread, client, "CTP esd_recovery_kthread");

    if (IS_ERR(ctp_esd_recovery_task)) {
        pr_tp(TPD_DEVICE "CTP ESD recovery task create fail\n");
    }
    else {
    	wake_up_process(ctp_esd_recovery_task);
    }
#endif /* __CTP_ESD__ */
//add by sen.luo for chipinfo 2013.09.24 start
	if(ret = ektf2k_create_attr(&ektf2k_tp_driver.driver))
	{
		pr_tp(TPD_DEVICE,"create attribute ret = %d\n", ret);
	}
//add by sen.luo for chipinfo 2013.09.24 end	
    show_ektf2k_name();
    return 0;
}

static int tpd_remove(struct i2c_client *client)

{
   int err;	
    pr_tp(TPD_DEVICE "TPD removed\n");
//add by sen.luo for chipinfo 2013.09.24 start	
   if(err == ektf2k_delete_attr(&ektf2k_tp_driver.driver))
	{
		pr_tp("ektf2k_delete_attr fail: %d\n", err);
	} 
   //add by sen.luo for chipinfo 2013.09.24 end
    return 0;
}


static int tpd_local_init(void)
{

// james    
 	bootMode = get_boot_mode();
	if(SW_REBOOT == bootMode)
		bootMode = NORMAL_BOOT;
// end james
    pr_tp(TPD_DEVICE "[elan]: I2C Touchscreen Driver init\n");

    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        pr_tp(TPD_DEVICE "[elan]: unable to add i2c driver.\n");
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if 0
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
#endif
    pr_tp("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;
    return 0;
}

static int ektf_tpd_resume(struct i2c_client *client)
{
    int retval = TPD_OK;
    int res = 0;
    pr_tp(TPD_DEVICE "TPD wake up\n");
    uint8_t cmd[] = {0x54, 0x58, 0x00, 0x01};
    pr_tp(TPD_DEVICE "[elan] TPD wake up\n");

	if(update_flage == 1){
		pr_tp(TPD_DEVICE  "[elan] can't enter wake up mode when tp update\n");
		return;
	}
   //if ((i2c_master_send(ektf_i2c_client, cmd, sizeof(cmd))) != sizeof(cmd)) 

#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:23:41
    if (down_interruptible(&sem_suspend)) {
        pr_tp(TPD_DEVICE " can't get semaphore in mtkfb_late_resume()\n");
        return;
    }
#endif /* __CTP_ESD__ */
    
    
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
#else
res = i2c_master_send(ektf_i2c_client, cmd, sizeof(cmd));
	
  if (res != sizeof(cmd))	 
    {
		pr_tp(TPD_DEVICE "[elan] %s: i2c_master_send failed\n", __func__);
		//return -retval;
   mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(20);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	 msleep(200);
    }
	else
		{
		pr_tp(TPD_DEVICE "[elan] %s: i2c_master_send successful\n", __func__);

		}
#endif 
   	msleep(100);
	
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:24:00
    is_suspended = false;
    
    up(&sem_suspend);
#endif /* __CTP_ESD__ */
#if defined(__CTP_ESD_ORIGIN__)
    	elan_timer.expires = jiffies + 5*HZ;
	add_timer(&elan_timer);
#endif

  #if defined(TP_REPLACE_ALSPS)
if(1==ckt_tp_replace_ps_state)
	{
		ckt_tp_replace_ps_mod_on();
		pr_k( "[elan] ckt_tp_replace_ps_mod_on\n");
	}
#endif    

    return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
    int retval = TPD_OK;
    static char data = 0x3;
uint8_t cmd[] = {0x54, 0x50, 0x00, 0x01};
    pr_tp(TPD_DEVICE "[elan]TPD enter sleep\n");

	if(update_flage == 1){
		pr_tp(TPD_DEVICE "[elan] can't enter sleep mode when tp update\n");
		return;
	}

#if defined(TP_REPLACE_ALSPS)
if(1==ckt_tp_replace_ps_state)
	{
	pr_tp(TPD_DEVICE  "[elan] can't enter sleep mode when tp replace ps\n");
	return;
	}
#endif    
#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:24:17
    if (down_interruptible(&sem_suspend)) {
        pr_tp(TPD_DEVICE "can't get semaphore in tpd_suspend()\n");
        return;
    }
	
	if(is_suspended){
		is_suspended = true;
		up(&sem_suspend);
		pr_tp(TPD_DEVICE " has been suspended\n");
		return;
	}
    is_suspended = true;
#endif /* __CTP_ESD__ */
	

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	
#ifdef TPD_CLOSE_POWER_IN_SLEEP
	hwPowerDown(TPD_POWER_SOURCE, "TP");

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
#else       // james
    if ((i2c_master_send(ektf_i2c_client, cmd, sizeof(cmd))) != sizeof(cmd)) 
    {
		pr_tp(TPD_DEVICE "[elan] %s: i2c_master_send failed\n", __func__);
		return -retval;
    }
	else
		{
		pr_tp(TPD_DEVICE "[elan] %s: i2c_master_send successful\n", __func__);
		}
#endif
	msleep(20);

#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:24:34
	up(&sem_suspend);
#endif /* __CTP_ESD__ */
	//esd
#if defined(__CTP_ESD_ORIGIN__)
	del_timer(&elan_timer);
	#endif
    return retval;
}

#if defined(TP_REPLACE_ALSPS)
int ckt_tp_replace_ps_mod_on(void)
{
	int res = 0;
	char buf_on[] = {0x54, 0xC1, 0x00, 0x01};
       ckt_tp_replace_ps_close = 0;
	res = i2c_master_send(ektf_i2c_client, buf_on, sizeof(buf_on));
	if (res != sizeof(buf_on)) {
		pr_tp(TPD_DEVICE "[elan] turn on face mod faild\n");
		ckt_tp_replace_ps_state=0;
		return 0;
	} else {
	       ckt_tp_replace_ps_state=1;
		pr_tp(TPD_DEVICE "[elan] turn on face mod ok\n");
		return 1;
	}
}

int ckt_tp_replace_ps_mod_off(void)
{
#if 0
	char buf_off[] = {0x54, 0xC0, 0x00, 0x01};
	ssize_t ret = 0;
	ckt_tp_replace_ps_close = 0;
	ret = i2c_master_send(ektf_i2c_client, buf_off, sizeof(buf_off));
	if (ret != sizeof(buf_off)) {	
		ckt_tp_replace_ps_state=1;
		pr_tp(TPD_DEVICE "[elan] turn off face mod faild\n");
		return 0;
	} else {
	       ckt_tp_replace_ps_state=0;
		pr_tp(TPD_DEVICE "[elan] turn off face mod ok\n");
		return 1;
	}
	#else
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(30);

	// for enable/reset pin
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, 1);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(300);
	ckt_tp_replace_ps_close = 0;
	ckt_tp_replace_ps_state=0;
	pr_tp(TPD_DEVICE "[elan] turn off face mod ok\n");
	return 1;
	#endif
}

int  ckt_tp_replace_ps_enable(int enable)
{
if (enable)
	{
	if(1==ckt_tp_replace_ps_mod_on())
		{
		pr_tp(TPD_DEVICE "[elan]open the ps mod successful\n");
	    return 1;
		}
	else
		{
		pr_tp(TPD_DEVICE "[elan]open the ps mod fail\n");
	    return 0;
		}
	
	}
else
	{
	if(1==ckt_tp_replace_ps_mod_off())
		{
		pr_tp(TPD_DEVICE "[elan]close the ps mod successful\n");
	    return 1;
		}
	else
		{
		pr_tp(TPD_DEVICE "[elan]close the ps mod fail\n");
	    return 0;
		}
	}
}

u16  ckt_get_tp_replace_ps_value(void)
	{
	if(1==ckt_tp_replace_ps_close)
		{
		pr_tp(TPD_DEVICE "[elan]ckt_get_tp_replace_ps_value 500\n");
		return 500;
		}
	else
		{
		pr_tp(TPD_DEVICE "[elan]ckt_get_tp_replace_ps_value 100\n");
		return 100;
		}
	
	}
#endif

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "ektf2k_mtk",       
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = ektf_tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

void show_ektf2k_name(void)
{
  int i=0;
  for(i=0;FW_AllInfor[i][0]!=0;i++)
  {
      if(FW_AllInfor[i][0]==FW_ID/* && FW_AllInfor[i][1]==FW_VERSION*/)
      	  break;
  }
  if(0 == FW_AllInfor[i][0])
  {
      sprintf(TPFW,"[unknown] ID:0x%x  VER:0x%x",FW_ID, FW_VERSION);
  }
  else if(0 == FW_AllInfor[i][2])
  {
      sprintf(TPFW,"[hrc] ID:0x%x  VER:0x%x",FW_ID, FW_VERSION);
  }
  else if(1 == FW_AllInfor[i][2])
  {
      sprintf(TPFW,"[dm] ID:0x%x  VER:0x%x",FW_ID, FW_VERSION);
  }
  else
  {
      sprintf(TPFW,"[lc] ID:0x%x  VER:0x%x",FW_ID, FW_VERSION);
  }
}
//add by sen.luo for chipinfo 2013.09.24 start
/*----------------------------------------------------------------------------*/
static int ektf2k_probe(struct platform_device *pdev) 
{
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ektf2k_remove(struct platform_device *pdev)
{

	return 0;
}
/*----------------------------------------------------------------------------*/
 struct platform_driver ektf2k_tp_driver = {
	.probe      = ektf2k_probe,
	.remove     = ektf2k_remove,    
	.driver     = {
		.name  = "touchpad",
//		.owner = THIS_MODULE,
	}
};
//add by sen.luo for chipinfo 2013.09.24 end

static struct i2c_board_info __initdata ekft2k_i2c_tpd={ I2C_BOARD_INFO("ekft2k", (0x15))};
/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    pr_tp(TPD_DEVICE "[elan]: Driver Verison mtk0003 for MTK65xx serial\n");
    pr_tp(TPD_DEVICE "MediaTek ekft2k touch panel driver init\n");
	   i2c_register_board_info(0, &ekft2k_i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        pr_tp(TPD_DEVICE "[elan]: %s driver failed\n", __func__);
    }
	
//add by sen.luo for chipinfo 2013.09.24 start	
   if(platform_driver_register(&ektf2k_tp_driver))
	{
		pr_tp("failed to register ektf2k_tp_driver\n");
		return -1;
	}
//add by sen.luo for chipinfo 2013.09.24 end

#if defined (__CTP_ESD__)  // �� �� 2012��11��29�� 17:24:51
    init_waitqueue_head(&esd_wait_queue);
#endif /* __CTP_ESD__ */

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    pr_tp(TPD_DEVICE "[elan]: %s elan touch panel driver exit\n", __func__);
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

