#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

#define DRIVER_NAME "bmp280"
#define DRIVER_CLASS "bmp280Class"

static struct i2c_adapter * bmp_i2c_adapter = NULL;
static struct i2c_client * bmp280_i2c_client = NULL;

/* Meta Information */
MODULE_AUTHOR("Johannes 4Linux");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A driver for reading out a BMP280 temperature sensor");

/* Defines for device identification */ 
#define I2C_BUS_AVAILABLE	2		/* The I2C Bus available on the raspberry */
#define SLAVE_DEVICE_NAME	"BMP280"	/* Device and Driver Name */
#define BMP280_SLAVE_ADDRESS	0x3c		/* BMP280 I2C address */

static const struct i2c_device_id bmp_id[] = {
		{ SLAVE_DEVICE_NAME, 0 }, 
		{ }
};

static struct i2c_driver bmp_driver = {
	.driver = {
		.name = SLAVE_DEVICE_NAME,
		.owner = THIS_MODULE
	}
};

static struct i2c_board_info bmp_i2c_board_info = {
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME, BMP280_SLAVE_ADDRESS)
};


/*
* This light-weight library provides functionality to render numeric digits and symbols 
* on the SSD1306 128x64 OLED Display module.
* It makes use of the "mapGen" utility (part of this repository) to create 16x16 bitmaps 
* for digits and symbols.
* Using the magGen utility, it can be extended to include alphabets and other ASCII/UTF 
* characters and render text on the display.
* To communicate with the SSD1306 device, it uses "i2csend" utility (part of this repsitory)
* to send commands and data (bitmaps).
*/

#include<linux/kernel.h>
#include<linux/delay.h>

/*
* This light-weight library provides functionality to render numeric digits and symbols 
* on the SSD1306 128x64 OLED Display module.
* It makes use of the "mapGen" utility (part of this repository) to create 16x16 bitmaps 
* for digits and symbols.
* Using the magGen utility, it can be extended to include alphabets and other ASCII/UTF 
* characters and render text on the display.
* To communicate with the SSD1306 device, it uses "i2csend" utility (part of this repsitory)
* to send commands and data (bitmaps).
*/

#ifndef DIGIT_RENDERER_H
#define DIGIT_RENDERER_H

typedef enum{
    DEGREE_CELSIUS,
    DEGREE_FAHRENHEIT,
    MINUS,
    NULLSYM,
    KELVIN      //Enumeration for the newly added character 'K'
} symbol; 

/*16x16 pixels font*/
#define FONT_WIDTH              0x0f   //16 columns
#define FONT_HEIGHT             0x01   //2 pages

#define NULL_BYTE               0x00

void renderDigit(int digit,const int x,const int y);
void renderSymbol(symbol sym,const int x,const int y);
void renderTest(void);
void initDisplay(void);
#endif

/*
* This light-weight library provides functionality to render numeric digits and symbols 
* on the SSD1306 128x64 OLED Display module.
* It makes use of the "mapGen" utility (part of this repository) to create 16x16 bitmaps 
* for digits and symbols.
* Using the magGen utility, it can be extended to include alphabets and other ASCII/UTF 
* characters and render text on the display.
* To communicate with the SSD1306 device, it uses "i2csend" utility (part of this repsitory)
* to send commands and data (bitmaps).
*/

#ifndef DIGIT_RENDERER_H
#define DIGIT_RENDERER_H

typedef enum{
    DEGREE_CELSIUS,
    DEGREE_FAHRENHEIT,
    MINUS,
    NULLSYM,
    KELVIN      //Enumeration for the newly added character 'K'
} symbol; 

/*16x16 pixels font*/
#define FONT_WIDTH              0x0f   //16 columns
#define FONT_HEIGHT             0x01   //2 pages

#define NULL_BYTE               0x00

void renderDigit(int digit,const int x,const int y);
void renderSymbol(symbol sym,const int x,const int y);
void renderTest(void);
void initDisplay(void);
#endif

/*
* This utility wraps the linux i2c subsystem API to send commands and data to 
* the SSD1306 128x64 OLED Display module.
*/
#ifndef I2CSEND_H
#define I2CSEND_H



/*SSD1306 Commands used in this implementation. For entire list of commands refer to the documentation for SSD1306.*/
#define SSD1306_SET_CONTRAST                0x81
#define SSD1306_SET_DISPLAY_ON_RAM          0xa4
#define SSD1306_SET_DISPLAY_NORMAL          0xa6
#define SSD1306_DISPLAY_OFF                 0xae
#define SSD1306_DISPLAY_ON                  0xaf
#define SSD1306_SET_MEM_ADDR_MODE           0x20
#define SSD1306_SET_COLUMN                  0x21
#define SSD1306_SET_PAGE                    0x22
#define SSD1306_SET_SEGMENT_REMAP127        0xa1
#define SSD1306_SET_CLK_DR_OSC_FRQ          0xd5
#define SSD1306_SET_MUX_RATIO               0xa8
#define SSD1306_SET_DISPLAY_OFFSET          0xd3
#define SSD1306_SET_DISPLAY_START_FIRST     0x40
#define SSD1306_SET_CHARGE_PUMP             0x8d
#define SSD1306_SET_COM_OUT_DESCENDING      0xc8
#define SSD1306_SET_COM_CONFIG              0xda
#define SSD1306_SET_PRE_CHARGE_PERIOD       0xd9
#define SSD1306_SET_VCOMH_DESELECT_LEVEL    0xdb
#define SSD1306_SET_SCROLL_OFF              0x2e

/*Initialization values*/
#define CONTRAST                0xff
#define ADDR_MODE_HZ            0x00
#define CLK_DR_OSC_FRQ          0x80
#define MUX64                   0x3f
#define OFFSET_VALUE            0x00
#define ENABLE_PUMP             0x14
#define COM_CONFIG              0x12
#define PERIOD                  0x22
#define VCOMH_DESELECT_LEVEL    0x20

#define DISPLAY_BEGIN_COL       0x00
#define DISPLAY_END_COL         0x7f
#define DISPLAY_BEGIN_PAGE      0x00
#define DISPLAY_END_PAGE        0x07


#endif

#define NUM_DIGITS 10
#define NUM_SYMBOLS 5

/*Generated using mapGen utility*/
unsigned char digitMap[NUM_DIGITS][32]={
    {0x0, 0x0, 0x0, 0x0, 0xf8, 0xfc, 0x1e, 0xe, 0xe, 0x1e, 0xfc, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x3f, 0x78, 0x70, 0x70, 0x78, 0x3f, 0x1f, 0x0, 0x0, 0x0, 0x0}, //0
    {0x0, 0x0, 0x0, 0x0, 0x0, 0x10, 0x18, 0xfc, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x70, 0x70, 0x7f, 0x7f, 0x70, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0},     //1
    {0x0, 0x0, 0x0, 0x0, 0x1c, 0x1e, 0xe, 0xe, 0x8e, 0xce, 0xfe, 0x7c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x78, 0x7c, 0x7e, 0x77, 0x73, 0x71, 0x70, 0x70, 0x0, 0x0, 0x0, 0x0}, //2
    {0x0, 0x0, 0x0, 0x0, 0x1c, 0x1e, 0xe, 0x8e, 0x8e, 0xce, 0xfe, 0x7c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x38, 0x78, 0x70, 0x71, 0x71, 0x73, 0x7f, 0x3e, 0x0, 0x0, 0x0, 0x0},//3
    {0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0x70, 0x38, 0xfc, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xe, 0xe, 0x4e, 0x7f, 0x7f, 0x4e, 0x0, 0x0, 0x0, 0x0},    //4
    {0x0, 0x0, 0x0, 0x0, 0xfe, 0xfe, 0xfe, 0xce, 0xce, 0x8e, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x71, 0x71, 0x71, 0x71, 0x71, 0x7f, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0},  //5
    {0x0, 0x0, 0x0, 0x0, 0xfc, 0xfe, 0x86, 0x86, 0x86, 0x9e, 0x1c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3f, 0x7f, 0x73, 0x61, 0x73, 0x7f, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0}, //6
    {0x0, 0x0, 0x0, 0x0, 0xe, 0xe, 0xe, 0xe, 0x8e, 0xfe, 0x7e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x78, 0x7e, 0xf, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},          //7
    {0x0, 0x0, 0x0, 0x0, 0x7c, 0xfe, 0xce, 0x86, 0xce, 0xfe, 0x7c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3e, 0x7f, 0x73, 0x61, 0x73, 0x7f, 0x3e, 0x0, 0x0, 0x0, 0x0, 0x0}, //8
    {0x0, 0x0, 0x0, 0x0, 0x7c, 0xfe, 0xce, 0x86, 0xce, 0xfe, 0xfc, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x38, 0x79, 0x61, 0x61, 0x61, 0x7f, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0}  //9 
};


/*Generated using mapGen utility*/
unsigned char symbolMap[NUM_SYMBOLS][32]={
    {0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0x0, 0xf8, 0xfc, 0xfe, 0xe, 0xe, 0x1e, 0x3c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1f, 0x3f, 0x7f, 0x70, 0x70, 0x78, 0x3c, 0x0, 0x0}, //Degree Celsius
    {0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0x0, 0xfe, 0xfe, 0xfe, 0xe, 0xe, 0x1e, 0x3e, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x7f, 0x7f, 0x7f, 0x43, 0x3, 0x7, 0x0, 0x0, 0x0},   //Degree Fahrenheit
    {0x0, 0x0, 0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0},       //Minus
    {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},               //NULL Symbol
    {0x0, 0x0, 0x0, 0x0, 0xfe, 0xfe, 0xfe, 0x78, 0x3c, 0x1e, 0xe, 0x6, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7f, 0x7f, 0x7f, 0x1e, 0x3c, 0x78, 0x70, 0x60, 0x40, 0x0, 0x0, 0x0} //Kelvin
};



//Send command to SSD1306
int sendCommand(unsigned char command){
    unsigned char buf[2];
    buf[0]=0x00;
    buf[1]=command;
    return i2c_master_send(bmp280_i2c_client,buf,2);
}

//Send a byte of data to SSD1306
int sendDataByte(unsigned char byte){
    unsigned char buf[2];
    buf[0]=0x40;
    buf[1]=byte;
    return i2c_master_send(bmp280_i2c_client,buf,2);
}

//Send a block of data to SSD1306
//Memory pointed by the block argument is to be freed by the callee.
int sendDataBlock(unsigned char *block,const unsigned int size){
    unsigned char *buf =kmalloc(size*(sizeof(unsigned char))+1,GFP_KERNEL);
    int ret;
    buf[0]=0x40;
    memcpy(buf+1,block,size);
    ret=i2c_master_send(bmp280_i2c_client,buf,size+1);
    kfree(buf);
    return ret;
}


/*Render a single digit on display at specified co-ordinates*/
void renderDigit(int digit,const int x,const int y){
    sendCommand(SSD1306_SET_COLUMN);
    sendCommand(y);                     //Column Start
    sendCommand(y+FONT_WIDTH);          //Column End
    sendCommand(SSD1306_SET_PAGE); 
    sendCommand(x);                     //Page Start
    sendCommand(x+FONT_HEIGHT);         //Page End

    sendDataBlock(digitMap[digit],32);
}
/*Render a single symbol on display at specified co-ordinates*/
void renderSymbol(symbol sym,const int x,const int y){
    sendCommand(SSD1306_SET_COLUMN);
    sendCommand(y);                     //Column Start
    sendCommand(y+FONT_WIDTH);          //Column End
    sendCommand(SSD1306_SET_PAGE); 
    sendCommand(x);                     //Page Start
    sendCommand(x+FONT_HEIGHT);         //Page End
    
    sendDataBlock(symbolMap[sym],32);
}

/*Display digits and symbols to test*/
void renderTest(void){
    renderDigit(0,0,8);
    renderDigit(1,0,40);
    renderDigit(2,0,72);
    renderDigit(3,0,104);
    renderDigit(4,2,8);
    renderDigit(5,2,40);
    renderDigit(6,2,72);
    renderDigit(7,2,104);
    renderDigit(8,4,8);
    renderDigit(9,4,40);
    renderSymbol(DEGREE_CELSIUS,4,72);
    renderSymbol(DEGREE_FAHRENHEIT,4,104);
}

/*Initializing sequence for SSD1306 display. More details about initialization sequence can be found in the SSD1306 Documentation.*/
void initDisplay(void){
    msleep(100);
    sendCommand(SSD1306_DISPLAY_OFF); 
    sendCommand(SSD1306_SET_CLK_DR_OSC_FRQ);
    sendCommand(CLK_DR_OSC_FRQ); 
    sendCommand(SSD1306_SET_MUX_RATIO); 
    sendCommand(MUX64); 
    sendCommand(SSD1306_SET_DISPLAY_OFFSET); 
    sendCommand(OFFSET_VALUE); 
    sendCommand(SSD1306_SET_DISPLAY_START_FIRST);
    sendCommand(SSD1306_SET_CHARGE_PUMP); 
    sendCommand(ENABLE_PUMP);
    sendCommand(SSD1306_SET_MEM_ADDR_MODE);
    sendCommand(ADDR_MODE_HZ); 
    sendCommand(SSD1306_SET_SEGMENT_REMAP127); 
    sendCommand(SSD1306_SET_COM_OUT_DESCENDING); 
    sendCommand(SSD1306_SET_COM_CONFIG); 
    sendCommand(COM_CONFIG); 
    sendCommand(SSD1306_SET_CONTRAST); 
    sendCommand(CONTRAST); 
    sendCommand(SSD1306_SET_PRE_CHARGE_PERIOD); 
    sendCommand(PERIOD); 
    sendCommand(SSD1306_SET_VCOMH_DESELECT_LEVEL); 
    sendCommand(VCOMH_DESELECT_LEVEL); 
    sendCommand(SSD1306_SET_DISPLAY_ON_RAM); 
    sendCommand(SSD1306_SET_DISPLAY_NORMAL); 
    sendCommand(SSD1306_SET_SCROLL_OFF); 
    sendCommand(SSD1306_DISPLAY_ON);

    printk(KERN_INFO"SSD1306 display ready!");

    /*for fast clear sendDataBlock can be used. Try it!*/
    int i;
    for(i=0;i<128*8;i++){ //Clear any garbage pixels
        sendDataByte(0x00);
    }
}


/* Variables for Device and Deviceclass*/
static dev_t myDeviceNr;
static struct class *myClass;
static struct cdev myDevice;

/* Variables for temperature calculation */
s32 dig_T1, dig_T2, dig_T3;

/**
 * @brief Read current temperature from BMP280 sensor
 *
 * @return temperature in degree
 */
s32 read_temperature(void) {
	int var1, var2;
	s32 raw_temp;
	s32 d1, d2, d3;

	/* Read Temperature */
	d1 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFA);
	d2 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFB);
	d3 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFC);
	raw_temp = ((d1<<16) | (d2<<8) | d3) >> 4;

	/* Calculate temperature in degree */
	var1 = ((((raw_temp >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11;

	var2 = (((((raw_temp >> 4) - (dig_T1)) * ((raw_temp >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
	return ((var1 + var2) *5 +128) >> 8;
}

/**
 * @brief Get data out of buffer
 */
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char out_string[20];
	int temperature;

	/* Get amount of bytes to copy */
	to_copy = min(sizeof(out_string), count);

	/* Get temperature */
	temperature = read_temperature();
	snprintf(out_string, sizeof(out_string), "%d.%d\n", temperature/100, temperature%100);

	/* Copy Data to user */
	not_copied = copy_to_user(user_buffer, out_string, to_copy);

	/* Calculate delta */
	delta = to_copy - not_copied;

	return delta;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_open(struct inode *deviceFile, struct file *instance) {
	printk("MyDeviceDriver -  Open was called\n");
	return 0;
}

/**
 * @brief This function is called, when the device file is close
 */
static int driver_close(struct inode *deviceFile, struct file *instance) {
	printk("MyDeviceDriver -  Close was called\n");
	return 0;
}

/* Map the file operations */
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.read = driver_read,
};


/*SSD1306 Commands used in this implementation. For entire list of commands refer to the documentation for SSD1306.*/
#define SSD1306_SET_CONTRAST                0x81
#define SSD1306_SET_DISPLAY_ON_RAM          0xa4
#define SSD1306_SET_DISPLAY_NORMAL          0xa6
#define SSD1306_DISPLAY_OFF                 0xae
#define SSD1306_DISPLAY_ON                  0xaf
#define SSD1306_SET_MEM_ADDR_MODE           0x20
#define SSD1306_SET_COLUMN                  0x21
#define SSD1306_SET_PAGE                    0x22
#define SSD1306_SET_SEGMENT_REMAP127        0xa1
#define SSD1306_SET_CLK_DR_OSC_FRQ          0xd5
#define SSD1306_SET_MUX_RATIO               0xa8
#define SSD1306_SET_DISPLAY_OFFSET          0xd3
#define SSD1306_SET_DISPLAY_START_FIRST     0x40
#define SSD1306_SET_CHARGE_PUMP             0x8d
#define SSD1306_SET_COM_OUT_DESCENDING      0xc8
#define SSD1306_SET_COM_CONFIG              0xda
#define SSD1306_SET_PRE_CHARGE_PERIOD       0xd9
#define SSD1306_SET_VCOMH_DESELECT_LEVEL    0xdb
#define SSD1306_SET_SCROLL_OFF              0x2e


/*Initialization values*/
#define CONTRAST                0xff
#define ADDR_MODE_HZ            0x00
#define CLK_DR_OSC_FRQ          0x80
#define MUX64                   0x3f
#define OFFSET_VALUE            0x00
#define ENABLE_PUMP             0x14
#define COM_CONFIG              0x12
#define PERIOD                  0x22
#define VCOMH_DESELECT_LEVEL    0x20

#define DISPLAY_BEGIN_COL       0x00
#define DISPLAY_END_COL         0x7f
#define DISPLAY_BEGIN_PAGE      0x00
#define DISPLAY_END_PAGE        0x07

static void clearDisplay(void){
    sendCommand(SSD1306_SET_COLUMN); 
    sendCommand(DISPLAY_BEGIN_COL);     //First Column
    sendCommand(DISPLAY_END_COL);       //Last Column
    sendCommand(SSD1306_SET_PAGE); 
    sendCommand(DISPLAY_BEGIN_PAGE);    //Set Page start and end addresses
    sendCommand(DISPLAY_END_PAGE);      //Set Page start and end addresses

    /*for fast clear sendDataBlock can be used. Try it!*/
	int i=0;
    for(i=0;i<128*8;i++){
        sendDataByte(0x00);
    }
}

/**
 * @brief function, which is called after loading module to kernel, do initialization there
 */
static int __init ModuleInit(void) {
	int ret = -1;
	u8 id;
	printk("MyDeviceDriver - Hello Kernel\n");

	/* Allocate Device Nr */
	if ( alloc_chrdev_region(&myDeviceNr, 0, 1, DRIVER_NAME) < 0) {
		printk("Device Nr. could not be allocated!\n");
	}
	printk("MyDeviceDriver - Device Nr %d was registered\n", myDeviceNr);

	/* Create Device Class */
	if ((myClass = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device Class can not be created!\n");
		goto ClassError;
	}

	/* Create Device file */
	if (device_create(myClass, NULL, myDeviceNr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize Device file */
	cdev_init(&myDevice, &fops);

	/* register device to kernel */
	if (cdev_add(&myDevice, myDeviceNr, 1) == -1) {
		printk("Registering of device to kernel failed!\n");
		goto KernelError;
	}

	bmp_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

	if(bmp_i2c_adapter != NULL) {
		bmp280_i2c_client = i2c_new_client_device(bmp_i2c_adapter, &bmp_i2c_board_info);
		if(bmp280_i2c_client != NULL) {
			if(i2c_add_driver(&bmp_driver) != -1) {
				ret = 0;
			}
			else
				printk("Can't add driver...\n");
		}
		i2c_put_adapter(bmp_i2c_adapter);
	}
	else{
		printk("bmp_i2c_adapter is null!\n");
		goto KernelError;
	}
	printk("BMP280 Driver added!\n");	 
		/* Read Chip ID */
	id = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xD0);
	printk("ID: 0x%x\n", id);
	#define SSD1306_WRITECOMMAND(command)      sendCommand(command)


	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel

	clearDisplay();
	renderTest();


	return ret;
KernelError:
	device_destroy(myClass, myDeviceNr);
FileError:
	class_destroy(myClass);
ClassError:
	unregister_chrdev(myDeviceNr, DRIVER_NAME);
	return (-1);
}

/**
 * @brief function, which is called when removing module from kernel
 * free alocated resources
 */
static void __exit ModuleExit(void) {
	printk("MyDeviceDriver - Goodbye, Kernel!\n");
	i2c_unregister_device(bmp280_i2c_client);
	i2c_del_driver(&bmp_driver);
	cdev_del(&myDevice);
    device_destroy(myClass, myDeviceNr);
    class_destroy(myClass);
    unregister_chrdev_region(myDeviceNr, 1);
}

module_init(ModuleInit);
module_exit(ModuleExit);