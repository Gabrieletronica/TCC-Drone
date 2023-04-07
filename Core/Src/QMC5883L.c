
#include "QMC5883L.h"
#include "VL53L0X.h"
/*
typedef struct vale{
	int16_t val1;
	int16_t val2;
	int16_t val3;
}valuue;

valuue valor;
*/

int QMC5883L_Read_Reg(uint8_t reg)
{
	uint8_t Buffer[1];
	HAL_I2C_Mem_Read(QMC5883L_I2C_PORT,QMC5883L_ADDRESS,reg,1,Buffer,1,20);
	return Buffer[0];
}

void QMC5883L_Write_Reg(uint8_t reg, uint8_t data)
{
	uint8_t Buffer[2]={reg,data};
	HAL_I2C_Master_Transmit(QMC5883L_I2C_PORT,QMC5883L_ADDRESS,Buffer,2,20);
}


void QMC5883L_Read_Data(struct QMC5883L *dev, int16_t *MagX,int16_t *MagY,int16_t *MagZ) // (-32768 / +32768)
{
	/*QMC5883L_readReg48Bit(&valor, dev, 0x00);
	*MagX = valor.val1;
	*MagY = valor.val2;
	*MagZ = valor.val3;
	VL53L0X_writeReg(dev,QMC5883L_CONFIG_2,0X40);*/
	*MagX=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_X_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_X_MSB))<<8));
	*MagY=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Y_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Y_MSB))<<8));
	*MagZ=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Z_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Z_MSB))<<8));
}


int16_t QMC5883L_Read_Temperature()
{
	return (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_LSB)) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_MSB))<<8))/100;
}

// Read a 48-bit register
/*/void QMC5883L_readReg48Bit(valuue  *valor, struct VL53L0X* dev, uint8_t reg)
{
  uint8_t buf[6];//={3,3,3,3,3,3};
  int8_t xxx;
  xxx = 8;
  i2c_write(dev->address, &reg, 1);
  dev->last_status = i2c_read(dev->address, buf, 8);
  valor->val1 = (int16_t) ( buf[0] | buf[1] << 8);
  valor->val2 = (int16_t) ( buf[2] | buf[3] << 8);
  valor->val3 = (int16_t) ( buf[4] | buf[5] << 8);
}*/

void QMC5883L_Initialize(struct QMC5883L *dev, _qmc5883l_MODE MODE,_qmc5883l_ODR ODR,_qmc5883l_RNG RNG,_qmc5883l_OSR OSR)
{
	VL53L0X_writeReg(dev,QMC5883L_CONFIG_3,0X01);
	VL53L0X_writeReg(dev,QMC5883L_CONFIG_1,MODE | ODR | RNG | OSR);
	VL53L0X_writeReg(dev,QMC5883L_CONFIG_2,0X40);

}

void QMC5883L_Reset()
{
	QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x81);
}

void QMC5883L_InterruptConfig(_qmc5883l_INT INT)
{
	if(INT==INTERRUPT_ENABLE){QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x00);}
	else {QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x01);}
}


_qmc5883l_status QMC5883L_DataIsReady()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0x00)	  {return NO_NEW_DATA;}
	else if((Buffer&0x01)==0X01){return NEW_DATA_IS_READY;}
	return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsSkipped()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x04)==0X04){return DATA_SKIPPED_FOR_READING;}
		return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsOverflow()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x02)==0X02){return DATA_OVERFLOW;}
		return NORMAL;
}


void QMC5883L_ResetCalibration()
{
	Xmin=Xmax=Ymin=Ymax=0;
}


float QMC5883L_Heading(int16_t Xraw,int16_t Yraw,int16_t Zraw)
{
   	float X=Xraw,Y=Yraw,Z=Zraw;
   	float Heading;

  	if(X<Xmin) {Xmin = X;}
    	  else if(X>Xmax) {Xmax = X;}

  	if(Y<Ymin) {Ymin = Y;}
    	  else if(Y>Ymax) {Ymax = Y;}


  	if( Xmin==Xmax || Ymin==Ymax ) {return 0.0;}


 	  X -= (Xmax+Xmin)/2;
  	Y -= (Ymax+Ymin)/2;

  	X = X/(Xmax-Xmin);
  	Y = Y/(Ymax-Ymin);

  	Heading = atan2(Y,X);
		//EAST
	Heading += QMC5883L_DECLINATION_ANGLE;
	//WEST
	//Heading -= QMC5883L_DECLINATION_ANGLE;

	if(Heading <0)
   	  {Heading += 2*M_PI;}
	else if(Heading > 2*M_PI)
   	  {Heading -= 2*M_PI;}

 return Heading;
}


void QMC5883L_Scale(int16_t *X,int16_t *Y,int16_t *Z)
{
	*X*=QMC5883L_SCALE_FACTOR;
	*Y*=QMC5883L_SCALE_FACTOR;
	*Z*=QMC5883L_SCALE_FACTOR;
}

uint16_t  QMC5883L_Azimuth(struct QMC5883L *dev){
	int16_t MagX,MagY,MagZ;
	MagX=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_X_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_X_MSB))<<8));
	MagY=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Y_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Y_MSB))<<8));
	MagZ=((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Z_LSB) | (((int16_t)VL53L0X_readReg(dev, QMC5883L_DATA_READ_Z_MSB))<<8));
	int a = atan2( MagY, MagX ) * 180.0 / M_PI;
	return a < 0 ? 360 + a : a;
}




