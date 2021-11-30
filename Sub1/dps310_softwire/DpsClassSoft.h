/**
 * Arduino library to control Dps310
 *
 * "Dps310" represents Infineon's high-sensetive pressure and temperature sensor. 
 * It measures in ranges of 300 - 1200 hPa and -40 and 85 °C. 
 * The sensor can be connected via SPI or I2C. 
 * It is able to perform single measurements
 * or to perform continuous measurements of temperature and pressure at the same time, 
 * and stores the results in a FIFO to reduce bus communication. 
 *
 * Have a look at the datasheet for more information. 
 */

#ifndef DPSCLASSSOFT_H_INCLUDED
#define DPSCLASSSOFT_H_INCLUDED

// #include <Wire.h>
#include <SlowSoftWire.h>
#include "util/dps_config.h"
#include <Arduino.h>

class DpsClassSoft
{
  public:
	//constructor
	DpsClassSoft(void);
	//destructor
	~DpsClassSoft(void);

	/**
	 * I2C begin function with standard address
	 */
	void begin(SlowSoftWire &bus);

	/**
	 * Standard I2C begin function
	 *
	 * @param &bus: 			I2CBus which connects MC to the sensor
	 * @param slaveAddress: 	I2C address of the sensor (0x77 or 0x76)
	 */
	void begin(SlowSoftWire &bus, uint8_t slaveAddress);

	/**
	 * End function for Dps310
	 * Sets the sensor to idle mode
	 */
	void end(void);

	/**
	 * returns the Product ID of the connected Dps310 sensor
	 */
	uint8_t getProductId(void);

	/**
	 * returns the Revision ID of the connected Dps310 sensor
	 */
	uint8_t getRevisionId(void);

	/**
	 * Sets the Dps310 to standby mode
	 *
	 * @return		status code
	 */
	int16_t standby(void);

	/**
	 * performs one temperature measurement
	 *
	 * @param &result:		reference to a float value where the result will be written
	 * @return 	status code
	 */
	int16_t measureTempOnce(float &result);

	/**
	 * performs one temperature measurement with specified oversamplingRate
	 *
	 * @param &result:				reference to a float where the result will be written
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128, which are defined as integers 0 - 7
	 * 						The number of measurements equals to 2^n, if the value written to the register field is n. 2^n internal measurements are combined to return a more exact measurement
	 * @return 			status code
	 */
	int16_t measureTempOnce(float &result, uint8_t oversamplingRate);

	/**
	 * starts a single temperature measurement
	 *
	 * @return 	status code
	 */
	int16_t startMeasureTempOnce(void);

	/**
	 * starts a single temperature measurement with specified oversamplingRate
	 *
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128, which are defined as integers 0 - 7
	 * @return 			status code
	 */
	int16_t startMeasureTempOnce(uint8_t oversamplingRate);

	/**
	 * performs one pressure measurement
	 *
	 * @param &result:		reference to a float value where the result will be written
	 * @return 	status code
	 */
	int16_t measurePressureOnce(float &result);

	/**
	 * performs one pressure measurement with specified oversamplingRate
	 *
	 * @param &result:				reference to a float where the result will be written
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * @return 			status code
	 */
	int16_t measurePressureOnce(float &result, uint8_t oversamplingRate);

	/**
	 * starts a single pressure measurement
	 *
	 * @return 	status code
	 */
	int16_t startMeasurePressureOnce(void);

	/**
	 * starts a single pressure measurement with specified oversamplingRate
	 *
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * @return 			status code
	 */
	int16_t startMeasurePressureOnce(uint8_t oversamplingRate);

	/**
	 * gets the result a single temperature or pressure measurement in °C or Pa
	 *
	 * @param &result:		reference to a float value where the result will be written
	 * @return 	status code
	 */
	int16_t getSingleResult(float &result);

	/**
	 * starts a continuous temperature measurement with specified measurement rate and oversampling rate
	 * If measure rate is n and oversampling rate is m, the DPS310 performs 2^(n+m) internal measurements per second. 
	 * The DPS310 cannot operate with high precision and high speed at the same time. Consult the datasheet for more information.
	 * 
	 * @param measureRate: 		DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * 						
	 * @return 			status code
	 * 						
	 */
	int16_t startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);

	/**
	 * starts a continuous temperature measurement with specified measurement rate and oversampling rate
	 *
	 * @param measureRate: 		DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
	 * @param oversamplingRate: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * @return 			status code
	 */
	int16_t startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);

	/**
	 * starts a continuous temperature and pressure measurement with specified measurement rate and oversampling rate for temperature and pressure measurement respectvely.
	 *
	 * @param tempMr				measure rate for temperature
	 * @param tempOsr				oversampling rate for temperature
	 * @param prsMr				measure rate for pressure
	 * @param prsOsr				oversampling rate for pressure
	 * @return 			status code
	 */
	int16_t startMeasureBothCont(uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);

	/**
	 * Gets the interrupt status flag of the FIFO
	 *
	 * @return 	1 if the FIFO is full and caused an interrupt
	 * 				0 if the FIFO is not full or FIFO interrupt is disabled
	 * 				-1 on fail
	 */
	int16_t getIntStatusFifoFull(void);

	/**
	 * Gets the interrupt status flag that indicates a finished temperature measurement
	 *
	 * @return 	1 if a finished temperature measurement caused an interrupt;
	 * 				0 if there is no finished temperature measurement or interrupts are disabled;
	 * 				-1 on fail.
	 */
	int16_t getIntStatusTempReady(void);

	/**
	 * Gets the interrupt status flag that indicates a finished pressure measurement
	 *
	 * @return 	1 if a finished pressure measurement caused an interrupt; 
	 * 				0 if there is no finished pressure measurement or interrupts are disabled;
	 * 				-1 on fail.
	 */
	int16_t getIntStatusPrsReady(void);

	/**
	 * Function to fix a hardware problem on some devices
	 * You have this problem if you measure a temperature which is too high (e.g. 60°C when temperature is around 20°C)
	 * Call correctTemp() directly after begin() to fix this issue
	 */
	int16_t correctTemp(void);

  protected:
	//scaling factor table
	static const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS];

	dpssoft::Mode m_opMode;

	//flags
	uint8_t m_initFail;

	uint8_t m_productID;
	uint8_t m_revisionID;

	//settings
	uint8_t m_tempMr;
	uint8_t m_tempOsr;
	uint8_t m_prsMr;
	uint8_t m_prsOsr;

	// compensation coefficients for both dps310 and dps422
	int32_t m_c00;
	int32_t m_c10;
	int32_t m_c01;
	int32_t m_c11;
	int32_t m_c20;
	int32_t m_c21;
	int32_t m_c30;

	// last measured scaled temperature (necessary for pressure compensation)
	float m_lastTempScal;

	//bus specific
	uint8_t m_SpiI2c; //0=SPI, 1=I2C

	//used for I2C
	SlowSoftWire *m_i2cbus;
	uint8_t m_slaveAddress;

	/**
	 * Initializes the sensor.
	 * This function has to be called from begin()
	 * and requires a valid bus initialization.
	 */
	virtual void init(void) = 0;

	/**
	 * reads the compensation coefficients from the sensor
	 * this is called once from init(), which is called from begin()
	 *
	 * @return 	0 on success, -1 on fail
	 */
	virtual int16_t readcoeffs(void) = 0;

	/**
	 * Sets the Operation Mode of the sensor
	 * 
	 * @param opMode: 			the new OpMode as defined by dps::Mode; CMD_BOTH should not be used for DPS310
	 * @return 			0 on success, -1 on fail
	 */
	int16_t setOpMode(uint8_t opMode);

	/**
	 * Configures temperature measurement
	 *
	 * @param temp_mr: 		DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
	 * @param temp_osr: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * 				
	 * @return 	0 normally or -1 on fail
	 */
	virtual int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);

	/**
	 * Configures pressure measurement
	 *
	 * @param prs_mr: 		DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
	 * @param prs_osr: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * @return 	0 normally or -1 on fail
	 */
	virtual int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);

	virtual int16_t flushFIFO() = 0;

	virtual float calcTemp(int32_t raw) = 0;

	virtual float calcPressure(int32_t raw) = 0;

	int16_t enableFIFO();

	int16_t disableFIFO();

	/**
	 * calculates the time that the sensor needs for 2^mr measurements with an oversampling rate of 2^osr (see table "pressure measurement time (ms) versus oversampling rate")
	 * Note that the total measurement time for temperature and pressure must not be more than 1 second.
	 * Timing behavior of pressure and temperature sensors can be considered the same.
	 * 
	 * @param mr: 		DPS__MEASUREMENT_RATE_1, DPS__MEASUREMENT_RATE_2,DPS__MEASUREMENT_RATE_4 ... DPS__MEASUREMENT_RATE_128
	 * @param osr: 	DPS__OVERSAMPLING_RATE_1, DPS__OVERSAMPLING_RATE_2, DPS__OVERSAMPLING_RATE_4 ... DPS__OVERSAMPLING_RATE_128
	 * @return time that the sensor needs for this measurement
	 */
	uint16_t calcBusyTime(uint16_t temp_rate, uint16_t temp_osr);

	/**
	 * reads the next raw value from the FIFO
	 *
	 * @param value: 	the raw pressure or temperature value read from the pressure register blocks, where the LSB of PRS_B0 marks wheather the value is a temperatur or a pressure.
	 * 
	 * @return	-1 on fail
	 * 			0 if result is a temperature raw value
	 * 			1 if result is a pressure raw value
	 */
	int16_t getFIFOvalue(int32_t *value);

	/**
	 * Gets the results from continuous measurements and writes them to given arrays
	 *
	 * @param *tempBuffer: 	The start address of the buffer where the temperature results are written
	 * 					If this is NULL, no temperature results will be written out
	 * @param &tempCount:		The size of the buffer for temperature results.
	 * 					When the function ends, it will contain the number of bytes written to the buffer.
	 * @param *prsBuffer: 		The start address of the buffer where the pressure results are written
	 * 					If this is NULL, no pressure results will be written out
	 * @param &prsCount:		The size of the buffer for pressure results.
	 * 					When the function ends, it will contain the number of bytes written to the buffer.
	 * @param reg The FIFO empty register field; needed since this field is different for each sensor
	 * @return			status code
	 */
	int16_t getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount, RegMask_t reg);

	/**
	 * reads a byte from the sensor
	 *
	 * @param regAdress: 	Address that has to be read
	 * @return 	register content or -1 on fail
	 */
	int16_t readByte(uint8_t regAddress);

	/**
	 * reads a block from the sensor
	 *
	 * @param regAdress: 	Address that has to be read
	 * @param length: 		Length of data block
	 * @param buffer: 	Buffer where data will be stored
	 * @return 	number of bytes that have been read successfully, which might not always equal to length due to rx-Buffer overflow etc.
	 */
	int16_t readBlock(RegBlock_t regBlock, uint8_t *buffer);

	/**
	 * writes a byte to a given register of the sensor without checking
	 *
	 * @param regAdress: 	Address of the register that has to be updated
	 * @param data:		Byte that will be written to the register
	 * @return		0 if byte was written successfully
	 * 				or -1 on fail
	 */
	int16_t writeByte(uint8_t regAddress, uint8_t data);

	/**
	 * writes a byte to a register of the sensor
	 *
	 * @param regAdress: 	Address of the register that has to be updated
	 * @param data:		Byte that will be written to the register
	 * @param check: 		If this is true, register content will be read after writing
	 * 				to check if update was successful
	 * @return		0 if byte was written successfully
	 * 				or -1 on fail
	 */
	int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check);


	/**
	 * updates a bit field of the sensor without checking
	 *
	 * @param regMask: 	Mask of the register that has to be updated
	 * @param data:		BitValues that will be written to the register
	 * @return		0 if byte was written successfully
	 * 				or -1 on fail
	 */
	int16_t writeByteBitfield(uint8_t data, RegMask_t regMask);

	/**
	 * updates a bit field of the sensor
	 *
	 * regMask: 	Mask of the register that has to be updated
	 * data:		BitValues that will be written to the register
	 * check: 		enables/disables check after writing; 0 disables check.
	 * 				if check fails, -1 will be returned
	 * @return		0 if byte was written successfully
	 * 				or -1 on fail
	 */
	int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, uint8_t check);

	/**
	 * reads a bit field from the sensor
	 * regMask: 	Mask of the register that has to be updated
	 * data:		BitValues that will be written to the register
	 * @return		read and processed bits
	 * 				or -1 on fail
	 */
	int16_t readByteBitfield(RegMask_t regMask);

	/**
	 * @brief converts non-32-bit negative numbers to 32-bit negative numbers with 2's complement
	 * 
	 * @param raw The raw number of less than 32 bits
	 * @param length The bit length
	 */
	void getTwosComplement(int32_t *raw, uint8_t length);

	/**
	 * @brief Get a raw result from a given register block
	 * 
	 * @param raw The address where the raw value is to be written
	 * @param reg The register block to be read from
	 * @return status code 
	 */
	int16_t getRawResult(int32_t *raw, RegBlock_t reg);
};




using namespace dpssoft;

const int32_t DpsClassSoft::scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

//////// 		Constructor, Destructor, begin, end			////////

DpsClassSoft::DpsClassSoft(void)
{
	//assume that initialization has failed before it has been done
	m_initFail = 1U;
}

DpsClassSoft::~DpsClassSoft(void)
{
	end();
}

void DpsClassSoft::begin(SlowSoftWire &bus)
{
	begin(bus, DPS__STD_SLAVE_ADDRESS);
}

void DpsClassSoft::begin(SlowSoftWire &bus, uint8_t slaveAddress)
{
	//this flag will show if the initialization was successful
	m_initFail = 0U;

	//Set I2C bus connection
	m_SpiI2c = 1U;
	m_i2cbus = &bus;
	m_slaveAddress = slaveAddress;

	// Init bus
	m_i2cbus->begin();

	delay(50); //startup time of Dps310

	init();
}


void DpsClassSoft::end(void)
{
	standby();
}

////////		Declaration of other public functions starts here			////////

uint8_t DpsClassSoft::getProductId(void)
{
	return m_productID;
}

uint8_t DpsClassSoft::getRevisionId(void)
{
	return m_revisionID;
}

int16_t DpsClassSoft::getContResults(float *tempBuffer,
								 uint8_t &tempCount,
								 float *prsBuffer,
								 uint8_t &prsCount, RegMask_t fifo_empty_reg)
{
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in background mode
	if (!(m_opMode & 0x04))
	{
		return DPS__FAIL_TOOBUSY;
	}

	if (!tempBuffer || !prsBuffer)
	{
		return DPS__FAIL_UNKNOWN;
	}
	tempCount = 0U;
	prsCount = 0U;

	//while FIFO is not empty
	while (readByteBitfield(fifo_empty_reg) == 0)
	{
		int32_t raw_result;
		float result;
		//read next result from FIFO
		int16_t type = getFIFOvalue(&raw_result);
		switch (type)
		{
		case 0: //temperature
			if (tempCount < DPS__FIFO_SIZE)
			{
				result = calcTemp(raw_result);
				tempBuffer[tempCount++] = result;
			}
			break;
		case 1: //pressure
			if (prsCount < DPS__FIFO_SIZE)
			{
				result = calcPressure(raw_result);
				prsBuffer[prsCount++] = result;
			}
			break;
		case -1: //read failed
			break;
		}
	}
	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::getSingleResult(float &result)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}

	//read finished bit for current opMode
	int16_t rdy;
	switch (m_opMode)
	{
	case CMD_TEMP: //temperature
		rdy = readByteBitfield(config_registers[TEMP_RDY]);
		break;
	case CMD_PRS: //pressure
		rdy = readByteBitfield(config_registers[PRS_RDY]);
		break;
	default: //DPS310 not in command mode
		return DPS__FAIL_TOOBUSY;
	}
	//read new measurement result
	switch (rdy)
	{
	case DPS__FAIL_UNKNOWN: //could not read ready flag
		return DPS__FAIL_UNKNOWN;
	case 0: //ready flag not set, measurement still in progress
		return DPS__FAIL_UNFINISHED;
	case 1: //measurement ready, expected case
		Mode oldMode = m_opMode;
		m_opMode = IDLE; //opcode was automatically reseted by DPS310
		int32_t raw_val;
		switch (oldMode)
		{
		case CMD_TEMP: //temperature
			getRawResult(&raw_val, registerBlocks[TEMP]);
			result = calcTemp(raw_val);
			return DPS__SUCCEEDED; // TODO
		case CMD_PRS:			   //pressure
			getRawResult(&raw_val, registerBlocks[PRS]);
			result = calcPressure(raw_val);
			return DPS__SUCCEEDED; // TODO
		default:
			return DPS__FAIL_UNKNOWN; //should already be filtered above
		}
	}
	return DPS__FAIL_UNKNOWN;
}

int16_t DpsClassSoft::measureTempOnce(float &result)
{
	return measureTempOnce(result, m_tempOsr);
}

int16_t DpsClassSoft::measureTempOnce(float &result, uint8_t oversamplingRate)
{
	//Start measurement
	int16_t ret = startMeasureTempOnce(oversamplingRate);
	if (ret != DPS__SUCCEEDED)
	{
		return ret;
	}

	//wait until measurement is finished
	delay(calcBusyTime(0U, m_tempOsr) / DPS__BUSYTIME_SCALING);
	delay(DPS310__BUSYTIME_FAILSAFE);

	ret = getSingleResult(result);
	if (ret != DPS__SUCCEEDED)
	{
		standby();
	}
	return ret;
}

int16_t DpsClassSoft::startMeasureTempOnce(void)
{
	return startMeasureTempOnce(m_tempOsr);
}

int16_t DpsClassSoft::startMeasureTempOnce(uint8_t oversamplingRate)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if (m_opMode != IDLE)
	{
		return DPS__FAIL_TOOBUSY;
	}

	if (oversamplingRate != m_tempOsr)
	{
		//configuration of oversampling rate
		if (configTemp(0U, oversamplingRate) != DPS__SUCCEEDED)
		{
			return DPS__FAIL_UNKNOWN;
		}
	}

	//set device to temperature measuring mode
	return setOpMode(CMD_TEMP);
}

int16_t DpsClassSoft::measurePressureOnce(float &result)
{
	return measurePressureOnce(result, m_prsOsr);
}

int16_t DpsClassSoft::measurePressureOnce(float &result, uint8_t oversamplingRate)
{
	//start the measurement
	int16_t ret = startMeasurePressureOnce(oversamplingRate);
	if (ret != DPS__SUCCEEDED)
	{
		return ret;
	}

	//wait until measurement is finished
	delay(calcBusyTime(0U, m_prsOsr) / DPS__BUSYTIME_SCALING);
	delay(DPS310__BUSYTIME_FAILSAFE);

	ret = getSingleResult(result);
	if (ret != DPS__SUCCEEDED)
	{
		standby();
	}
	return ret;
}

int16_t DpsClassSoft::startMeasurePressureOnce(void)
{
	return startMeasurePressureOnce(m_prsOsr);
}

int16_t DpsClassSoft::startMeasurePressureOnce(uint8_t oversamplingRate)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if (m_opMode != IDLE)
	{
		return DPS__FAIL_TOOBUSY;
	}
	//configuration of oversampling rate, lowest measure rate to avoid conflicts
	if (oversamplingRate != m_prsOsr)
	{
		if (configPressure(0U, oversamplingRate))
		{
			return DPS__FAIL_UNKNOWN;
		}
	}
	//set device to pressure measuring mode
	return setOpMode(CMD_PRS);
}

int16_t DpsClassSoft::startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if (m_opMode != IDLE)
	{
		return DPS__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if (calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return DPS__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if (configTemp(measureRate, oversamplingRate))
	{
		return DPS__FAIL_UNKNOWN;
	}

	if (enableFIFO())
	{
		return DPS__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if (DpsClassSoft::setOpMode(CONT_TMP))
	{
		return DPS__FAIL_UNKNOWN;
	}
	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if (m_opMode != IDLE)
	{
		return DPS__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if (calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return DPS__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if (configPressure(measureRate, oversamplingRate))
		return DPS__FAIL_UNKNOWN;
	//enable result FIFO
	if (enableFIFO())
	{
		return DPS__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if (DpsClassSoft::setOpMode(CONT_PRS))
	{
		return DPS__FAIL_UNKNOWN;
	}
	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::startMeasureBothCont(uint8_t tempMr,
									   uint8_t tempOsr,
									   uint8_t prsMr,
									   uint8_t prsOsr)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if (m_opMode != IDLE)
	{
		return DPS__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if (calcBusyTime(tempMr, tempOsr) + calcBusyTime(prsMr, prsOsr) >= DPS310__MAX_BUSYTIME)
	{
		return DPS__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if (configTemp(tempMr, tempOsr))
	{
		return DPS__FAIL_UNKNOWN;
	}
	//update precision and measuring rate
	if (configPressure(prsMr, prsOsr))
		return DPS__FAIL_UNKNOWN;
	//enable result FIFO
	if (enableFIFO())
	{
		return DPS__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if (setOpMode(CONT_BOTH))
	{
		return DPS__FAIL_UNKNOWN;
	}
	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::standby(void)
{
	//abort if initialization failed
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	//set device to idling mode
	int16_t ret = setOpMode(IDLE);
	if (ret != DPS__SUCCEEDED)
	{
		return ret;
	}
	ret = disableFIFO();
	return ret;
}

int16_t DpsClassSoft::correctTemp(void)
{
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	writeByte(0x0E, 0xA5);
	writeByte(0x0F, 0x96);
	writeByte(0x62, 0x02);
	writeByte(0x0E, 0x00);
	writeByte(0x0F, 0x00);

	//perform a first temperature measurement (again)
	//the most recent temperature will be saved internally
	//and used for compensation when calculating pressure
	float trash;
	measureTempOnce(trash);

	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::getIntStatusFifoFull(void)
{
	return readByteBitfield(config_registers[INT_FLAG_FIFO]);
}

int16_t DpsClassSoft::getIntStatusTempReady(void)
{
	return readByteBitfield(config_registers[INT_FLAG_TEMP]);
}

int16_t DpsClassSoft::getIntStatusPrsReady(void)
{
	return readByteBitfield(config_registers[INT_FLAG_PRS]);
}

//////// 	Declaration of private functions starts here	////////

int16_t DpsClassSoft::setOpMode(uint8_t opMode)
{
	if (writeByteBitfield(opMode, config_registers[MSR_CTRL]) == -1)
	{
		return DPS__FAIL_UNKNOWN;
	}
	m_opMode = (Mode)opMode;
	return DPS__SUCCEEDED;
}

int16_t DpsClassSoft::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
	tempMr &= 0x07;
	tempOsr &= 0x07;
	// two accesses to the same register; for readability
	int16_t ret = writeByteBitfield(tempMr, config_registers[TEMP_MR]);
	ret = writeByteBitfield(tempOsr, config_registers[TEMP_OSR]);

	//abort immediately on fail
	if (ret != DPS__SUCCEEDED)
	{
		return DPS__FAIL_UNKNOWN;
	}
	m_tempMr = tempMr;
	m_tempOsr = tempOsr;
}

int16_t DpsClassSoft::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
	prsMr &= 0x07;
	prsOsr &= 0x07;
	int16_t ret = writeByteBitfield(prsMr, config_registers[PRS_MR]);
	ret = writeByteBitfield(prsOsr, config_registers[PRS_OSR]);

	//abort immediately on fail
	if (ret != DPS__SUCCEEDED)
	{
		return DPS__FAIL_UNKNOWN;
	}
	m_prsMr = prsMr;
	m_prsOsr = prsOsr;
}

int16_t DpsClassSoft::enableFIFO()
{
	return writeByteBitfield(1U, config_registers[FIFO_EN]);
}

int16_t DpsClassSoft::disableFIFO()
{
	int16_t ret = flushFIFO();
	ret = writeByteBitfield(0U, config_registers[FIFO_EN]);
	return ret;
}

uint16_t DpsClassSoft::calcBusyTime(uint16_t mr, uint16_t osr)
{
	//formula from datasheet (optimized)
	return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

int16_t DpsClassSoft::getFIFOvalue(int32_t *value)
{
	uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};

	//abort on invalid argument or failed block reading
	if (value == NULL || readBlock(registerBlocks[PRS], buffer) != DPS__RESULT_BLOCK_LENGTH)
		return DPS__FAIL_UNKNOWN;
	*value = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
	getTwosComplement(value, 24);
	return buffer[2] & 0x01;
}

int16_t DpsClassSoft::readByte(uint8_t regAddress)
{
	m_i2cbus->beginTransmission(m_slaveAddress);
	m_i2cbus->write(regAddress);
	m_i2cbus->endTransmission(false);
	//request 1 byte from slave
	if (m_i2cbus->requestFrom(m_slaveAddress, 1U, 1U) > 0)
	{
		return m_i2cbus->read(); //return this byte on success
	}
	else
	{
		return DPS__FAIL_UNKNOWN; //if 0 bytes were read successfully
	}
}

int16_t DpsClassSoft::writeByte(uint8_t regAddress, uint8_t data)
{
	return writeByte(regAddress, data, 0U);
}

int16_t DpsClassSoft::writeByte(uint8_t regAddress, uint8_t data, uint8_t check)
{
	m_i2cbus->beginTransmission(m_slaveAddress);
	m_i2cbus->write(regAddress);		  //Write Register number to buffer
	m_i2cbus->write(data);				  //Write data to buffer
	if (m_i2cbus->endTransmission() != 0) //Send buffer content to slave
	{
		return DPS__FAIL_UNKNOWN;
	}
	else
	{
		if (check == 0)
			return 0;					  //no checking
		if (readByte(regAddress) == data) //check if desired by calling function
		{
			return DPS__SUCCEEDED;
		}
		else
		{
			return DPS__FAIL_UNKNOWN;
		}
	}
}


int16_t DpsClassSoft::writeByteBitfield(uint8_t data, RegMask_t regMask)
{
	return writeByteBitfield(data, regMask.regAddress, regMask.mask, regMask.shift, 0U);
}

int16_t DpsClassSoft::writeByteBitfield(uint8_t data,
									uint8_t regAddress,
									uint8_t mask,
									uint8_t shift,
									uint8_t check)
{
	int16_t old = readByte(regAddress);
	if (old < 0)
	{
		//fail while reading
		return old;
	}
	return writeByte(regAddress, ((uint8_t)old & ~mask) | ((data << shift) & mask), check);
}

int16_t DpsClassSoft::readByteBitfield(RegMask_t regMask)
{
	int16_t ret = readByte(regMask.regAddress);
	if (ret < 0)
	{
		return ret;
	}
	return (((uint8_t)ret) & regMask.mask) >> regMask.shift;
}

int16_t DpsClassSoft::readBlock(RegBlock_t regBlock, uint8_t *buffer)
{
	//do not read if there is no buffer
	if (buffer == NULL)
	{
		return 0; //0 bytes read successfully
	}

	m_i2cbus->beginTransmission(m_slaveAddress);
	m_i2cbus->write(regBlock.regAddress);
	m_i2cbus->endTransmission(false);
	//request length bytes from slave
	int16_t ret = m_i2cbus->requestFrom(m_slaveAddress, regBlock.length, 1U);
	//read all received bytes to buffer
	for (int16_t count = 0; count < ret; count++)
	{
		buffer[count] = m_i2cbus->read();
	}
	return ret;
}

void DpsClassSoft::getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

int16_t DpsClassSoft::getRawResult(int32_t *raw, RegBlock_t reg)
{
	uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};
	if (readBlock(reg, buffer) != DPS__RESULT_BLOCK_LENGTH)
		return DPS__FAIL_UNKNOWN;

	*raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
	getTwosComplement(raw, 24);
	return DPS__SUCCEEDED;
}

#endif //DPSCLASSSOFT_H_INCLUDED
