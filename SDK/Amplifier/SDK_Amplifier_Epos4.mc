/**
*	@file		SDK_Amplifier_Epos4.mc
*	@brief		This file provides all the function used to setup a Epos4 drive
*	@details	The Epos4 can be operated in the different modes csp and csv
*				The amplifiers can be controlled via EtherCAT or CAN bus.
*				The motor, encoder and controller parameters are set directly with "Epos Studio".
*	$Revision: 274 $
*	@example 	ECAT_3Ax_Epos4-Test_csp.mc
*	@example 	ECAT_1Ax_EPOS4-Test_cst.mc
*	@example 	CAN_1Ax_EPOS4-Test_csp.mc
*	@example 	CAN_2Ax_EPOS4-Test_csp.mc
*	@example 	CAN_1Ax_EPOS4-Test_csv.mc
*	@example 	CAN_1Ax_EPOS4-Test_cst.mc
*/
#pragma once

#include <SysDef.mh>
#include "SDK_Amplifier_Epos4.mh"

/**
*	@brief 		Setup the ECAT bus module for an Epos4
*	@details	This function sets up the ECAT bus module for an Epos4. It can be defined with which operation mode is used.
*				Currently there are no differences in the different operation modes regarding the Bus Module setup.
* 	@param 		axis		Axis module number
* 	@param 		busId		Bus ID of the connected slave
* 	@param 		pdoNumber 	Used PDO number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 1: Cyclic synchronous position (csp) mode \n
*								@b 2: Cyclic synchronous velocity (csv) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_SetupECatBusModule(long axis, long busId, long pdoNumber, long operationMode, long pos_target)
{
	long busmod, slaveNo;

    busmod = axis;
    slaveNo = (busId-1000000);

    print("sdkEpos4_SetupECatBusModule: ",busmod );

    BUSMOD_PARAM(busmod, BUSMOD_MODE) =  BUSMOD_MODE_DEACTIVATE;	// Deactivate (objects will be deleted)
    BUSMOD_PARAM(busmod, BUSMOD_BUSTYPE) =  BUSMOD_BUSTYPE_ECAT_M;          // EtherCAT Master
    BUSMOD_PARAM(busmod, BUSMOD_ID) = slaveNo;                  			// Select the nodeid of the connected slave

    BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT1) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_CMDWORD);   	// Select the input value sources object (send to bus): CMD Word

    if (operationMode==EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFPOS);	// Select the input value sources object (send to bus): position setpoint
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode==EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFVEL);	// Select the input value sources object (send to bus): velocity setvel
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  AXE_PROCESS_SRCINDEX(axis,REG_USERREFCUR);	// Select the input value sources object (send to bus): target torque
    	print("...: cycle synchronous torque (cst) mode");
    }
    else if (operationMode == EPOS4_OP_JPVT)	// jpvt mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) = USER_PARAM_INDEX(pos_target);//AXE_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFPOS);	// Select the input value sources object (send to bus): target torque
    	print("...: jpvt mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0;   // pdo ; length in bytes; bytes offset  control word
    BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2;   // pdo ; length in bytes; bytes offset  target position/ velocity

    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0;   // pdo ; length in bytes; bytes offset   status word
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2;   // pdo ; length in bytes; bytes offset   position actual value
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE3) =  pdoNumber*0x01000000 + 4*0x00010000 + 6;   // pdo ; length in bytes; bytes offset   current actual value

    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE4) =  pdoNumber*0x01000000 + 4*0x00010000 + 10;   // pdo ; length in bytes; bytes offset   velocity actual value
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE5) =  pdoNumber*0x01000000 + 4*0x00010000 + 12;   // pdo ; length in bytes; bytes offset   torque actual value
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE6) =  pdoNumber*0x01000000 + 4*0x00010000 + 14;   // pdo ; length in bytes; bytes offset   joint velocity actual value

    //BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE7) =  pdoNumber*0x01000000 + 4*0x00010000 + 15;   // pdo ; length in bytes; bytes offset   position controller P gain
    //BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE7) =  pdoNumber*0x01000000 + 4*0x00010000 + 16;   // pdo ; length in bytes; bytes offset   target position

    BUSMOD_PARAM(busmod, BUSMOD_MODE) = BUSMOD_MODE_ACTIVATE_NOSTOP;                    // Start bus module

    return(1);
}

/**
*	@brief 		Setup the virtual amplifier for an Epos4 with EtherCat
*	@details	This function sets up the virtual amplifier with EtherCat for an Epos4. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
*	@param		maxRpm			Definition of the maximum motor speed \[rpm\]
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode\n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_SetupECatVirtAmp(long axis, long maxRpm, long operationMode)
{
	print("sdkEpos4_SetupECatVirtAmp: ",axis );

	// virtual amplifiers have a fixed connection to axes number, axes 0 uses amplifier 0.
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CMDWORD)    = AXE_PROCESS_SRCINDEX(axis,REG_CNTRLWORD);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFPOS)     = AXE_PROCESS_SRCINDEX(axis,REG_COMPOS);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFVEL)     = AXE_PROCESS_SRCINDEX(axis,REG_REFERENCE);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFACC)     = AXE_PROCESS_SRCINDEX(axis,PID_FFACCPART);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_STATUS)     = BUSMOD_PROCESS_SRCINDEX(axis,PO_BUSMOD_VALUE1);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CURRENT)    = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE3);

    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWROFF)    = 0x06;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONDIS)  = 0x06;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENP)  = 0x0F;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENN)  = 0x0F;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_QUICKSTOP) = 0x02;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_RESET)     = 0x80;

    VIRTAMP_PARAM(axis,VIRTAMP_STOPDELAY)        = 0x0;
    VIRTAMP_PARAM(axis,VIRTAMP_ERROR_BITMASK)    = 0x0008;
    VIRTAMP_PARAM(axis,VIRTAMP_ERROR_POLARITY)   = 1;

    if (operationMode==EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in csp
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in csp
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode==EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = maxRpm;
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 2* maxRpm;
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in cst
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in cst
    	print("...: cycle synchronous torque (cst) mode");
    }
    else if (operationMode == EPOS4_OP_JPVT)	// jpvt mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in jpvt?
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in jpvt?
    	print("...: jpvt mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    VIRTAMP_PARAM(axis,VIRTAMP_MODE) = VIRTAMP_MODE_ENABLE; 	// has to be the last one because it activates all

    return(1);
}

/**
*	@brief 		Setup the virtual counter input for an Epos4 with EtherCat
*	@details	This function sets up the virtual counter input with EtherCat for an Epos4. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 1: Cyclic synchronous position (csp) mode\n
*								@b 2: Cyclic synchronous velocity (csv) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/


long sdkEpos4_SetupECatVirtCntin(long axis, long operationMode)
{
    print("sdkEpos4_SetupECatVirtCntin: ",axis);

    VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_PISRC_COUNTER) = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE2);

    if (operationMode==EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE_DIRECT;   // Source is absolute and is taken as it is
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode==EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;   		// Source is a position value and difference to last value is added
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode==EPOS4_OP_JPVT)	// cycle synchronous velocity (csv) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;   		//
    	print("...: jpvt mode");
    }
    else											// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    return(1);
}

/**
*	@brief 		Setup the Sdo parameter for an Epos4 with EtherCat
*	@details	This function sets up the Sdo parameter with EtherCat for an Epos4. It can be defined with which operation mode is used.
* 	@param 		busId			Bus ID of the connected slave
* 	@param 		pdoNumber 		Used PDO number
* 	@param 		axisPolarity	Definition of the polarity, @b 0: Normal polarity, @b 1: Inverse polarity
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode\n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/
long sdkEpos4_SetupECatSdoParam(long busId, long pdoNumber, long axisPolarity, long operationMode)
{
	long sm_TX = EPOS4_SYNC_MANAGER_2_PDO_ASSIGNMENT;
	long sm_RX = EPOS4_SYNC_MANAGER_3_PDO_ASSIGNMENT;

	print("sdkEpos4_SetupECatSdoParam Ax: ",busId," pdoNumber ",pdoNumber);

    SdoWriten( busId, sm_TX, 0x00, 1, 0x00);		// disable entry
    SdoWriten( busId, sm_RX, 0x00, 1, 0x00);		// disable entry

    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 0, 1, 0);				// clear pdo 0x1a00 entries
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 1, 4, 0x60410010);		// download pdo 0x1A00 entry: 	statusword
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 2, 4, 0x60640020);		// download pdo 0x1A00 entry:  	actual position
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 3, 4, 0x30D10120);		// download pdo 0x1A00 entry:  	Current actual value averaged [mA]  0x30D1 / 1    32bit
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 4, 4, 0x606C0020);		// download pdo 0x1A00 entry:  	actual velocity
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 5, 4, 0x60770010);		// download pdo 0x1A00 entry:  	actual torque
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 6, 4, 0x34CA0020);		// download pdo 0x1A00 entry:  	joint actual velocity (for JPVT) 32 bits
    //SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 7, 4, 0x30A10110);		// download pdo 0x1A00 entry:  	P gain position controller
    //SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 7, 4, 0x60640020);		// download pdo 0x1A00 entry:  	target position

    SdoWriten( busId, EPOS4_TRANSMIT_PDO_1_MAPPING, 0, 1, 6);				// download pdo 0x1A00 entry:	number of entries



    SdoWriten( busId, EPOS4_TRANSMIT_PDO_2_MAPPING, 0, 1, 0);				// clear pdo 0x1a01 entries
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_3_MAPPING, 0, 1, 0);				// clear pdo 0x1a02 entries
    SdoWriten( busId, EPOS4_TRANSMIT_PDO_4_MAPPING, 0, 1, 0);				// clear pdo 0x1a03 entries

    SdoWriten( busId, EPOS4_RECEIVE_PDO_1_MAPPING, 0, 1, 0);				// clear pdo 0x1600 entries
    SdoWriten( busId, EPOS4_RECEIVE_PDO_1_MAPPING, 1, 4, 0x60400010);		// download pdo 0x1600 entry:	controlword

    if (operationMode==EPOS4_OP_CSP)		// cycle synchronous position (csp) mode
    {
    	SdoWriten( busId, EPOS4_RECEIVE_PDO_1_MAPPING, 2, 4, 0x607A0020);	// download pdo 0x1600 entry:	position set point
    }
    else if (operationMode==EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
      	SdoWriten( busId, EPOS4_RECEIVE_PDO_1_MAPPING, 2, 4,  0x60FF0020);  // download pdo 0x1600 entry: target velocity 32bit
    }
    else if (operationMode == EPOS4_OP_CST) {
		SdoWriten(busId, (EPOS4_RECEIVE_PDO_1_MAPPING), 2, 4, 0x60710010);  	// download pdo 0x1600 entry: Target torque 16bit
    }
    else if (operationMode == EPOS4_OP_JPVT) {
		SdoWriten(busId, EPOS4_RECEIVE_PDO_1_MAPPING, 2, 4, 0x607A0020);  	// download pdo 0x1600 entry: Target position 32bit
    }

    SdoWriten( busId, EPOS4_RECEIVE_PDO_1_MAPPING, 0, 1, 2);				// download pdo 0x1600 entry:	number of entries

    SdoWriten( busId, EPOS4_RECEIVE_PDO_2_MAPPING, 0, 1, 0);				// clear pdo 0x1601 entries
    SdoWriten( busId, EPOS4_RECEIVE_PDO_3_MAPPING, 0, 1, 0);				// clear pdo 0x1602 entries
    SdoWriten( busId, EPOS4_RECEIVE_PDO_4_MAPPING, 0, 1, 0);				// clear pdo 0x1603 entries


    SdoWriten( busId, sm_TX, 1, 2, 0x1600);									// download pdo 0x1C1@b 2:01 index
    SdoWriten( busId, sm_TX, 0, 1, 1);										// download pdo 0x1C12 count

    SdoWriten( busId, sm_RX, 1, 2, 0x1A00 );								// download pdo 0x1C13:01 index
    SdoWriten( busId, sm_RX, 0, 1, 1);										// download pdo 0x1C13 count


    SdoWriten( busId, EPOS4_INTERPOLATION_TIME_PERIOD, 1, 0, 1);			//interpolation time periode value

    if (axisPolarity==1) // set axis polarity
    {
       SdoWriten( busId, EPOS4_AXIS_CONFIGURATION, 4, 4, 1);
       print("...: change axis direction");
     }
    else
    {
    	SdoWriten( busId, EPOS4_AXIS_CONFIGURATION, 4, 4, 0);
    }

    if (operationMode==EPOS4_OP_CSP)
    {
    	SdoWriten( busId, EPOS4_MODES_OF_OPERATION, 0, 1, EPOS4_OP_CSP);				// cycle synchronous position (csp) mode	//$B
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode==EPOS4_OP_CSV)
    {
    	SdoWriten( busId, EPOS4_MODES_OF_OPERATION, 0, 1, EPOS4_OP_CSV);				// cycle synchronous velocity (csv) mode	//$B
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST) {
		SdoWriten(busId, EPOS4_MODES_OF_OPERATION, 0, 1, EPOS4_OP_CST);					// cycle synchronous torque (cst) mode	//$B
    	print("...: cycle synchronous torque (cst) mode");
    }
    else if (operationMode == EPOS4_OP_JPVT) {
		SdoWriten(busId, EPOS4_MODES_OF_OPERATION, 0, 1, EPOS4_OP_JPVT);					// jpvt mode
    	print("...: jpvt mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    return(1);
}

/**
*	@brief 		Setup the Can bus module for an Epos4
*	@details	This function sets up the Can bus module for an Epos4. It can be defined with which operation mode is used.
*				Currently there are no differences in the different operation modes regarding the Bus Module setup.
* 	@param 		axis		Axis module number
* 	@param 		busId		Bus ID of the connected slave
* 	@param 		pdoNumber 	Used PDO number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode\n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@note		The value passed from "BUSMOD_RXMAP_POVALUE3" is the current torque. The value is given in per
*				thousand of “Motor rated torque” and is signed.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_SetupCanBusModule(long axis, long busId, long pdoNumber, long operationMode)
{
	long busmod, signedFlag;
    busmod = axis;
	signedFlag = 0x10000000;

    print("sdkEpos4_SetupCanBusModule: ", busmod);

    BUSMOD_PARAM(busmod, BUSMOD_MODE) =  BUSMOD_MODE_DEACTIVATE;			// Deactivate (objects will be deleted)
    BUSMOD_PARAM(busmod, BUSMOD_BUSTYPE) =  BUSMOD_BUSTYPE_CAN;          	// Can bus
    BUSMOD_PARAM(busmod, BUSMOD_MODE) =  BUSMOD_MODE_ACTIVATE;				// Create bus module

    BUSMOD_PARAM(busmod, BUSMOD_BUSNO) =  busId / 100000;       			// Bus number (0 = master, 1 = slave CAN bus)
    BUSMOD_PARAM(busmod, BUSMOD_ID)   =  busId % 1000;        				// CAN Id for this bus module
    BUSMOD_PARAM(busmod, BUSMOD_SYNC) =  1;                     			// Sync active
    BUSMOD_PARAM(busmod, BUSMOD_GUARDTIME) =  0;               				// no guarding

    BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT1) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_CMDWORD);   	// Select the input value sources object (send to bus): CMD Word

    if (operationMode == EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFPOS);	// Select the input value sources object (send to bus): position setpoint
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  VIRTAMP_PROCESS_SRCINDEX(axis,PO_VIRTAMP_REFVEL);	// Select the input value sources object (send to bus): velocity setvel
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	BUSMOD_PARAM(busmod, BUSMOD_PISRC_INPUT2) =  AXE_PROCESS_SRCINDEX(axis,REG_USERREFCUR);	// Select the input value sources object (send to bus): target torque
    	print("...: cycle synchronous torque (cst) mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0;   // pdo ; length in bytes; bytes offset  control word
    BUSMOD_PARAM(busmod, BUSMOD_TXMAP_INPUT2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2;   // pdo ; length in bytes; bytes offset  target position/ velocity

    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE1) =  pdoNumber*0x01000000 + 2*0x00010000 + 0; // pdo ; length in bytes; bytes offset   status word
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE2) =  pdoNumber*0x01000000 + 4*0x00010000 + 2; // pdo ; length in bytes; bytes offset   position actual value
    BUSMOD_PARAM(busmod, BUSMOD_RXMAP_POVALUE3) =  signedFlag + pdoNumber*0x01000000 + 2*0x00010000 + 6; // pdo ; length in bytes; bytes offset   torque actual value

    BUSMOD_PARAM(busmod, BUSMOD_MODE) = BUSMOD_MODE_ACTIVATE_NOSTOP;                    	// Start bus module

    return(1);
}

/**
*	@brief 		Setup the virtual amplifier for an Epos4 with Can bus
*	@details	This function sets up the virtual amplifier with Can bus for an Epos4. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
*	@param		maxRpm			Definition of the maximum motor speed \[rpm\]
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@note		The value passed from "PO_BUSMOD_VALUE3" to "VIRTAMP_PISRC_CURRENT" is the current torque. The value is given in per
*				thousand of “Motor rated torque”.
*	@return 	value:	Always 1 in this function \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_SetupCanVirtAmp(long axis, long maxRpm, long operationMode)
{
	print("sdkEpos4_SetupCanVirtAmp: ",axis );

	// virtual amplifiers have a fixed connection to axes number, axes 0 uses amplifier 0.
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CMDWORD)    = AXE_PROCESS_SRCINDEX(axis,REG_CNTRLWORD);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFPOS)     = AXE_PROCESS_SRCINDEX(axis,REG_COMPOS);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFVEL)     = AXE_PROCESS_SRCINDEX(axis,REG_REFERENCE);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_REFACC)     = AXE_PROCESS_SRCINDEX(axis,PID_FFACCPART);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_STATUS)     = BUSMOD_PROCESS_SRCINDEX(axis,PO_BUSMOD_VALUE1);
    VIRTAMP_PARAM(axis,VIRTAMP_PISRC_CURRENT)    = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE3);

    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWROFF)    = 0x06;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONDIS)  = 0x06;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENP)  = 0x0F;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_PWRONENN)  = 0x0F;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_QUICKSTOP) = 0x02;
    VIRTAMP_PARAM(axis,VIRTAMP_CNTRLW_RESET)     = 0x80;

    VIRTAMP_PARAM(axis,VIRTAMP_STOPDELAY)        = 0x00;
    VIRTAMP_PARAM(axis,VIRTAMP_ERROR_BITMASK)    = 0x08;
    VIRTAMP_PARAM(axis,VIRTAMP_ERROR_POLARITY)   = 1;

    if (operationMode == EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in csp
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in csp
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = maxRpm;
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 2* maxRpm;
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	VIRTAMP_PARAM(axis,VIRTAMP_REF100PERC)       = 0;  // not used in cst
    	VIRTAMP_PARAM(axis,VIRTAMP_REFLIMIT)         = 0;  // not used in cst
    	print("...: cycle synchronous torque (cst) mode");
    }
    else	// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    VIRTAMP_PARAM(axis,VIRTAMP_MODE) = VIRTAMP_MODE_ENABLE; 	// has to be the last one because it activates all

    return(1);
}

/**
*	@brief 		Setup the virtual counter input for an Epos4 with Can bus
*	@details	This function sets up the virtual counter input for an Epos4. It can be defined with which operation mode is used.
* 	@param 		axis			Axis module number
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode\n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_SetupCanVirtCntin(long axis, long operationMode)
{
    print("sdkEpos4_SetupCanVirtCntin: ",axis);

    VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_PISRC_COUNTER) = BUSMOD_PROCESS_SRCINDEX(axis, PO_BUSMOD_VALUE2);

    if (operationMode == EPOS4_OP_CSP)	// cycle synchronous position (csp) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE_DIRECT;   // Source is absolute and is taken as it is
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == EPOS4_OP_CSV)	// cycle synchronous velocity (csv) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE;   		// Source is a position value and difference to last value is added
    	print("...: cycle synchronous velocity (csv) mode");
    }
    else if (operationMode == EPOS4_OP_CST)	// cycle synchronous torque (cst) mode
    {
    	VIRTCOUNTIN_PARAM(axis,VIRTCNTIN_MODE) = VIRTCNTIN_MODE_ABSOLUTE_DIRECT;   // Source is absolute and is taken as it is
    	print("...: cycle synchronous torque (cst) mode");
    }
    else											// Error
    {
    	print("...: not supported operation mode");
    	return(-1);
    }

    return(1);
}

/**
*	@brief 		Setup the Sdo parameter for an Epos4
*	@details	This function sets up the Sdo parameter for an Epos4. It can be defined with which operation mode is used.
*				The CANSYNCTIMER must be set before calling this function.
* 	@param 		busId			Bus ID of the connected slave
* 	@param 		pdoNumber 		Used PDO number
* 	@param 		axisPolarity	Definition of the polarity, @b 0: Normal polarity, @b 1: Inverse polarity
* 	@param 		operationMode	Definition of the operation mode \n
*								@b 0x08: Cyclic synchronous position (csp) mode\n
*								@b 0x09: Cyclic synchronous velocity (csv) mode\n
*								@b 0x0A: Cyclic synchronous torque (cst) mode
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/
long sdkEpos4_SetupCanSdoParam(long busId, long pdonumber, long axisPolarity, long operationMode)
///////////////////////////////////////////////////////////////////////////////
{
    long nodeid, last_value;
    nodeid = busId % 100000;

	print("sdkEpos4_SetupCanSdoParam: " ,nodeid);

    if (axisPolarity==1){	// set axis polarity
		SdoWrite( busId, EPOS4_AXIS_CONFIGURATION, 4, 1);
		print("...: change axis direction");
    }
    else{
		SdoWrite( busId, EPOS4_AXIS_CONFIGURATION, 4, 0);
    }

	//interpolation time periode value
    SdoWrite(busId, EPOS4_INTERPOLATION_TIME_PERIOD, 1, GLB_PARAM(CANSYNCTIMER));
	print("...: set interpolation time periode: ", GLB_PARAM(CANSYNCTIMER), " ms");

    // reset Transmittype
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_1_PARAMETER, 2, 255);    // TxPDO1 Transmittype
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_2_PARAMETER, 2, 255);    // TxPDO2 Transmittype
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_3_PARAMETER, 2, 255);    // TxPDO3 Transmittype
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_4_PARAMETER, 2, 255);    // TxPDO4 Transmittype

    // the pdos have to be disabled for configuring
    last_value = (SdoRead(busId, EPOS4_TRANSMIT_PDO_1_PARAMETER, 1));
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_1_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 1
    last_value = (SdoRead(busId, EPOS4_TRANSMIT_PDO_2_PARAMETER, 1));
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_2_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 2
    last_value = (SdoRead(busId, EPOS4_TRANSMIT_PDO_3_PARAMETER, 1));
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_3_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 3
    last_value = (SdoRead(busId, EPOS4_TRANSMIT_PDO_4_PARAMETER, 1));
    SdoWrite(busId, EPOS4_TRANSMIT_PDO_4_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 4

    //the pdos have to be disabled for configuring
    last_value = (SdoRead(busId, EPOS4_RECEIVE_PDO_1_PARAMETER, 1));
    SdoWrite(busId, EPOS4_RECEIVE_PDO_1_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 1
    last_value = (SdoRead(busId, EPOS4_RECEIVE_PDO_2_PARAMETER, 1));
    SdoWrite(busId, EPOS4_RECEIVE_PDO_2_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 2
    last_value = (SdoRead(busId, EPOS4_RECEIVE_PDO_3_PARAMETER, 1));
    SdoWrite(busId, EPOS4_RECEIVE_PDO_3_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 3
    last_value = (SdoRead(busId, EPOS4_RECEIVE_PDO_4_PARAMETER, 1));
    SdoWrite(busId, EPOS4_RECEIVE_PDO_4_PARAMETER, 1, (last_value | 0x80000000)); // disable pdo 4

    // Now we setup the correct PDO transmission for Tx and Rx
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_PARAMETER + pdonumber-1), 2,  1);			// TxPDO Transmittype SYNC
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_PARAMETER + pdonumber-1), 2,  1);				// RxPDO Transmittype SYNC

    // config RX PDO
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber-1), 0x00, 0);				// disable
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber-1), 0x01, 0x60400010);	// controlword

    if (operationMode == EPOS4_OP_CSP) {											// cycle synchronous position (csp) mode
    	SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x607A0020);	// download pdo 0x1600 entry:	position set point
    	SdoWrite(busId, EPOS4_MODES_OF_OPERATION, 0, EPOS4_OP_CSP);					// mode of operation CSP	//$B
    	print("...: cycle synchronous position (csp) mode");
    }
    else if (operationMode == EPOS4_OP_CSV ) {
		SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x60FF0020);  	// download pdo 0x1600 entry: target velocity 32bit
		SdoWrite(busId, EPOS4_MODES_OF_OPERATION, 0, EPOS4_OP_CSV);					// mode of operation CSV	//$B
    	print("...: cycle synchronous velocity (csv) mode");
    }
   else if (operationMode == EPOS4_OP_CST) {
		SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber-1), 2, 0x60710010);  	// download pdo 0x1600 entry: Target torque 32bit
		SdoWrite(busId, EPOS4_MODES_OF_OPERATION, 0, EPOS4_OP_CST);					// mode of operation CST	//$B
    	print("...: cycle synchronous torque (cst) mode");
    }
    else {	// Error
		print("...: not supported operation mode");
    	return(-1);
    }
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_MAPPING + pdonumber - 1), 0x00, 2);     		// enable

    // config TX PDO
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x00, 0);     		// disable
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x01, 0x60410010);	// statusword
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x02, 0x60640020);	// actpos
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x03, 0x30D20110);	// Torque actual value averaged
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_MAPPING + pdonumber - 1), 0x00, 3 );			// enable

    // fill in the wanted pdo
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_PARAMETER + pdonumber - 1), 1, (0x80000180 + (pdonumber - 1)*0x100 + nodeid) );// fill in pdo
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_PARAMETER + pdonumber - 1), 1, (0x80000200 + (pdonumber - 1)*0x100 + nodeid) );// fill in pdo

    // enable
    SdoWrite(busId, (EPOS4_TRANSMIT_PDO_1_PARAMETER + pdonumber - 1), 1, (0x00000180 + (pdonumber - 1)*0x100 + nodeid) );// enable pdo
    SdoWrite(busId, (EPOS4_RECEIVE_PDO_1_PARAMETER + pdonumber - 1), 1, (0x00000200 + (pdonumber - 1)*0x100 + nodeid) );// enable pdo

	return(1);
}

/**
*	@brief 		State machine function for performing a homing on an EPOS4
*	@details	This function sets the EPOS4 in homing mode and starts the homing defined on the EPOS4. Before calling this function,
*				all homing parameters must be configured on the EPOS4. The function is not blocking.
*				The return parameter must be checked.
* 	@param 		axis			Axis module number
* 	@param 		busId			Bus ID of the connected slave
* 	@param 		operationMode 	Operation mod which should be set after homing.
* 	@param 		homingState		Call-by-reference variable for the iteration of the different homing states.
*								Must be initialized with 0.
*	@return 	value:	Process value \n
*				value 	> 0 Process successful 	\n
*				value 	= 0 Process is active 	\n
*				value 	< 0 Error
*
*/

long sdkEpos4_AxisHomingStart(long axis, long busId, long operationMode, long &homingState)
///////////////////////////////////////////////////////////////////////////////
{
    long time, displayMode;
    long homingMethode = 0;

    time= Time()%1000;

   	switch (homingState) {
		case 0:
			print("sdkEpos4_AxisHomingStart: " ,axis);
			homingMethode = SdoRead(busId, EPOS4_HOMING_METHOD,0);
			print("...: EPOS4 Homing AxisNo: ",axis," - Homing methode: ",homingMethode);
   			homingState = 1;
   			break;
		case 1:
			print("...: EPOS4 Homing AxisNo: ",axis," - Enable drive");
			// The trajectory generator of the EPOS4 is used for homing. With this command the axis is switched on
			// and the trajectory generator of the MACS is switched off. Thus REG_COMPOS = REG_ACTPOS is set.
		   	AxisControl(axis, USERREFVEL);

			// x6060 Change operation mode to ”6: Homing mode.”
			SdoWriten( busId, EPOS4_MODES_OF_OPERATION, 0, 1, EPOS4_OP_HMM);	//$B
			print("...: EPOS4 Homing AxisNo: ",axis," - Set mode of OP: 0x06");

			homingState = 2;

		case 2:
			displayMode = SdoRead(busId, EPOS4_MODES_OF_OPERATION_DISPLAY, 0);	//$B
			if(displayMode== EPOS4_OP_HMM)
			{
				print("...: EPOS4 Homing AxisNo: ",axis," - Display mode of OP: ",displayMode);
				homingState = 3;
			}
			else if(time==0)
			{
				printf("...: EPOS4 Homing AxisNo: %ld - Waiting for OP mode display:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
		   	break;

		case 3:
			// Bit 2 Operation enabled - EPOS4 is ready to start
			// Bit 10 Target reached - Homing procedure is interrupted or not started
			if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[2]==1 && VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[10]==1)
			{
				printf("...: EPOS4 Homing AxisNo: %ld - Homing start signal Status:   %lx\n",axis,VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENP) = 0x1F;
				VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENN) = 0x1F;
				homingState = 4;
			}
			// Bit 13 Homingerror - Homing error occurred
			else if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[13]==1)
			{
				printf("...: EPOS4 Homing AxisNo: %ld - Homing Error - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				return(-1);
			}
			else if(time==0)
			{
				printf("...: EPOS4 Homing AxisNo: %ld - Waiting for ready bit:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
			break;

		case 4:
			   	// Homing procedure is completed successfully
				if (VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[12]==1)
				{
					printf("...: EPOS4 Homing AxisNo: %ld - Homing done - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));

					VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENP) = 0x0F;
					VIRTAMP_PARAM(axis, VIRTAMP_CNTRLW_PWRONENN) = 0x0F;

					SdoWriten(busId, 0x6060, 0, 1, operationMode );        // 0x6060 Change operation mode to ”xxx mode.”

					homingState = 5;
					break;
			   	}
				// Bit 13 Homingerror - Homing error occurred
				else if(VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS).i[13]==1)
				{
					printf("...: EPOS4 Homing AxisNo: %ld - Homing Error - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
					return(-1);
				}
				// Homing procedure is in progress
			   	else if(time==0)
			   	{
			   		printf("...: EPOS4 Homing AxisNo: %ld - Homing in progress - Status:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
			   		// This section  must be called in a 1 ms loop because of the print function.
			   		Delay(1);
			   	}
			   	break;

		case 5:
			displayMode = SdoRead(busId, EPOS4_MODES_OF_OPERATION_DISPLAY, 0);	//$B
			if(displayMode== operationMode)
			{
				print("...: EPOS4 Homing AxisNo: ",axis," - Display mode of OP: ",displayMode);
				homingState = 6;
			}
			else if(time==0)
			{
				printf("...: EPOS4 Homing AxisNo: %ld - Waiting for OP mode display:   %lx\n",axis, VIRTAMP_PROCESS(axis,PO_VIRTAMP_STATUS));
				// This section  must be called in a 1 ms loop because of the print function.
				Delay(1);
			}
		case 6:
			print("...: EPOS4 Homing AxisNo: ",axis," - Disable Drive");
		   	AxisControl(axis, OFF);
		   	homingState = 7;
		case 7:
			return(1);
		default :
			print("...: EPOS4 Homing AxisNo: ",axis," - Incorrect hominState value: ", homingState);
			return(-1);
	}

	return(0);
}

/**
*	@brief 		Prints a error description for Epos4 errors
*	@details	This function prints a description for Epos4 error codes
* 	@param 		errorCode		Epos4 error code
*
*/
void sdkEpos4_PrintErrorDescription(long errorCode)
{
	switch (errorCode) {
		case EPOS4_F_NO_ERROR:
			printf("No error.");
			break;
		case EPOS4_F_GENERIC:
			printf("Generic error.");
			break;
		case EPOS4_F_GENERIC_INIT1:
		case EPOS4_F_GENERIC_INIT2:
		case EPOS4_F_GENERIC_INIT3:
		case EPOS4_F_GENERIC_INIT4:
		case EPOS4_F_GENERIC_INIT5:
		case EPOS4_F_GENERIC_INIT6:
		case EPOS4_F_GENERIC_INIT7:
		case EPOS4_F_GENERIC_INIT8:
		case EPOS4_F_GENERIC_INIT9:
			printf("Generic initialization error.");
			break;
		case EPOS4_F_FIRMWARE_INCOMPATIBILITY:
			printf("Firmware incompatibility.");
			break;
		case EPOS4_F_OVERCURRENT:
			printf("Overcurrent.");
			break;
		case EPOS4_F_POWER_STAGE_PROTECTION:
			printf("Power stage protection.");
			break;
		case EPOS4_F_OVERVOLTAGE:
			printf("Overvoltage.");
			break;
		case EPOS4_F_UNDERVOLTAGE:
			printf("Undervoltage.");
			break;
		case EPOS4_F_THERMAL_OVERLOAD:
			printf("Thermal overload.");
			break;
		case EPOS4_F_THERMAL_MOTOR_OVERLOAD:
			printf("Thermal motor overload.");
			break;
		case EPOS4_F_LOGIC_VOLTAGE_LOW:
			printf("Logic supply voltage too low.");
			break;
		case EPOS4_F_HARDWARE_DEFECT:
			printf("Hardware defect.");
			break;
		case EPOS4_F_HARDWARE_INCOMPATIBILITY:
			printf("Hardware incompatibility.");
			break;
		case EPOS4_F_HARDWARE1:
		case EPOS4_F_HARDWARE2:
		case EPOS4_F_HARDWARE3:
		case EPOS4_F_HARDWARE4:
			printf("Hardware error.");
			break;
		case EPOS4_F_SIGN_OF_LIFE:
			printf("Sign of life.");
			break;
		case EPOS4_F_EXT_1_WATCHDOG:
			printf("Extension 1 watchdog.");
			break;
		case EPOS4_F_INTERNAL_SOFTWARE:
			printf("Internal software.");
			break;
		case EPOS4_F_SOFTWARE_PARAM:
			printf("Software parameter error.");
			break;
		case EPOS4_F_PERSISTENT_PARAM_CORRUPT:
			printf("Persistent param corrupt.");
			break;
		case EPOS4_F_POSITION_SENSOR:
			printf("Position sensor error.");
			break;
		case EPOS4_F_POSITION_BREACH:
			printf("Position breach.");
			break;
		case EPOS4_F_POSITION_RESOLUTION:
			printf("Position resolution error.");
			break;
		case EPOS4_F_POSITION_INDEX:
			printf("Position index.");
			break;
		case EPOS4_F_HALL_SENSOR:
			printf("Hall sensor error.");
			break;
		case EPOS4_F_HALL_NOT_FOUND:
			printf("Hall not found.");
			break;
		case EPOS4_F_HALL_ANGLE:
			printf("Hall angle detection error.");
			break;
		case EPOS4_F_SSI_SENSOR:
			printf("SSI sensor.");
			break;
		case EPOS4_F_SSI_FRAME:
			printf("SSI frame.");
			break;
		case EPOS4_F_MISSING_MAIN_SENSOR:
			printf("Missing main sensor.");
			break;
		case EPOS4_F_MISSING_COMM_SENSOR:
			printf("Missing comm sensor.");
			break;
		case EPOS4_F_MAIN_SENSOR_DIR:
			printf("Main sensor direction.");
			break;
		case EPOS4_F_CAN_OVERRUN_OBJ_LOST:
			printf("CAN overrun (object lost).");
			break;
		case EPOS4_F_CAN_OVERRUN:
			printf("CAN overrun.");
			break;
		case EPOS4_F_CAN_PASSIVE_MODE:
			printf("CAN passive mode.");
			break;
		case EPOS4_F_CAN_HEARTBEAT:
			printf("CAN heartbeat error.");
			break;
		case EPOS4_F_CAN_PDO_COB_ID_COLLISION:
			printf("CAN PDO COB-ID collision.");
			break;
		case EPOS4_F_ETHERCAT_COMM:
			printf("EtherCAT communication error.");
			break;
		case EPOS4_F_ETHERCAT_INIT:
			printf("EtherCAT initialization error.");
			break;
		case EPOS4_F_ETHERCAT_RX_QUEUE:
			printf("EtherCAT Rx queue overflow error.");
			break;
		case EPOS4_F_ETHERCAT_COMM_INTERNAL:
			printf("EtherCAT communication (internal) error.");
			break;
		case EPOS4_F_ETHERCAT_CYCLE_TIME:
			printf("EtherCAT communication cycle time error.");
			break;
		case EPOS4_F_CAN_BUS_OFF:
			printf("CAN bus turned off.");
			break;
		case EPOS4_F_CAN_RX_QUEUE:
			printf("CAN Rx queue overflow.");
			break;
		case EPOS4_F_CAN_TX_QUEUE:
			printf("CAN Tx queue overflow.");
			break;
		case EPOS4_F_CAN_PDO_LENGTH:
			printf("CAN PDO length error.");
			break;
		case EPOS4_F_RPDO_TIMEOUT:
			printf("RPDO timeout.");
			break;
		case EPOS4_F_ETHERCAT_PDO_COMM:
			printf("EtherCAT PDO communication error.");
			break;
		case EPOS4_F_ETHERCAT_SDO_COMM:
			printf("EtherCAT SDO communication error.");
			break;
		case EPOS4_F_FOLLOWING:
			printf("Following error.");
			break;
		case EPOS4_F_NEG_LIMIT_SWITCH:
			printf("Negative limit switch error.");
			break;
		case EPOS4_F_POS_LIMIT_SWITCH:
			printf("Positive limit switch error.");
			break;
		case EPOS4_F_SOFTWARE_POSITION_LIMIT:
			printf("Software position limit error.");
			break;
		case EPOS4_F_STO:
			printf("STO error.");
			break;
		case EPOS4_F_SYSTEM_OVERLOADED:
			printf("System overloaded.");
			break;
		case EPOS4_F_WATCHDOG:
			printf("Watchdog error.");
			break;
		case EPOS4_F_SYSTEM_PEAK_OVERLOADED:
			printf("System peak overloaded.");
			break;
		case EPOS4_F_CONTROLLER_GAIN:
			printf("Controller gain error.");
			break;
		case EPOS4_F_AUTO_TUNING_ID:
			printf("Auto tuning identification error.");
			break;
		case EPOS4_F_AUTO_TUNING_CURRENT_LIMIT:
			printf("Auto tuning current limit error.");
			break;
		case EPOS4_F_AUTO_TUNING_ID_CURRENT:
			printf("Auto tuning identification current error.");
			break;
		case EPOS4_F_AUTO_TUNING_DATA_SAMPLING:
			printf("Auto tuning data sampling error.");
			break;
		case EPOS4_F_AUTO_TUNING_SAMPLE_MISMATCH:
			printf("Auto tuning sample mismatch error.");
			break;
		case EPOS4_F_AUTO_TUNING_PARAM:
			printf("Auto tuning parameter error.");
			break;
		case EPOS4_F_AUTO_TUNING_AMPLITUDE_MISMATCH:
			printf("Auto tuning amplitude mismatch error.");
			break;
		case EPOS4_F_AUTO_TUNING_TIMEOUT:
			printf("Auto tuning timeout error.");
			break;
		case EPOS4_F_AUTO_TUNING_STANDSTILL:
			printf("Auto tuning standstill error.");
			break;
		case EPOS4_F_AUTO_TUNING_TORQUE_INVALID:
			printf("Auto tuning torque invalid error.");
			break;
		case EPOS4_F_AUTO_TUNING_MAX_SPEED:
			printf("Auto tuning max system speed error.");
			break;
		case EPOS4_F_AUTO_TUNING_MOTOR_CONNECTION:
			printf("Auto tuning motor connection error.");
			break;
		case EPOS4_F_AUTO_TUNING_SENSOR_SIGNAL:
			printf("Auto tuning sensor signal error.");
			break;
		default:
			if(errorCode >= 0x6180 && errorCode <= 0x61F0) //0x6180 through 0x61F0 is internal software error
			{
				printf("Internal software error.");
			}
			else
			{
				printf("Unknown error code: 0x%04X\n", errorCode);
			}
			break;
	}
}
