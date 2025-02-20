/**
*	@brief		This is a testprogram which demonstrates the use of the EPOS4 SDK
*	@detail		The drive will run back and forth when the program is started. It will also
*				record some test data. This data can be displayed by pressing the "Oscilloscope(Record)"
*				button after the program has run.
*
*	$Revision: 274 $
*
*	@example 	ECAT_1Ax_EPOS4-Test_cst.mc
*
*/

#define F_HALL 0x7388
#define F_HALL_ANGLE 0x738A
#define F_STO 0x8A88

#define EPOS4_ACTUAL_ANALOG_INPUT 0x3160


#include "SDK_ApossC.mc"

// Parameters for the SDK function
#define C_AXIS1	0						// Axis module number
#define C_AXIS2	1

#define C_AXIS1_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse
#define C_AXIS2_POLARITY 	0

#define C_DRIVE_BUSID1 1000001			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID2 1000002			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus

#define C_EC_CYCLE_TIME	1				// Cycletime in milliseconds
#define C_EC_OFFSET		1				// Shift offset
#define C_PDO_NUMBER	1				// Used PDO number

// Encoder settings & axis user units (MACS)
#define C_AXIS_ENCRES 			4*1600						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS_MAX_RPM			2000						// Maximum velocity in RPM
#define C_AXIS_VELRES			100							// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED		// Defines the ramptype
#define C_AXIS_RAMPMIN			800							// Maximum acceleration
#define C_AXIS_JERKMIN			1000						// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			0						// There is also a following error on EPOS4, could be set to zero on the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS_KPROP			0
#define	C_AXIS_KINT				0
#define	C_AXIS_KDER				0
#define	C_AXIS_KILIM			0
#define	C_AXIS_KILIMTIME		0
#define	C_AXIS_BANDWIDTH		1000
#define	C_AXIS_FFVEL			1000
#define	C_AXIS_KFFAC			0
#define	C_AXIS_KFFDEC			0


long main(void) {

    long slaveCount, i,retval, axisIndex;
    long homingStateAx_0=0,  homingStateAx_1=0;

    long status;
	long masterStatus;
	long adcRawValue;
    double voltage;

    long ActPosition, ActVelocity, ActTorque, ActAvgCurrent;


	print("-----------------------------------------------------------");
	print(" Test application EtherCAT Master with 2 EPOS4 drive");
	print("-----------------------------------------------------------");

	ErrorClear();
	AmpErrorClear(C_AXIS1); // Clear error on EPOS4
	AmpErrorClear(C_AXIS2); // Clear error on EPOS4
	InterruptSetup(ERROR, ErrorHandler);

	ECatMasterCommand(0x1000, 0);


	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	slaveCount = sdkEtherCATMasterInitialize();
	print("slavecount: ",slaveCount);

	// initialising maxon drives
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CST );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXIS2_POLARITY, EPOS4_OP_CST );

	sdkEtherCATMasterDoMapping();


	sdkEtherCATSetupDC(1, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
	sdkEtherCATSetupDC(2, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]


	// starting the ethercat
	sdkEtherCATMasterStart();

	// All axis have in this example the same parameters

	// setup EtherCAT bus module for cst mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CST);
	sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, EPOS4_OP_CST);

	// setup virtual amplifier for cst mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CST);
	sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_AXIS_MAX_RPM, EPOS4_OP_CST);

	// setup irtual counter for cst mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CST);
	sdkEpos4_SetupECatVirtCntin(C_AXIS2, EPOS4_OP_CST);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	C_AXIS1,
								C_AXIS_VELRES,
								C_AXIS_MAX_RPM,
								C_AXIS_RAMPTYPE,
								C_AXIS_RAMPMIN,
								C_AXIS_JERKMIN,
								C_AXIS_TRACKERR
								);
	sdkSetupAxisMovementParam(	C_AXIS2,
								C_AXIS_VELRES,
								C_AXIS_MAX_RPM,
								C_AXIS_RAMPTYPE,
								C_AXIS_RAMPMIN,
								C_AXIS_JERKMIN,
								C_AXIS_TRACKERR
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS1,
								C_AXIS_POSENCREV,
								C_AXIS_POSENCQC,
								C_AXIS_POSFACT_Z,
								C_AXIS_POSFACT_N,
								C_AXIS_FEEDREV,
								C_AXIS_FEEDDIST
								);
	sdkSetupAxisUserUnits(		C_AXIS2,
								C_AXIS_POSENCREV,
								C_AXIS_POSENCQC,
								C_AXIS_POSFACT_Z,
								C_AXIS_POSFACT_N,
								C_AXIS_FEEDREV,
								C_AXIS_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS1,
									C_AXIS_KPROP,
									C_AXIS_KINT,
									C_AXIS_KDER,
									C_AXIS_KILIM,
									C_AXIS_KILIMTIME,
									C_AXIS_BANDWIDTH,
									C_AXIS_FFVEL,
									C_AXIS_KFFAC,
									C_AXIS_KFFDEC
									);
	sdkSetupPositionPIDControlExt( 	C_AXIS2,
									C_AXIS_KPROP,
									C_AXIS_KINT,
									C_AXIS_KDER,
									C_AXIS_KILIM,
									C_AXIS_KILIMTIME,
									C_AXIS_BANDWIDTH,
									C_AXIS_FFVEL,
									C_AXIS_KFFAC,
									C_AXIS_KFFDEC
									);


	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	ErrorClear();

	//----------------------------------------------------------------
	// Homing start
	//----------------------------------------------------------------

	// Homing setup
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_METHOD, 						0,   EPOS4_HOMING_ACT_POSITION);	// 0x6098 Set homing method to “-4" : Homing Method -4 (Current Threshold Negative Speed).”
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_SPEEDS, 						1,   20);    						// Homing Speed / Speed for switch speed [velocity units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_SPEEDS, 						2,   20);    						// Homing Speed / Speed for zero search [velocity units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_ACCELERATION, 				0,   20);    						// Homing acceleration [acceleration units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOME_OFFSET_MOVE_DISTANCE, 			0,   6400);   						// Home offset move distance [position units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOME_POSITION, 						0,   0);   							// Home position [position units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_CURRENT_THRESHOLD_FOR_HOMING_MODE, 	0,   1500);  						// Current threshold for homing mode [mA]

	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOMING_METHOD, 						0,   EPOS4_HOMING_ACT_POSITION);	// 0x6098 Set homing method to “-4" : Homing Method -4 (Current Threshold Negative Speed).”
	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOMING_SPEEDS, 						1,   20);    						// Homing Speed / Speed for switch speed [velocity units]
	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOMING_SPEEDS, 						2,   20);    						// Homing Speed / Speed for zero search [velocity units]
	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOMING_ACCELERATION, 				0,   20);    						// Homing acceleration [acceleration units]
	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOME_OFFSET_MOVE_DISTANCE, 			0,   6400);   						// Home offset move distance [position units]
	SdoWrite( C_DRIVE_BUSID2, EPOS4_HOME_POSITION, 						0,   0);   							// Home position [position units]
	SdoWrite( C_DRIVE_BUSID2, EPOS4_CURRENT_THRESHOLD_FOR_HOMING_MODE, 	0,   1500);  						// Current threshold for homing mode [mA]

	print("...: EPOS4 Homing AxisNo: ",C_AXIS1," - Homing methode: ",SdoRead(C_DRIVE_BUSID1, EPOS4_HOMING_METHOD,0));
	print("...: EPOS4 Homing AxisNo: ",C_AXIS2," - Homing methode: ",SdoRead(C_DRIVE_BUSID2, EPOS4_HOMING_METHOD,0));

	// Disable MACS trackerror for homing
	AXE_PARAM(C_AXIS1,POSERR)=0;
	AXE_PARAM(C_AXIS2,POSERR)=0;


	// Homing statemachine
	retval=0;

	while(retval!=2)
	{
		retval.i[0] = 	sdkEpos4_AxisHomingStart(C_AXIS1, C_DRIVE_BUSID1, EPOS4_OP_CST, homingStateAx_0);
		retval.i[1] = 	sdkEpos4_AxisHomingStart(C_AXIS2, C_DRIVE_BUSID2, EPOS4_OP_CST, homingStateAx_1);
	}

	AxisControl(C_AXIS1,ON, C_AXIS2,ON);

	print("");
	print("-----------------------------------------------------------");
	print("                Set torque in CST Mode                       ");
	print("----------------------------------------------------------- \n");

	Vel(C_AXIS1, 40, C_AXIS2, 40);
	Acc(C_AXIS1, 30, C_AXIS2, 30);
	Dec(C_AXIS1, 30, C_AXIS2, 30);

	status = ECatMasterInfo(0x1000, 0, masterStatus);  // Pass the variable, not the address
	if (status == 0) {
		print("EtherCAT master status: ", masterStatus);
		if (masterStatus == 0x08) {  // EC_STATE_OPERATIONAL
			print("Master is operational.");
		} else {
			print("masterStatus=",masterStatus);
			print("Master is not operational.");
		}
	} else {
		print("Error: Unable to retrieve EtherCAT master status.");
	}



	for(i=0;i>=0;i--)
	{
		// The value is given in per thousand of “Motor rated torque”
		print("Set target torque for C_AXIS1 → positive");
		AXE_PROCESS(C_AXIS1,REG_USERREFCUR)= 100;

		Delay(4000);
		print("Set target torque for C_AXIS2 → negative");
		AXE_PROCESS(C_AXIS2,REG_USERREFCUR)= -40;

		Delay(3000);
		print("Set target torque for both C_AXIS1 and C_AXIS2 → zero");
		AXE_PROCESS(C_AXIS1,REG_USERREFCUR)= 0;
		AXE_PROCESS(C_AXIS2,REG_USERREFCUR)= 0;

		Delay(3000);
		print(i, " repetitions to go \n");

		adcRawValue = SdoRead(C_DRIVE_BUSID1, EPOS4_ACTUAL_ANALOG_INPUT, 1);
		print("Potentiometer ADC raw value for C_AXIS1: ", adcRawValue);
		voltage = (adcRawValue * 10.0) / 4096.0;  // Convert to voltage
		print("Potentiometer ADC Voltage for C_AXIS2: ", voltage, " V");


		adcRawValue = SdoRead(C_DRIVE_BUSID2, EPOS4_ACTUAL_ANALOG_INPUT, 1);
		print("Potentiometer ADC raw value for C_AXIS2: ", adcRawValue);
		voltage = (adcRawValue * 10.0) / 4096.0;  // Convert to voltage
		print("Potentiometer ADC Voltage for C_AXIS2: ", voltage, " V");

		ActPosition = Sysvar[BUSMOD_PROCESS_INDEX(0, 2)];  // 0x01606400 is the correct SDO for actual position
        print("Actual Position Axis 0: ", ActPosition);
        ActPosition = Sysvar[BUSMOD_PROCESS_INDEX(1, 2)];  // 0x01606400 is the correct SDO for actual position
        print("Actual Position Axis 1: ", ActPosition);


	}


	// Loop to send torque and print position every 1 ms
    for (i = 0; i < 500; i++) {  // Example loop to run for 1000 iterations
        // Send motor rated torque (e.g., 1000 mNm)
        AXE_PROCESS(C_AXIS1, REG_USERREFCUR) = 100;  // Set the torque (adjust value as needed)
        // Retrieve the actual position (using Sysvar to read the actual position)
        ActPosition = Sysvar[BUSMOD_PROCESS_INDEX(0, 2)];  // actual position
        ActVelocity = Sysvar[BUSMOD_PROCESS_INDEX(0, 4)];  // actual velocity
        ActTorque = Sysvar[BUSMOD_PROCESS_INDEX(0, 5)];  // actual torque
        ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(0, 3)];  // actual averaged current
        print("Actual Position Axis 0: ", ActPosition);
        print("Actual Velocity Axis 0: ", ActVelocity);
        print("Actual Torque Axis 0: ", ActTorque);
        print("Actual Average Current Axis 1: ", ActAvgCurrent);

        // Send motor rated torque (e.g., 1000 mNm)
        AXE_PROCESS(C_AXIS2, REG_USERREFCUR) = 100;  // Set the torque (adjust value as needed)
        // Retrieve the actual position (using Sysvar to read the actual position)
        ActPosition = Sysvar[BUSMOD_PROCESS_INDEX(1, 2)];  // actual position
        ActVelocity = Sysvar[BUSMOD_PROCESS_INDEX(1, 4)];  // actual velocity
        ActTorque = Sysvar[BUSMOD_PROCESS_INDEX(1, 5)];  // actual torque
        ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(1, 3)];  // actual averaged current
        print("Actual Position Axis 1: ", ActPosition);
        print("Actual Velocity Axis 1: ", ActVelocity);
        print("Actual Torque Axis 1: ", ActTorque);
        print("Actual Average Current Axis 1: ", ActAvgCurrent);


        Delay(10);
    }

	AxisControl(AXALL, OFF);

	print("Program done, ALL Axis OFF ");
    return(0);
}

void ErrorHandler(void)
{
	long axeNbr 	= ErrorAxis();
	long errNbr		= ErrorNo();
	long errInfoNbr	= ErrorInfo();
	long sdoAbortCode, eposErr;

    AxisControl(AXALL,OFF);

    print("-----------------------------------------------------------");
    printf("Error %d: ",errNbr);
    sdkErrorPrint_ApossIdeErrorDescription(errNbr);
    print();
    print("...Info: ",errInfoNbr, " AxisNo: ", axeNbr);
    print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    print();


  	switch(errNbr)
	{
		case F_AMP:		eposErr = SdoRead(C_DRIVE_BUSID1,EPOS4_ERROR_CODE,0x00);
						printf("Error Axis: %d, Epos4 Error 0x%lX: ", axeNbr , eposErr);
						sdkEpos4_PrintErrorDescription(eposErr);
						print();
						AmpErrorClear(axeNbr); // Clear error on EPOS4
						break;

		case F_CANIO:	print("ErrorNo: ",errNbr," SlaveAddr(info): ",errInfoNbr);
						ECatMasterInfo(0x1000, 22, sdoAbortCode);
						printf("SDO Abort Code EtherCAT 0x%lX\n", sdoAbortCode);
						break;

		default:		print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
	}
	ErrorClear();
	print("");	print(" There is no error handlig → Exit()");
	Exit(0);
}