#define F_HALL 0x7388
#define F_HALL_ANGLE 0x738A
#define F_STO 0x8A88

#define EPOS4_ACTUAL_ANALOG_INPUT 0x3160



#include "SDK_ApossC.mc"

// Parameters for the SDK function
#define C_AXIS1 0						// Axis module number

#define C_AXIS1_POLARITY 0		// Definition of the polarity 0: Normal, 1: Inverse

#define C_DRIVE_BUSID1 1000001			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus

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


// Define appropriate register addresses or use the API to access these values
#define REG_ACTPOS 0x6064     // Actual Position Register
#define REG_ACTTORQUE 0x6077  // Actual Torque Register
#define REG_USERREFCUR 0x6071// Target Torque Register

long main(void) {

    long slaveCount, i,retval, axisIndex;
    long homingStateAx_0=0;

    long adcRawValue;
    long activeTxPDOs;
    long mappedObject1;
    long adcPdoValue;
    double voltage;

	print("-----------------------------------------------------------");
	print(" Test application EtherCAT Master with 1 EPOS4 drive");
	print("-----------------------------------------------------------");

	ErrorClear();
	AmpErrorClear(C_AXIS1); // Clear error on EPOS4
	InterruptSetup(ERROR, ErrorHandler);

	ECatMasterCommand(0x1000, 0);


	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	slaveCount = sdkEtherCATMasterInitialize();
	print("slavecount: ",slaveCount);



	// Step 1: Enable configuration mode
	//SdoWrite(C_DRIVE_BUSID1, 0x1C12, 0, 0);  // Disable RxPDOs
	// SdoWrite(C_DRIVE_BUSID1, 0x1C13, 0, 0);  // Disable TxPDOs
	// Step 2: Assign a new TxPDO (0x1A01) for Analog Input
	// SdoWrite(C_DRIVE_BUSID1, 0x1A01, 0, 0);  // Clear existing mappings in 0x1A01
	// SdoWrite(C_DRIVE_BUSID1, 0x1A01, 1, 0x20310010);  // Map Analog Input (0x2031, 16-bit)
	// Step 3: Activate TxPDOs
	//SdoWrite(C_DRIVE_BUSID1, 0x1C13, 1, 0x1A00);  // Keep TxPDO1
	//SdoWrite(C_DRIVE_BUSID1, 0x1C13, 2, 0x1A01);  // Add new TxPDO2 (Analog Input)
	// Step 4: Finalize Mapping
	//sdkEtherCATMasterDoMapping();



	// initialising maxon drives
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_CST );

	sdkEtherCATMasterDoMapping();


	sdkEtherCATSetupDC(1, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]


	// starting the ethercat
	sdkEtherCATMasterStart();

	// All axis have in this example the same parameters

	// setup EtherCAT bus module for cst mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CST);

	// setup virtual amplifier for cst mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CST);

	// setup irtual counter for cst mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_CST);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	C_AXIS1,
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

	print("...: EPOS4 Homing AxisNo: ",C_AXIS1," - Homing methode: ",SdoRead(C_DRIVE_BUSID1, EPOS4_HOMING_METHOD,0));

	// Disable MACS trackerror for homing
	AXE_PARAM(C_AXIS1,POSERR)=0;


	// Homing statemachine
	retval=0;

	while(retval!=1)
	{
		retval.i[0] = 	sdkEpos4_AxisHomingStart(C_AXIS1, C_DRIVE_BUSID1, EPOS4_OP_CST, homingStateAx_0);
	}

    AxisControl(C_AXIS1, ON);
    print("-----------------------------------------------------------");
    print("        Printing Actual Position, Torque, and Target Torque ");
    print("----------------------------------------------------------- \n");

    while (1) {
        long actual_position = AXE_PROCESS(C_AXIS1, REG_ACTPOS);
        long actual_torque = AXE_PROCESS(C_AXIS1, REG_ACTTORQUE);
        long target_torque = AXE_PROCESS(C_AXIS1, REG_USERREFCUR);

        print("Actual Position: ", actual_position);
        print("Actual Torque: ", actual_torque);
        print("Target Torque: ", target_torque);
        print("----------------------------------");

        Delay(100);  // Adjust as needed
    }

    AxisControl(C_AXIS1, OFF);
    print("Program done, Axis OFF");
    return 0;
}
void ErrorHandler(void)
{
    long axeNbr = ErrorAxis();           // Get the axis number with the error
    long errNbr = ErrorNo();             // Get the error number
    long errInfoNbr = ErrorInfo();       // Get additional error information
    long sdoAbortCode;                  // Variable for SDO abort code
    long eposErr;                       // Declare eposErr variable

    AxisControl(AXALL, OFF);             // Disable all axes

    print("-----------------------------------------------------------");
    printf("Error %d: ", errNbr);
    sdkErrorPrint_ApossIdeErrorDescription(errNbr);   // Print error description
    print();
    print("...Info: ", errInfoNbr, " AxisNo: ", axeNbr);
    print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    print();

    switch (errNbr)
    {
        case F_AMP:
            eposErr = SdoRead(C_DRIVE_BUSID1, EPOS4_ERROR_CODE, 0x00);  // Read the EPOS4 error code
            printf("Error Axis: %d, Epos4 Error 0x%lX: ", axeNbr, eposErr);
            sdkEpos4_PrintErrorDescription(eposErr);    // Print the error description from EPOS4
            print();
            AmpErrorClear(axeNbr);  // Clear the error on EPOS4
            break;

        case F_CANIO:
            print("ErrorNo: ", errNbr, " SlaveAddr(info): ", errInfoNbr);
            ECatMasterInfo(0x1000, 22, sdoAbortCode);
            printf("SDO Abort Code EtherCAT 0x%lX\n", sdoAbortCode);
            break;

        default:
            print("ErrorNo: ", errNbr, " info: ", errInfoNbr, " AxisNo: ", axeNbr);
    }

    ErrorClear();   // Clear any errors
    print("");       // Print a blank line
    print("There is no error handling → Exit()");
    Exit(0);         // Exit the program
}
