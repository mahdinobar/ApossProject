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
#define C_AXIS1	0						// Axis module number here

#define C_AXIS1_POLARITY 	0		// Definition of the polarity 0: Normal, 1: Inverse

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

#define pos_target			2

long main(void) {

    long slaveCount, i,retval, axisIndex;
    long homingStateAx_0=0;

    long status;
	long masterStatus;
	long adcRawValue;
    double voltage;
    long slaveAddress;

	long resval;
	long pGain;

    long ActPosition, ActVelocity, ActTorque, ActAvgCurrent, ActJointVelocity, PosPgain;
    long positionLog[1000];

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

	// initialising maxon drives
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXIS1_POLARITY, EPOS4_OP_JPVT );

	sdkEtherCATMasterDoMapping();

	sdkEtherCATSetupDC(1, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]

	// starting the ethercat
	sdkEtherCATMasterStart();

	// All axis have in this example the same parameters

	// setup EtherCAT bus module for cst mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_JPVT);

	// setup virtual amplifier for cst mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_JPVT);

	// setup irtual counter for cst mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, EPOS4_OP_JPVT);
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
	//AXE_PARAM(C_AXIS1,POSERR)=0;


	//// Homing statemachine
	//retval=0;
	//while(retval!=1)
	//{
	//	retval.i[0] = 	sdkEpos4_AxisHomingStart(C_AXIS1, C_DRIVE_BUSID1, EPOS4_OP_JPVT, homingStateAx_0);
	//}

	AxisControl(C_AXIS1,ON);

	print("");
	print("-----------------------------------------------------------");
	print("                Set torque in JPVT Mode                    ");
	print("----------------------------------------------------------- \n");

	AxisControl(C_AXIS1, ON);

	Vel(C_AXIS1, 50);
	Acc(C_AXIS1, 30);
	Dec(C_AXIS1, 30);

	status = ECatMasterInfo(0x1000, 0, masterStatus);  // Pass the variable, not the address
	if (status == 0) {
		print("EtherCAT master status: ", masterStatus);
		if (masterStatus == 0x08) {  // EC_STATE_OPERATIONAL
			print("Master is operational.");
		} else {
			print("Master is not operational.");
		}
	} else {
		print("Error: Unable to retrieve EtherCAT master status.");
	}

	USER_PARAM(pos_target) = 0;
	//#define USER_PARAM(parno)                           Sysvar[USER_PARAM_INDEX(parno)]

	//ActPosition = Sysvar[0x01606400];
	//ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(0,3)];
	//#define BUSMOD_PROCESS_INDEX(modno,parno)           ((0x01000000 | (SDOINDEX_BUSMOD_PROCESS<<8)) | (((long) (modno))<<8) | ((long) (parno)))

	RecordIndex(0x01606400, 0x01606C00, 0x0134CA00, 0x01607700, 0x01607A00);
	RecordStart(0); // Start recording (until RecordStop or DYNMEM is filled up)

	print("00000000000000000000000000000000000000000000000000000000000000000000000000000000");
	SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x01, 8000);
	resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x01);
	SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x03, 4000);
	resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x03);
	//SdoWrite(C_DRIVE_BUSID1, 0x607A, 0x00, 0);
	resval = SdoRead(C_DRIVE_BUSID1, 0x607A, 0x00);
	print(">>>>>>> go to position=",resval);
	Sysvar[0x01607A00]=0;
	//resval = Sysvar[0x01607A00];
	//print("!!!!!!!!!!!!!! go to position2=",resval);
	Delay(2000);
	ActPosition = Sysvar[0x01606400];  // 0x01606400 is the correct SDO for actual position
	ActVelocity = Sysvar[0x01606C00];  // 0x01606400 is the correct SDO for actual velocity
	ActJointVelocity = Sysvar[0x0134CA00];  // 0x01606400 is the correct SDO for actual joint velocity
	ActTorque = Sysvar[0x01607700];  // 0x01606400 is the correct SDO for actual torque
	print("Actual Position: ", ActPosition);
	print("Actual Velocity: ", ActVelocity);
	print("Actual Torque: ", ActTorque);
	ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(0,3)];
	print("Actual Averaged Current: ", ActAvgCurrent);
	print("00000000000000000000000000000000000000000000000000000000000000000000000000000000");
	Delay(1000);


	SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x01, 80000);
	resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x01);
	print("JPVTC controller P gain=",resval);
	SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x03, 4000);
	resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x03);
	print("JPVTC controller D gain=",resval);

	// Configure system objects for recording:
	// Actual position, Command position, Tracking Error, system timer
	//RecordIndex(0x01250001, 0x01250002, 0x01250006, 0x01220217, 0x01606400);

	//AxisPosAbsStart(C_AXIS1, 500);
	//AxisPosRelStart(C_AXIS1, 500);
	//SdoWrite(C_DRIVE_BUSID1, 0x607A, 0x00, 500); //set Target position
	USER_PARAM(pos_target) = 500;
	resval = SdoRead(C_DRIVE_BUSID1, 0x607A, 0x00);
	print("resval=",resval);

	//Sysvar[0x01607A00]=500;
	//resval = Sysvar[0x01607A00];
	//print("resval2=",resval);

	print("++++++++++++++++++++++++++++++++++++++++ Target position is set=",resval);
	//AxisWaitReached(C_AXIS1);
	//Delay(3000);
	//Sysvar[0x01607A00]=500;
	//SdoWriten(C_DRIVE_BUSID1, 0x1600, 2, 4, 0x607A0020);
	//Vel(0, 50); // 50% of maximum velocity
	//AxisPosAbsStart(0, 500); // Start positioning
	//AxisWaitReached(0); // Wait until position is reached
	//Vel(0, 100); // Increase velocity to 100%
	//AxisPosAbsStart(0, 0); // Use new velocity for moving back
	//AxisWaitReached(0); // Wait until position is reached

	Delay(1000); // Wait 200ms
	//RecordStop(0, 0); // Stop recording



	// Loop to send torque and print position every 1 ms
    for (i = 0; i < 0; i++) {  // Example loop to run for 1000 iterations
        // Send motor rated torque (e.g., 1000 mNm)
        //AXE_PROCESS(C_AXIS1, REG_USERREFCUR) = 100;  // Set the torque (adjust value as needed)
        //AxisPosRelStart(C_AXIS1, 2000);
       	//AxisWaitReached(C_AXIS1);
		//print("Target position is reached \n");
		//print("Start, back to start position");

		//AxisPosAbsStart(C_AXIS1, 500);
		//AxisPosAbsStart( C_AXIS1, 0);
		//AxisWaitReached(C_AXIS1);
		//print("Start position is reached");
		print(i, " repetitions to go \n");
		//SdoWrite(C_DRIVE_BUSID1, 0x607A, 0x00, 500); //set Target position
		resval = SdoRead(C_DRIVE_BUSID1, 0x607A, 0x00);
		print("resval=",resval);

		//Sysvar[0x01607A00]=500;
		resval = Sysvar[0x01607A00];
		print("resval2=",resval);


        //resval = SdoRead(C_DRIVE_BUSID1, 0x30A1, 0x01);
		//print("resval=",resval);


		//resval = SdoRead(C_DRIVE_BUSID1, 0x6040, 0x00);
		//print("Controlword=",resval);
		//SdoWrite(C_DRIVE_BUSID1, 0x3000, 0x02, 68309521);
		//resval = SdoRead(C_DRIVE_BUSID1, 0x3000, 0x02);
		//print("Control structure=",resval);
		//resval = SdoRead(C_DRIVE_BUSID1, 0x3001, 0x03);
		//print("Number of pole pairs=",resval);


		//SdoWrite(C_DRIVE_BUSID1, 0x30AE, 0x01, 30);
		//SdoWrite(C_DRIVE_BUSID1, 0x30A0, 0x01, 3000);
		//SdoWrite(C_DRIVE_BUSID1, 0x30A0, 0x02, 7000);
		//SdoWrite(C_DRIVE_BUSID1, 0x30AE, 0x02, 30);


		// Retrieve the actual position (using Sysvar to read the actual position)
		ActPosition = Sysvar[0x01606400];  // 0x01606400 is the correct SDO for actual position
        //ActVelocity = Sysvar[0x01606C00];  // 0x01606400 is the correct SDO for actual velocity
        //ActJointVelocity = Sysvar[0x0134CA00];  // 0x01606400 is the correct SDO for actual velocity
        //ActTorque = Sysvar[0x01607700];  // 0x01606400 is the correct SDO for actual torque
        //PosPgain = Sysvar[0x0130A101];  // 0x01606400 is the correct SDO for position controller P gain
        //print("PosPgain=",PosPgain);
        // Print the actual values
        print("Actual Position: ", ActPosition);
        positionLog[i] = positionLog[i] = ActPosition;ActPosition;
        //print("Actual Velocity: ", ActVelocity);
        //print("Actual Torque: ", ActTorque);
        //ActAvgCurrent = Sysvar[0x0130D101];  // 0x01606400 is the correct SDO for actual averaged current
        // Sysvar[BUSMOD_PROCESS_INDEX(C_AXIS1,PO_BUSMOD_VALUE3)
        //ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(0,3)];
        //print("Actual Averaged Current: ", ActAvgCurrent);
        // Delay for 1 ms
        Delay(1);  // Assuming Delay(1) causes a 20-millisecond delay
    }
    RecordStop(0, 0); // Stop recording


    //for (i = 0; i < 1000; i++) {
     //   print("i=",i);
       // print(positionLog[i]);
        //Delay(1);
    //}

	AxisControl(C_AXIS1, OFF);

	print("Program done, Axis OFF ");
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