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


long main(void) {

    long slaveCount, i,retval, axisIndex;
    long homingStateAx_0=0;

    long status;
	long masterStatus;	//$B
	long adcRawValue;
    double voltage;
    long slaveAddress;

	long resval;
	long pGain;

    long ActPosition, ActVelocity, ActTorque, ActAvgCurrent, ActJointVelocity, PosPgain;

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

	status = ECatMasterInfo(0x1000, 0, masterStatus);  // Pass the variable, not the address	//$B
	if (status == 0) {
		print("EtherCAT master status: ", masterStatus);	//$B
		if (masterStatus == 0x08) {  // EC_STATE_OPERATIONAL	//$B
			print("Master is operational.");
		} else {
			print("Master is not operational.");
		}
	} else {
		print("Error: Unable to retrieve EtherCAT master status.");
	}


	// Loop to send torque and print position every 1 ms
    for (i = 0; i < 20; i++) {  // Example loop to run for 1000 iterations
        // Send motor rated torque (e.g., 1000 mNm)
        //AXE_PROCESS(C_AXIS1, REG_USERREFCUR) = 100;  // Set the torque (adjust value as needed)
        AxisPosAbsStart(C_AXIS1, 2000);
       	AxisWaitReached(C_AXIS1);
		print("Target position is reached \n");
		print("Start, back to start position");

		AxisPosAbsStart( C_AXIS1, 0);
		AxisWaitReached(C_AXIS1);
		print("Start position is reached");
		print(i, " repetitions to go \n");

        // Retrieve the actual position (using Sysvar to read the actual position)
        ActPosition = Sysvar[0x01606400];  // 0x01606400 is the correct SDO for actual position
        ActVelocity = Sysvar[0x01606C00];  // 0x01606400 is the correct SDO for actual velocity
        ActJointVelocity = Sysvar[0x0134CA00];  // 0x01606400 is the correct SDO for actual velocity
        ActTorque = Sysvar[0x01607700];  // 0x01606400 is the correct SDO for actual torque
        //PosPgain = Sysvar[0x0130A101];  // 0x01606400 is the correct SDO for position controller P gain
        //print("PosPgain=",PosPgain);

        //resval = SdoRead(C_DRIVE_BUSID1, 0x30A1, 0x01);
		//print("resval=",resval);
		resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x01);
		SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x01, 55000);
		print("JPVTC controller P gain=",resval);
		SdoWrite(C_DRIVE_BUSID1, 0x34C6, 0x03, 2000);
		resval = SdoRead(C_DRIVE_BUSID1, 0x34C6, 0x03);
		print("JPVTC controller D gain=",resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x6040, 0x00);
		print("Controlword=",resval);
		//SdoWrite(C_DRIVE_BUSID1, 0x3000, 0x02, 68309521);
		resval = SdoRead(C_DRIVE_BUSID1, 0x3000, 0x02);
		print("Control structure=",resval);
		resval = SdoRead(C_DRIVE_BUSID1, 0x3001, 0x03);
		print("Number of pole pairs=",resval);


		SdoWrite(C_DRIVE_BUSID1, 0x607A, 0x00, 500); //Target position
		//SdoWrite(C_DRIVE_BUSID1, 0x30AE, 0x01, 30);
		//SdoWrite(C_DRIVE_BUSID1, 0x30A0, 0x01, 3000);
		//SdoWrite(C_DRIVE_BUSID1, 0x30A0, 0x02, 7000);
		//SdoWrite(C_DRIVE_BUSID1, 0x30AE, 0x02, 30);

		print("+++++++++++++++++++++++++++++++++++++++++++++");
		resval = SdoRead(C_DRIVE_BUSID1, 0x607A, 0x00);
		print("Target position=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3000, 0x02);
		print("Control structure=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3001, 0x03);
		print("Number of pole pairs=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3001, 0x04);
		print("Thermal time constant winding=", resval, " s");

		resval = SdoRead(C_DRIVE_BUSID1, 0x3001, 0x05);
		print("Torque constant=", resval, " mN/mA");

		resval = SdoRead(C_DRIVE_BUSID1, 0x3003, 0x01);
		print("Gear reduction numerator=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3003, 0x02);
		print("Gear reduction denominator=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3003, 0x03);
		print("Max gear input speed=", resval, " rpm");

		resval = SdoRead(C_DRIVE_BUSID1, 0x3010, 0x01);
		print("Digital incremental encoder 1 number of pulses=", resval, " pulses/rev");

		resval = SdoRead(C_DRIVE_BUSID1, 0x3012, 0x02);
		print("SSI number of data bits=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3012, 0x03);
		print("SSI encoding type=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x3012, 0x05);
		print("SSI timeout time=", resval, " us");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A0, 0x01);
		print("Current controller P gain=", resval, " mV/A");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A0, 0x02);
		print("Current controller I gain=", resval, " mV/(A*ms)");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A1, 0x01);
		print("Position controller P gain=", resval, " mA/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A1, 0x02);
		print("Position controller I gain=", resval, " mA/(rad*s)");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A1, 0x03);
		print("Position controller D gain=", resval, " mA*s/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A2, 0x01);
		print("Velocity controller P gain=", resval, " mA*s/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A2, 0x02);
		print("Velocity controller I gain=", resval, " mA/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30A2, 0x05);
		print("Velocity controller filter cut-off frequency=", resval, " Hz");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x01);
		print("Main loop P gain low bandwidth=", resval, " 1/s");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x02);
		print("Main loop P gain high bandwidth=", resval, " 1/s");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x20);
		print("Auxiliary loop P gain=", resval, " mA*s/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x21);
		print("Auxiliary loop I gain=", resval, " mA/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x30);
		print("Auxiliary loop observer position correction gain=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x31);
		print("Auxiliary loop observer velocity correction gain=", resval, " Hz");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x32);
		print("Auxiliary loop observer load correction gain=", resval, " mNm/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x33);
		print("Auxiliary loop observer friction=", resval, " uNm/rad");

		resval = SdoRead(C_DRIVE_BUSID1, 0x30AE, 0x34);
		print("Auxiliary loop observer inertia=", resval, " g*cm^2");

		resval = SdoRead(C_DRIVE_BUSID1, 0x607F, 0x00);
		print("Max profile velocity=", resval, " mrpm");

		resval = SdoRead(C_DRIVE_BUSID1, 0x6080, 0x00);
		print("Max motor speed=", resval, " rpm");

		resval = SdoRead(C_DRIVE_BUSID1, 0x6081, 0x00);
		print("Profile velocity=", resval, " mrpm");

		resval = SdoRead(C_DRIVE_BUSID1, 0x6083, 0x00);
		print("Profile acceleration=", resval, " rpm/s");

		resval = SdoRead(C_DRIVE_BUSID1, 0x6084, 0x00);
		print("Profile deceleration=", resval, " rpm/s");

		resval = SdoRead(C_DRIVE_BUSID1, 0x60A9, 0x00);
		print("SI unit velocity=", resval);

		resval = SdoRead(C_DRIVE_BUSID1, 0x6060, 0x00);
		print("Modes of operation=", resval);
		print("------------------------------------------");



        // Print the actual values
        print("Actual Position: ", ActPosition);
        print("Actual Velocity: ", ActVelocity);
        print("Actual Torque: ", ActTorque);
        //ActAvgCurrent = Sysvar[0x0130D101];  // 0x01606400 is the correct SDO for actual averaged current
        // Sysvar[BUSMOD_PROCESS_INDEX(C_AXIS1,PO_BUSMOD_VALUE3)
        ActAvgCurrent = Sysvar[BUSMOD_PROCESS_INDEX(0,3)];
        print("Actual Averaged Current: ", ActAvgCurrent);
        // Delay for 1 ms
        Delay(100);  // Assuming Delay(1) causes a 20-millisecond delay
    }


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