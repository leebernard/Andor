/*
Hacked
Andor example program showing the use of the SDK to perform a single
full image acquisition from the CCD.
The image is saved in file image.fits
*/

#include "fitsimage.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
using namespace std;

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define EXIT1           1
#define FAILURE        -1
#define SUCCESS         0

#define UNS16           0       /* FITS BITPIX 16 and unsigned */
#define InvalidTemp     999
#define InvalidIdx     -1
#define CHAN0           0       /* iXon has only one channel */
#define AMPEMCCD        0
#define AMPCONV         1
#define MAXADCHAN       10
#define MAXOUTAMP       10
#define MAXHSSPEED      10
#define MAXVSSPEED      10
#define MAXPREAMPGAIN   10
#define MAXFRAME        1000000
#define MUSECperSEC     1000000
#define GEThbin         0
#define GETvbin         1

// the Andor sdk defines initial pixel as 1
#define INIPIX          1

#define FrameTransferModeInvalid        -1
#define FrameTransferModeOff            0
#define FrameTransferModeOn             1

#define ReadModeInvalid         -1
#define ReadModeImage           4

#define AcquireModeInvalid      -1
#define AcquireModeSingleScan   1
#define AcquireModeAccumulate   2
#define AcquireModeKinetics     3
#define AcquireModeFastKinetics 4
#define AcquireModeRunTillAbort 5

#define ShutterTTLopenLo        0
#define ShutterTTLopenHi        1

#define ShutterModeInvalid      -1
#define ShutterModeFullAuto     0
#define ShutterModePermOpen     1
#define ShutterModePermClos     2
#define ShutterModeOpenFVBser   4
#define ShutterModeOpenANYser   5

#define OutputAmpInvalid        -1
#define OutputAmpElectronMult   0
#define OutputAmpConventional   1

/*******************************************************************************/

#define MAXINITWAIT 20

int main(int argc, char* argv[])
{
    unsigned long       error;
    bool                quit;
    char                choice;
    float               etime, atime, ktime;
    int                 sleepus;
    unsigned int        gotstat;
    int                 currstat;
    int                 prevstat;
    int                 initok          = 0;
    int                 initwait        = 0;

    int                 ctemp;                  /* current temp */
    int                 itemp           = InvalidTemp;  /* target temp */
    float               stemp, ttemp, atemp, cvolt;
    int                 curttemp        = InvalidTemp;

    int                 curemadvgain;
    int                 curemccdgain;
    int                 emgainlo, emgainhi;

    int                 curemgainmode   = InvalidIdx;

    char                hexstring[1024];

    int                 numframes   = 1;

    int                 hsspeedidx;
    int                 curhsspeedidx   = InvalidIdx;

    char                fitsfname[1024];

    int                 camidx;
    int                 caminfo;

    int                 numadchan;
    int                 adchanidx;
    int                 curadchanidx    = InvalidIdx;

    float               curhsspeed      = InvalidIdx;

    int                 curshutmode     = ShutterModeInvalid;

    int                 numamp;
    int                 outampidx;
    int                 curoutampidx    = OutputAmpInvalid;

    int                 numpreampgain;

    int                 numvsspeed;
    int                 fastvsspeedidx;
    float               fastvsspeed;

    float               vsspeed[MAXVSSPEED];

    int                 bitdepth[MAXADCHAN];

    int                 numhsspeed[MAXADCHAN][MAXOUTAMP];

    float               hsspeed[MAXADCHAN][MAXOUTAMP][MAXHSSPEED];

    float               preampgain[MAXADCHAN][MAXOUTAMP][MAXHSSPEED][MAXPREAMPGAIN];
    int                 preampgainavail[MAXADCHAN][MAXOUTAMP][MAXHSSPEED][MAXPREAMPGAIN];

    AndorCapabilities   andorcaps;

    int                 newint;
    float               newfloat;
    int                 howmany;


    int                 readmode;
    int                 curreadmode     = ReadModeInvalid;

    int                 curacqmode      = AcquireModeInvalid;

			// manual says default is off
    int                 curftmode       = FrameTransferModeOff;

    int                 curnumkinetics  =

    char                inputline[256];

    struct tm           tm;

    // default binning
    int                 curhbin = 1;
    int                 curvbin = 1;
    int                 maxhbin;
    int                 maxvbin;
    int                 newhbin;
    int                 newvbin;

    // initial pixel of readout region
    // it looks like Andor sdk uses 1-indexed pixel counts
    int                 curhbeg     = INIPIX;
    int                 curvbeg     = INIPIX;
    int                 curhend     = -1;
    int                 curvend     = -1;
    int                 maxhend;
    int                 maxvend;
    int                 newhbeg;
    int                 newvbeg;
    int                 newhend;
    int                 newvend;

    int                 curisocropexactv        = 0;
    int                 curisocropexvsiz        = 256;
    int                 curisocropexhsiz        = 256;
    int                 curisocropexvbeg        = 1;
    int                 curisocropexhbeg        = 1;
    int                 newisocropexactv;
    int                 newisocropexvsiz;
    int                 newisocropexhsiz;
    int                 newisocropexvbeg;
    int                 newisocropexhbeg;

    int                 curisocropactv          = 0;
    int                 curisocropvsiz          = 256;
    int                 curisocrophsiz          = 256;
    int                 newisocropactv;
    int                 newisocropvsiz;
    int                 newisocrophsiz;

    int                 dobreak;

    int                 numfails    = 0;

    /********************************************/

    camidx = CameraSelect (argc, argv);
    if (camidx < 0) {
	cout << "*** CAMERA SELECTION ERROR" << endl;
	return FAILURE;
    }
    cout << "camidx = " << camidx << endl;

    //Initialize CCD
    cout << "Initialize..." << std::flush;
    error = Initialize((char *)"/usr/local/etc/andor");
    if(error!=DRV_SUCCESS){
	cout << "Initialisation error...exiting" << endl;
	return EXIT1;
    }
    cout << "done" << endl;

    while (! initok && initwait < MAXINITWAIT ) {
	// ask the camera how it feels
	// we might want to continue invoking GetStatus() during normal operation
	gotstat   = GetStatus(&currstat);
	switch(gotstat) {
	    case DRV_SUCCESS:
		initok  = 1;
		break;
	    case DRV_NOT_INITIALIZED:
		cout << "initwait " << initwait << endl;
		initwait++;
		usleep(100000);
		break;
	    default:
		cout << "GetStatus() returned undocumented response " << gotstat << endl;
		// other status values are not defined so we exit
		return FAILURE;
		break;
	} /* end switch gotstat */
    } /* end while initok */
    cout << "current status " << drvdef(currstat) << endl;
    if (initwait >= MAXINITWAIT) {
	cout << "did not init within " << MAXINITWAIT << "%d tries" << endl;
    }

    gotstat = GetCameraInformation(camidx, &caminfo);
    if (gotstat == DRV_SUCCESS) {
	if (!(caminfo & 01)) cout << "NOT ";
	cout << "USB camera present" << endl;

	if (!(caminfo & 02)) cout << "NOT ";
	cout << "All dlls loaded properly" << endl;

	if (!(caminfo & 04)) cout << "NOT ";
	cout << "Camera Initialized correctly" << endl;
    } else {
	cout << "FAIL GetCameraInformation()" << endl;
	numfails++;
    }

    /***********************************************************/
    // refer to page 81 of sdk2 api manual

    gotstat = GetNumberADChannels(&numadchan);
    cout << "NUMADCHAN " << numadchan
    << " status " << drvdef(gotstat) << endl;
    if (gotstat == DRV_SUCCESS) {
	// WHY are we setting to channel 0?
	// because we know answer to number of AD channels was 1
	adchanidx   = 0;
    } else {
	numfails++;
    }

    gotstat = SetADChannel(adchanidx);
    cout
    << "SetADChannel(" << adchanidx << ")"
    << " status " << drvdef(gotstat) << endl;
    if (gotstat == DRV_SUCCESS) {
	curadchanidx = adchanidx;
    } else {
	numfails++;
    }

    gotstat = GetNumberAmp(&numamp);
    cout << "NUMAMP " << numamp
    << " status " << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    gotstat = GetNumberPreAmpGains(&numpreampgain);
    cout << "NUMPREAMPGAIN " << numpreampgain
    << " status " << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    gotstat = GetNumberVSSpeeds(&numvsspeed);
    cout << "NUMVSSPEED " << numvsspeed
    << " status " << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    gotstat = GetFastestRecommendedVSSpeed(&fastvsspeedidx,  &fastvsspeed);
    cout
    << "FASTVSSPEEDIDX "        << fastvsspeedidx
    << " FASTVSSPEED "          << fastvsspeed << " us "
    << " status " << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    for (int vsspeedidx = 0; vsspeedidx < numvsspeed; vsspeedidx++)
    {
	gotstat = GetVSSpeed(vsspeedidx, &vsspeed[vsspeedidx]);
	cout
	<< "VSSPEED["   << vsspeedidx << "] "  << vsspeed[vsspeedidx] << " us"
	<< " status "   << drvdef(gotstat) << endl;
	if (gotstat != DRV_SUCCESS) {
	    numfails++;
	}
    }

    for (int adchanidx = 0; adchanidx < numadchan; adchanidx++)
    {

	gotstat = GetBitDepth(adchanidx, &bitdepth[adchanidx]);
	cout
	<< "BITDEPTH[adchanidx=" << adchanidx << "]"
	<< bitdepth[adchanidx]
	<< " status "   << drvdef(gotstat) << endl;
	if (gotstat != DRV_SUCCESS) {
	    numfails++;
	}

	for (outampidx = 0; outampidx < numamp; outampidx++)
	{
	    // suppose that numpreampgains depends on chosen output amp
	    // it looks like output amp defaults to 0 meaning EMCCD
	    gotstat = SetOutputAmplifier(outampidx);
	    cout
	    << "SetOutputAmplifier " << outampidx
	    << " status " << drvdef(gotstat) << endl;
	    curoutampidx        = outampidx;
	    if (gotstat != DRV_SUCCESS) {
		numfails++;
	    }

	    gotstat = GetNumberPreAmpGains(&numpreampgain);
	    cout
	    << " outampidx "     << outampidx
	    << " NUMPREAMPGAIN " << numpreampgain
	    << " status "        << drvdef(gotstat) << endl;
	    if (gotstat != DRV_SUCCESS) {
		numfails++;
	    }

	    gotstat = GetNumberHSSpeeds(adchanidx, outampidx, &numhsspeed[adchanidx][outampidx]);
	    cout
	    << "NUMHSSPEED"
	    << "[adchanidx="    << adchanidx    << "]"
	    << "[outampidx="    << outampidx    << "]"
	    << " " << numhsspeed[adchanidx][outampidx]
	    << " status "       << drvdef(gotstat) << endl;
	    if (gotstat != DRV_SUCCESS) {
		numfails++;
	    }

	    for (hsspeedidx = 0; hsspeedidx < numhsspeed[adchanidx][outampidx]; hsspeedidx++)
	    {
		gotstat = GetHSSpeed(adchanidx, outampidx, hsspeedidx,
		&hsspeed[adchanidx][outampidx][hsspeedidx]);
		cout
		<< "HSSPEED"
		<< "[adchanidx="        << adchanidx    << "]"
		<< "[outampidx="        << outampidx    << "]"
		<< "[hsspeedidx="       << hsspeedidx   << "]"
		<< " " << hsspeed[adchanidx][outampidx][hsspeedidx] << " MHz"
		<< " status "           << drvdef(gotstat) << endl;
		if (gotstat != DRV_SUCCESS) {
		    numfails++;
		}

		for (int preampgainidx = 0; preampgainidx < numpreampgain; preampgainidx++)
		{
		    gotstat = GetPreAmpGain( preampgainidx,
		    &preampgain[adchanidx][outampidx][hsspeedidx][preampgainidx]);
		    cout
		    << "PREAMPGAIN"
		    << "[adchanidx="        << adchanidx        << "]"
		    << "[outampidx="        << outampidx        << "]"
		    << "[hsspeedidx="       << hsspeedidx       << "]"
		    << "[preampgainidx="    << preampgainidx    << "]"
		    << " " << preampgain[adchanidx][outampidx][hsspeedidx][preampgainidx]
		    << " status "           << drvdef(gotstat)  << endl;
		    if (gotstat != DRV_SUCCESS) {
			numfails++;
		    }

		    gotstat = IsPreAmpGainAvailable( adchanidx, outampidx, hsspeedidx, preampgainidx,
		    &preampgainavail[adchanidx][outampidx][hsspeedidx][preampgainidx]);
		    cout
		    << " preampgainavail "      << preampgainavail[adchanidx][outampidx][hsspeedidx][preampgainidx]
		    << " status "               << drvdef(gotstat) << endl;
		    if (gotstat != DRV_SUCCESS) {
			numfails++;
		    }

		} /* end for preampgainidx */

	    } /* end for hsspeedidx */

	    // reset back to default HS Speed
	    hsspeedidx  = 0;
	    gotstat = SetHSSpeed(outampidx, hsspeedidx);
	    cout
	    << "SetHSSpeed(" << outampidx << "," << curhsspeedidx << ")"
	    << " status " << drvdef(gotstat) << endl;
	    if (gotstat == DRV_SUCCESS) {
		curoutampidx    = outampidx;
		curhsspeedidx   = hsspeedidx;
	    } else {
		numfails++;
	    }

	} /* end for outampidx */

	// reset back to Conventional keep the EMCCD amplifier safer from too much light
	outampidx   = OutputAmpConventional;
	gotstat     = SetOutputAmplifier(curoutampidx);
	cout
	<< "SetOutputAmplifier " << curoutampidx
	<< " status " << drvdef(gotstat) << endl;
	if (gotstat == DRV_SUCCESS) {
	    curoutampidx = outampidx;
	} else {
	    numfails++;
	}

    } /* end for adchanidx */

    cout << "GetCapabilities()...";
    andorcaps.ulSize = sizeof(AndorCapabilities);
    gotstat = GetCapabilities(&andorcaps);
    cout << " status " << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    cout << "CameraType=" << andorcaps.ulCameraType
    << " " << acCameratype(andorcaps.ulCameraType) << endl;

    fprintf(stdout, "ReadModes=0x%x %s\n", andorcaps.ulReadModes,
    acReadModes(andorcaps.ulReadModes));

    fprintf(stdout, "EMGainCapability = 0x%0x\n", andorcaps.ulEMGainCapability);

    // Get Detector dimensions
    // ASSUME that this cannot change at runtime
    cout << "Detector size";
    gotstat     = GetDetector(&maxhend, &maxvend);
    cout
    << " maxhend="      << maxhend
    << " maxvend="      << maxvend
    << " status "   << drvdef(gotstat) << endl;
    if (gotstat == DRV_SUCCESS) {
	curhend         = maxhend;
	curvend         = maxvend;
    } else {
	numfails++;
    }

    // end of queries that are part of initialization
    /***********************************************************/

    //Set Read Mode to --Image--
    readmode    = ReadModeImage;
    gotstat     = SetReadMode(readmode);
    fprintf(stdout, "SetReadMode(%d) status %s\n",
    readmode, drvdef(gotstat));
    if (gotstat == DRV_SUCCESS) {
	curreadmode = readmode;
    } else {
	numfails++;
    }

    //Set Acquisition mode to SingleScan
    gotstat = SetAcquisitionMode(AcquireModeSingleScan);
    fprintf(stdout,
    "SetAcquisitionMode(mode=%d) status %s\n",
    AcquireModeSingleScan, drvdef(gotstat)));
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    } else {
	curacqmode = AcquireModeSingleScan;
    }

    //Initialize Shutter to be closed for sake of safety
    // WHY are we setting delays to 50 here and not later?
    gotstat = SetShutter(ShutterTTLopenHi, ShutterModePermClos,50,50);
    cout
    << "SetShutter(ShutterTTLopenHi, ShutterModePermClos,50,50)"
    << " status "       << drvdef(gotstat) << endl;
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    } else {
	curshutmode = ShutterModePermClos;
    }

    // WHY are we setting gain mode to 1?
    // manual says mode 1 means DAC settings range 0/4095
    gotstat = SetEMGainMode(1);
    cout
    << "SetEMGainMode(1)"
    << " status "       << drvdef(gotstat);
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    } else {
	curemgainmode = 1;
    }

    etime = 0.1;
    //Set initial exposure time
    gotstat = SetExposureTime(etime);
    cout
    << "SetExposureTime(" << etime << ")"
    << " status "       << drvdef(gotstat);
    if (gotstat != DRV_SUCCESS) {
	numfails++;
    }

    if (numfails) {
	cout << numfails << "failures occurred during initialization" << endl;
    }

    // main interactive loop with person at terminal
    quit = false;
    do{
	if (gotstat == DRV_SUCCESS) {
	    cout << endl << "------------------------------------------------------" << endl;
	} else if (gotstat == DRV_notPassed) {
	    cout << endl << "NOT PASSED  NOT PASSED  NOT PASSED  NOT PASSED  NOT PASSED  NOT PASSED" << endl;
	} else {
	    cout << endl << "FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL FAIL " << endl;
	}

	cout << " numframes=" << numframes << endl;

	cout << "Timings";
	gotstat = GetAcquisitionTimings(&etime, &atime, &ktime);
	cout
	<< " exp="      << etime
	<< " acc="      << atime
	<< " kin="      << ktime
	<< " status "   << drvdef(gotstat)
	<< endl;

	/*
	Rather than poll continously, process sleep during exposure.
	Sleep process for 1/10 of the expected exposure time,
	so we will poll only about 10 times during exposure.
	*/
	sleepus     = etime * (MUSECperSEC / 10);
	// do not sleep longer than 0.1 s
	if (sleepus > 100000)  sleepus  = 100000;

	gotstat = GetTemperature(&ctemp);
	cout
	<< " current temp="     << ctemp
	<< " status "           << drvdef(gotstat) << endl;

	gotstat = GetTemperatureStatus(&stemp,&ttemp,&atemp,&cvolt);
	cout
	<< " sensor temp="      << stemp
	<< " target temp="      << ttemp
	<< " ambienttemp="      << atemp
	<< " coolervolts="      << cvolt
	<< " status "           << drvdef(gotstat) << endl;

	cout << "Current target temperature " << curttemp << endl;
	cout << "Current shutter mode  " << curshutmode << endl;
	cout << "Current EM gain mode  " << curemgainmode << endl;
	cout << "Current IsoCropMode   " << curisocropactv << endl;
	cout << "Current IsoCropModeEx " << curisocropexactv << endl;

	gotstat = GetHSSpeed(curadchanidx, curoutampidx, curhsspeedidx,
	&curhsspeed);
	cout
	<< "hsspeed"
	<< "[curadchanidx="     << curadchanidx         << "]"
	<< "[curoutampidx="     << curoutampidx         << "]"
	<< "[curhsspeedidx="    << curhsspeedidx        << "]"
	<< " " << curhsspeed << " MHz "
	<< " status " << drvdef(gotstat) << endl;

	gotstat = GetEMAdvanced(&curemadvgain);
	cout
	<< " current Advanced gain value="      << curemadvgain
	<< " status "                   << drvdef(gotstat) << endl;

	gotstat = GetEMCCDGain(&curemccdgain);
	cout << " current EMCCD gain value=" << curemccdgain
	<< " status " << drvdef(gotstat) << endl;

	gotstat = GetEMGainRange(&emgainlo, &emgainhi);
	cout << "EMCCD gain range"
	<< " lo=" << emgainlo
	<< " hi=" << emgainhi
	<< " status " << drvdef(gotstat) << endl;

	//Setup Image dimensions
	// this can be used to define a window/ROI
	gotstat = SetImage(curhbin,curvbin,curhbeg,curhend,curvbeg,curvend);
	fprintf(stdout, "menu SetImage(hbin=%d, vbin=%d, hbeg=%d, hend=%d, vbeg=%d, vend=%d)",
	curhbin, curvbin, curhbeg, curhend, curvbeg, curvend );
	cout << " status " << drvdef(gotstat) << endl;

	//Show menu options
	cout << "        Menu" << endl;
	cout << "====================" << endl;
	cout << "a. Start Acquisition" << endl;
	cout << "e. Set Exposure Time" << endl;
	cout << "n. Set number frames" << endl;
	cout << "h. Set hsspeed"       << endl;
	cout << "t. Set Temperature  " << endl;
	cout << "m. Get Temperature  " << endl;
	cout << "f. Cooler On        " << endl;
	cout << "w. Cooler Off       " << endl;
	cout << "g. EM gain mode     " << endl;
	cout << "v. EM gain value    " << endl;
	cout << "p. Set Output Amp   " << endl;
	cout << "c. Shutter Closed   " << endl;
	cout << "o. Shutter Open     " << endl;
	cout << "s. Shutter Auto     " << endl;
	cout << "b. binning          " << endl;
	cout << "r. region/window    " << endl;
	cout << "i. isolated crop    " << endl;
	cout << "x. isolated crop EX " << endl;
	cout << "z.     Exit         " << endl;
	cout << "====================" << endl;
	cout << "Choice?::";
	//menu choice: get single character
	cin >> choice;
	// consume any other input
	cin.getline(inputline, sizeof(inputline));

	prevstat = DRV_nosuch;

	switch(choice){
	case 'a': //expose
	    {
		// time of begin of sequence of exposures
		time_t  sbegtime;

		fprintf(stdout, "StartAcquisition %d:%d:%d,%d:%d:%d\n",
		curhbeg, curhend, curhbin,
		curvbeg, curvend, curvbin);

		// we want to name all files in sequence with same start time
		sbegtime        = time(NULL);   /* sequence begin for file names */

		// loop to take this many frames
		for (int nf = 0; nf < numframes; nf++) {

		    time_t currtime;
		    time_t begtime      = time(NULL);   /* DATE-BEG */
		    time_t prevtime     = begtime;

		    StartAcquisition();

		    cout << "Started " << etime << " second exposure..." << endl;

		    int     waiting = 1;

		    int     numwait = 0;
		    //Loop until acquisition finished
		    while (waiting)
		    {
			numwait++;
			gotstat   = GetStatus(&currstat);

			switch(gotstat) {
			    case DRV_SUCCESS:
				if (currstat != prevstat) {
				    cout
				    << endl
				    << "current status "        << drvdef(currstat)
				    << '\r' << flush;
				    prevstat = currstat;
				}
				break;
			    default:
				// this is probably a fail
				cout
				<< endl
				<< "exposure GetStatus() "
				<< drvdef(gotstat) << endl;
				// end the exposure loop
				waiting = 0;
				break;
			} /* end switch gotstat */

			switch(currstat) {
			    case DRV_ACQUIRING:
				//  GetStatus() said we are still exposing
				currtime = time(NULL);
				if ((currtime - prevtime) >= 1)
				{
				    // tick off seconds of exposure
				    time_t exptime = currtime - begtime;
				    cout
				    << " current status "       << drvdef(currstat)
				    << " " << exptime
				    << '\r' << flush;
				    prevtime = currtime;
				}
				// do not poll GetStatus() at max possible rate
				// let the processor rest a bit
				usleep(sleepus);
				break;
			    default:
				// end the exposure loop
				waiting = 0;
				break;
			} /* end switch currstat */

		    } /* end while waiting */

		    // newline after waiting phase
		    cout <<  endl;

		    // construct FITS file name using begin time of sequence
		    gmtime_r( &sbegtime, &tm);
		    strftime( fitsfname, sizeof(fitsfname), "./im%Y%m%dT%H%M%Sn", &tm);
		    newint = strlen(fitsfname);
		    // append sequence number to FITS file name
		    snprintf( fitsfname+newint, sizeof(fitsfname)-newint, "%06d.fits", nf);

		    // use Andor-supplied FITS file writer
		    // this writes FITS header keyword=value lines for many internal parameters
		    // not clear if all of those internal parameters are otherwise available
		    // note that this gives us no control over FITS header cards
		    cout << "SaveAsFITS " << fitsfname << " ...";
		    gotstat = SaveAsFITS(fitsfname, UNS16);
		    if (gotstat != DRV_SUCCESS)
		    {
			cout
			<< "FAIL SaveAsFITS"
			<< " status "   << drvdef(gotstat) << endl;
		    }

		    cout << "done" << endl;

		} /* end for nf */

	    } /* end case a */

	    break;

	case 'e': //Set new exposure time

	    cout << endl << "Enter new Exposure Time(s)::";
	    cin.getline(inputline, sizeof(inputline));
	    cout << "inputline<:" << inputline << ":>" << endl;
	    howmany = sscanf(inputline, "%f", &newfloat);
	    if (howmany < 1) {
		cout << "Need a floating point value for exposure" << endl;
	    } else if (newfloat < 0.) {
		cout << "exposure time cannot be negative" << endl;
	    } else {
		gotstat = SetExposureTime(newfloat);
		cout
		<< "SetExposureTime()"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    etime = newfloat;
		}
	    }
	    break;

	case 'n': //Set numframes
	    cout << endl << "Enter number of frames::";
	    cin.getline(inputline, sizeof(inputline));
	    sscanf(inputline, "%d", &newint);
	    if (newint < 0) {
		cout << "numframes must be positive" << endl;
	    } else if (newint > MAXFRAME) {
		cout << "numframes must not be greater than " << MAXFRAME << endl;
	    } else {
		numframes = newint;
	    }

	    break;

	case 'b': //Set binning
	    //  ultimately we must use PANE so that binning is independent of window
	    maxvbin = maxhbin = -1;
	    gotstat = GetMaximumBinning (curreadmode, GEThbin, &newint);
	    if (gotstat != DRV_SUCCESS) {
		cout
		<< "Failed to get max h bin "
		<< drvdef(gotstat) << endl;
	    } else {
		maxhbin = newint;
	    }
	    gotstat = GetMaximumBinning (curreadmode, GETvbin, &newint);
	    if (gotstat != DRV_SUCCESS) {
		cout
		<< "Failed to get max v bin "
		<< drvdef(gotstat) << endl;
	    } else {
		maxvbin = newint;
	    }
	    if (maxhbin < 1 || maxvbin < 1) {
		gotstat = DRV_notPassed;
		break;
	    }

	    cout << endl;
	    cout << "Max H (x) bin " << maxhbin << endl;
	    cout << "Max V (y) bin " << maxvbin << endl;
	    cout << "Enter xbin ybin::";
	    newhbin = newvbin = -1;
	    cin.getline(inputline, sizeof(inputline));
	    howmany = sscanf(inputline, "%d %d", &newhbin, &newvbin);
	    if (howmany != 2) {
		cout << "we need 2 input values to set binning" << endl;
		gotstat = DRV_notPassed;
		break;
	    } else if (newhbin < 1 || newvbin < 1) {
		cout << "binning must be >= 1" << endl;
		gotstat = DRV_notPassed;
		break;
	    } else if (newhbin > maxhbin || newvbin > maxvbin) {
		cout << "binning must not exceed max allowed" << endl;
		gotstat = DRV_notPassed;
		break;
	    }

	    // note we do not know how binning affects begin pixel and extent
	    // we need a routine that recalculates those consistently
	    gotstat = SetImage(newhbin,newvbin,curhbeg,curhend,curvbeg,curvend);
	    fprintf(stdout, "bin SetImage(hbin=%d, vbin=%d, hbeg=%d, hend=%d, vbeg=%d, vend=%d)",
	    curhbin, curvbin, curhbeg, maxhend, curvbeg, maxvend );
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat == DRV_SUCCESS)
	    {
		curhbin = newhbin;
		curvbin = newvbin;
	    }

	    break;

	case 'r': //Set region/window
	    // we do not know whether last 4 args to SetImage are binned pixels or not
	    // we do not know whether pixel counts are 0-indexed or 1-indexed
	    // we may have to mount a lens to get an actual image in order to know that
	    //  ultimately we must use PANE so that binning is independent of window
	    // this also needs consistent handling with binning
	    // maybe best if we define PANE coordinates as in the Keck detectors

	    // looks like Andor sdk requires (begpix -1)%bin = 0 else it says bad binning value
	    // looks like Andor sdk allows endpix as large as maxpix without crash
	    // looks like Andor sdk hangs if endpix < begpix
	    // looks like Andor sdk hangs if endpix > maxpix

	    cout << "Enter xcolbeg xcolend yrowbeg yrowend::";
	    newhbeg   = newvbeg   = -1;
	    newhend   = newvend   = -1;
	    cin.getline(inputline, sizeof(inputline));
	    howmany = sscanf(inputline, "%d %d %d %d", &newhbeg, &newhend, &newvbeg, &newvend);
	    dobreak = 0;
	    if (howmany != 4) {
		cout << "we need 4 input values to set ROI window" << endl;
		dobreak++;
	    }
	    if (newhbeg < INIPIX) {
		cout << "h begin pixel must be >= " << INIPIX << endl;
		dobreak++;
	    }
	    if (newvbeg < INIPIX) {
		cout << "v begin pixel must be >= " << INIPIX << endl;
		dobreak++;
	    }
	    if (newhend < newhbeg) {
		cout << "h end pixel must be >= begin pixel" << endl;
		dobreak++;
	    }
	    if (newvend < newvbeg) {
		cout << "v end pixel must be >= begin pixel" << endl;
		dobreak++;
	    }
	    /*
	    if (newhend > maxhend) {
		cout << "h end pixel must be <= " << maxhend << endl;
		dobreak++;
	    }
	    if (newvend > maxvend) {
		cout << "v end pixel must be <= " << maxvend << endl;
		dobreak++;
	    }
	    */
	    if (dobreak) {
		gotstat = DRV_notPassed;
		break;
	    }

	    // note we do not know how binning affects begin and extent of region
	    // we need a routine that recalculates those consistently
	    gotstat = SetImage(curhbin,curvbin,newhbeg,newhend,newvbeg,newvend);
	    fprintf(stdout, "rw SetImage(hbin=%d, vbin=%d, hbeg=%d, hend=%d, vbeg=%d, vend=%d)",
	    curhbin, curvbin, newhbeg, newhend, newvbeg, newvend );
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat == DRV_SUCCESS)
	    {
		curhbeg         = newhbeg;
		curhend         = newhend;
		curvbeg         = newvbeg;
		curvend         = newvend;
	    }

	    break;

	case 'i': //Set isolated crop mode NOT EX

	    cout << "Enter isocropactv isocropvsiz isocrophsiz::";
	    newisocropactv      = -1;
	    newisocropvsiz      = newisocrophsiz        = -1;
	    cin.getline(inputline, sizeof(inputline));
	    howmany = sscanf(inputline, "%d %d %d",
	    &newisocropactv,
	    &newisocropvsiz, &newisocrophsiz);
	    dobreak = 0;
	    if (howmany != 3) {
		cout << "we need 3 input values to set IsoCropMode" << endl;
		dobreak++;
	    } else if (newisocropactv == 0) {
		// just turn off while keeping other params the same
		newisocropvsiz    = curisocropvsiz;
		newisocrophsiz    = curisocrophsiz;
	    } else if (newisocropactv == 1) {
		if (newisocrophsiz < 1) {
		    cout << "h size must be >= 1" << endl;
		    dobreak++;
		}
		if (newisocropvsiz < 1) {
		    cout << "v size must be >= 1" << endl;
		    dobreak++;
		}
		// there are probably more gotcha values to test
	    } else {
		cout << "actv must be 0 or 1" << endl;
		dobreak++;
	    }
	    if (dobreak) {
		gotstat = DRV_notPassed;
		break;
	    }

	    // according to Andor reps we must do these ...
	    //Set Acquisition mode to Kinetics
	    gotstat = SetAcquisitionMode(AcquireModeKinetics);
	    fprintf(stdout,
	    "SetAcquisitionMode(mode=%d)",
	    AcquireModeKinetics);
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat != DRV_SUCCESS) {
		break;
	    } else {
		curacqmode = AcquireModeKinetics;
	    }
	    //Set Frame Transfer mode On
	    gotstat = SetFrameTransferMode(FrameTransferModeOn);
	    fprintf(stdout,
	    "SetFrameTransferMode(mode=%d)",
	    FrameTransferModeOn);
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat != DRV_SUCCESS) {
		break;
	    } else

	    // ... before we do this
	    gotstat = SetIsolatedCropMode(
	    newisocropactv,
	    newisocropvsiz, newisocrophsiz,
	    curvbin, curhbin);
	    fprintf(stdout,
	    "SetIsolatedCropMode(actv=%d, vsiz=%d, hsiz=%d, vbin=%d, hbin=%d)",
	    newisocropactv,
	    newisocropvsiz, newisocrophsiz,
	    curvbin, curhbin);
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat == DRV_SUCCESS)
	    {
		curisocropactv    = newisocropactv;
		curisocropvsiz    = newisocropvsiz;
		curisocrophsiz    = newisocrophsiz;
	    }

	    break;

	case 'x': //Set isolated crop mode Ex
	    // Andor sdk manual page 290 gives table of optimal windows for iXon Ultra 897

	    if (curoutampidx != OutputAmpElectronMult) {
		cout << "Please choose EM amplifier before setting this" << endl;
		cout << "IsolatedCropModeEx wants EM amplifier" << endl;
		gotstat = DRV_notPassed;
		break;
	    }

	    cout << "Enter isocropactv isocropexvsiz isocropexhsiz isocropexhbeg isocropexvbeg::";
	    newisocropexactv    = -1;
	    newisocropexvsiz    = newisocropexhsiz      = -1;
	    newisocropexhbeg    = newisocropexvbeg      = -1;
	    cin.getline(inputline, sizeof(inputline));
	    howmany = sscanf(inputline, "%d %d %d %d %d",
	    &newisocropexactv,
	    &newisocropexvsiz, &newisocropexhsiz,
	    &newisocropexhbeg, &newisocropexvbeg);
	    dobreak = 0;
	    if (howmany != 5) {
		cout << "we need 5 input values to set IsoCropModeEx" << endl;
		dobreak++;
	    } else if (newisocropexactv == 0) {
		// just turn off while keeping other params the same
		newisocropexvsiz    = curisocropexvsiz;
		newisocropexhsiz    = curisocropexhsiz;
		newisocropexvbeg    = curisocropexvbeg;
		newisocropexhbeg    = curisocropexhbeg;
	    } else if (newisocropexactv == 1) {

		if (newisocropexvbeg < INIPIX) {
		    cout << "v begin pixel must be >= " << INIPIX << endl;
		    dobreak++;
		}
		if (newisocropexhbeg < INIPIX) {
		    cout << "h begin pixel must be >= " << INIPIX << endl;
		    dobreak++;
		}
		if (newisocropexhsiz < 1) {
		    cout << "h size must be >= 1" << endl;
		    dobreak++;
		}
		if (newisocropexvsiz < 1) {
		    cout << "v size must be >= 1" << endl;
		    dobreak++;
		}
		// there are probably more gotcha values to test
	    } else {
		cout << "actv must be 0 or 1" << endl;
		dobreak++;
	    }
	    if (dobreak) {
		gotstat = DRV_notPassed;
		break;
	    }

	    gotstat = SetIsolatedCropModeEx(
	    newisocropexactv,
	    newisocropexvsiz, newisocropexhsiz,
	    curvbin, curhbin,
	    newisocropexhbeg, newisocropexvbeg);
	    fprintf(stdout,
	    "SetIsolatedCropModeEx(actv=%d, vsiz=%d, hsiz=%d, vbin=%d, hbin=%d, hbeg=%d, vbeg=%d)",
	    newisocropexactv,
	    newisocropexvsiz, newisocropexhsiz,
	    curvbin, curhbin,
	    newisocropexhbeg, newisocropexvbeg);
	    cout << " status " << drvdef(gotstat) << endl;
	    if (gotstat == DRV_SUCCESS)
	    {
		curisocropexactv    = newisocropexactv;
		curisocropexvsiz    = newisocropexvsiz;
		curisocropexhsiz    = newisocropexhsiz;
		curisocropexvbeg    = newisocropexvbeg;
		curisocropexhbeg    = newisocropexhbeg;
	    }

	    break;

	case 'h': //set HS speed
	    cout << endl;
	    cout << "Current HS speed index: " << curhsspeedidx << endl;
	    // remind user what speeds were reported as possible
	    cout << "Available HS speeds are:" << endl;
	    for (hsspeedidx=0; hsspeedidx < numhsspeed[curadchanidx][curoutampidx]; hsspeedidx++) {
		cout << hsspeedidx << ": " << hsspeed[curadchanidx][curoutampidx][hsspeedidx] << " MHz " << endl;
	    }

	    cout << endl << "HS speed index::";
	    newint      = -1;
	    cin.getline(inputline, sizeof(inputline));
	    sscanf(inputline, "%d", &newint);
	    if (newint < 0) {
		cout << "HS speed index cannot be negative" << endl;
	    } else if (newint >= numhsspeed[curadchanidx][curoutampidx]) {
		cout
		<< "HS speed index must be less than "
		<< numhsspeed[curadchanidx][curoutampidx] << endl;
	    } else {
		gotstat = SetHSSpeed(curoutampidx, newint);
		cout
		<< "SetHSSpeed"
		<< " " << curoutampidx
		<< " " << curhsspeedidx
		<< drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curhsspeedidx  = newint;
		    cout
		    << "should give hsspeed"
		    << "[curadchanidx="     << curadchanidx         << "]"
		    << "[curoutampidx="     << curoutampidx         << "]"
		    << "[curhsspeedidx="    << curhsspeedidx        << "]"
		    << " " << hsspeed[curadchanidx][curoutampidx][curhsspeedidx] << " MHz "
		    << endl;
		}
	    }
	    break;

	case 'p': //Set output amplifier
	    cout << endl;
	    cout << "Output Amp (EM = 0, conv = 1)::";
	    newint      = -1;
	    cin.getline(inputline, sizeof(inputline));
	    sscanf(inputline, "%d", &newint);
	    if (newint < 0) {
		cout << "outamp index cannot be negative" << endl;
	    } else if (newint >= numamp) {
		cout << "outamp index must be less than " << numamp << endl;
	    } else {
		gotstat = SetOutputAmplifier(newint);
		cout
		<< "SetOutputAmplifier " << newint << " "
		<< drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    // if new amp is not EM we must turn off IsolatedCropEx
		    if (curisocropexactv && (curoutampidx != OutputAmpElectronMult)) {
			gotstat = SetIsolatedCropModeEx(0,
			curisocropexvsiz,curisocropexhsiz,
			curvbin,curhbin,
			curisocropexhbeg, curisocropexvbeg);
			cout
			<< "SetIsolatedCropModeEx() "
			<< drvdef(gotstat) << endl;
		    }

		    curoutampidx = newint;
		}
	    }
	    break;

	case 't': //for entering target temperature
	    {
		cout << "Enter the Required temperature" << endl;
		cout << ":";
		itemp   = InvalidTemp;
		cin.getline(inputline, sizeof(inputline));
		sscanf(inputline, "%d", &itemp);
		if (itemp == InvalidTemp)
		{
		    cout << itemp << "is not a valid temperature" << endl;
		    gotstat = DRV_notPassed;
		    break;
		}
		gotstat = SetTemperature(itemp);
		if (gotstat == DRV_SUCCESS)
		{
		    curttemp    = itemp;
		}
	    } /* end case t */
	    break;

	case 'm': //for getting the current state of the cooler
	    {
		gotstat = GetTemperature(&ctemp);
		cout
		<< "Current Temperature: " << ctemp
		<< " status "   << drvdef(gotstat) << endl;
	    } /* end case m */
	    break;

	case 'f': // switch on the cooler
	    {
		gotstat = CoolerON();
		cout
		<< "CoolerON()"
		<< " status "   << drvdef(gotstat) << endl;
	    }
	    break;
	case 'w': // switch off the cooler
	    {
		gotstat = CoolerOFF();
		cout
		<< "CoolerOFF()"
		<< " status "   << drvdef(gotstat) << endl;
	    }
	    break;

	case 'g': // select EM gain mode
	    {
		int     emgainmode;

		// WHAT are the valid values?
		cout << "Enter the EM Gain mode: ";
		cin.getline(inputline, sizeof(inputline));
		sscanf(inputline, "%d", &emgainmode);
		gotstat = SetEMGainMode(emgainmode);
		cout
		<< "SetEMGainMode()"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curemgainmode       = emgainmode;
		}
	    }
	    break;

	case 'v': // set EM gain value
	    {
		int     emgval;

		// valid value limits should be emgainlo emgainhi
		cout << "Enter the EM Gain value: ";
		cin.getline(inputline, sizeof(inputline));
		sscanf(inputline, "%d", &emgval);
		gotstat = SetEMCCDGain(emgval);
		cout
		<< "SetEMCCDGain()"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curemccdgain    = emgval;
		}
	    }
	    break;

	case 'c':
	    {
		// set the shutter mode permanently closed
		gotstat = SetShutter(ShutterTTLopenLo, ShutterModePermClos, 0, 0);
		cout
		<< "SetShutter(closed)"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curshutmode = ShutterModePermClos;
		}
	    } /* end case c */
	    break;
	case 'o':
	    {
		// set the shutter mode permanently open
		gotstat = SetShutter(ShutterTTLopenLo, ShutterModePermOpen, 0, 0);
		cout
		<< "SetShutter(open)"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curshutmode = ShutterModePermOpen;
		}
	    } /* end case o */
	    break;
	case 's':
	    {
		// set the shutter mode auto
		gotstat = SetShutter(ShutterTTLopenLo, ShutterModeFullAuto, 0, 0);
		cout
		<< "SetShutter(auto)"
		<< " status "   << drvdef(gotstat) << endl;
		if (gotstat == DRV_SUCCESS)
		{
		    curshutmode = ShutterModeFullAuto;
		}
	    } /* end case s */
	    break;

	case 'z': //Exit
	    quit = true;
	    break;

	default:
	    cout << "!Invalid Option!" << endl;
	}

    } while(!quit);

    cout << "ShutDown" << endl;

    gotstat   = GetStatus(&currstat);
    cout << "GetStatus() "      << drvdef(gotstat)  << endl;
    cout << "current status "   << drvdef(currstat) << endl;

    //Shut down CCD
    ShutDown();

    cout << "Return" << endl;

    return SUCCESS;
} /* end main() */

