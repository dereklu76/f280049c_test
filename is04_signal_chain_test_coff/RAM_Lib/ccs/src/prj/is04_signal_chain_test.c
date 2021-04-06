//#############################################################################
// $TI Release: MotorControl SDK v3.01.00.00 $
// $Release Date: Mon Jan 11 11:23:03 CST 2021 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//! \file   solutions/common/sensorless_foc/source/is04_signal_chain_test.c
//! \brief  This lab is used to implements a scalar current closed loop
//!         control to test the signal chain integrity of the hardware
//!

// **************************************************************************

//
// solutions
//
#include "labs.h"

#pragma CODE_SECTION(mainISR, ".TI.ramfunc");

//
// the globals
//
HAL_ADCData_t adcData = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0};

HAL_PWMData_t pwmData = {{0.0, 0.0, 0.0}};

uint16_t counterLED = 0;  //!< Counter used to divide down the ISR rate for
                           //!< visually blinking an LED

uint16_t counterSpeed = 0;
uint16_t counterTrajSpeed = 0;
uint16_t counterTrajId = 0;

uint32_t offsetCalcCount = 0;      //!< Counter used to count the wait time
                                   //!< for offset calibration, unit: ISR cycles

uint32_t offsetCalcWaitTime = 50000;  //!< Wait time setting for current/voltage
                                      //!< offset calibration, unit: ISR cycles

EST_InputData_t estInputData = {0, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0};

EST_OutputData_t estOutputData = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  {0.0, 0.0}, {0.0, 0.0}, 0, 0.0};
float32_t angleDelta_rad;   //!< the rotor angle compensation value
float32_t angleFoc_rad;     //!< the rotor angle for FOC modules
float32_t angleEst_rad;     //!< the rotor angle from FAST estimator

MATH_Vec2 Idq_ref_A;        //!< the reference current on d&q rotation axis
MATH_Vec2 Idq_offset_A;     //!< the offsetting current on d&q rotation axis
MATH_Vec2 Iab_in_A;         //!< the alpha&beta axis current are converter from
                            //!< 3-phase sampling input current of motor
MATH_Vec2 Idq_in_A;         //!< the d&q axis current are converter from
                            //!< 3-phase sampling input current of motor
MATH_Vec2 Vab_out_V;        //!< the output control voltage on alpha&beta axis
MATH_Vec2 Vdq_out_V;        //!< the output control voltage on d&q axis

USER_Params userParams;    //!< the user parameters for motor control
                            //!< and hardware board configuration
#pragma DATA_SECTION(userParams, "ctrl_data");

volatile MOTOR_Vars_t motorVars = MOTOR_VARS_INIT;
#pragma DATA_SECTION(motorVars, "ctrl_data");

CTRL_Handle   ctrlHandle;       //!< the handle for the controller
CTRL_Obj      ctrl;             //!< the controller object

CLARKE_Handle clarkeHandle_I;   //!< the handle for the current Clarke transform
CLARKE_Obj    clarke_I;         //!< the current Clarke transform object

CLARKE_Handle clarkeHandle_V;   //!< the handle for the voltage Clarke transform
CLARKE_Obj    clarke_V;         //!< the voltage Clarke transform object

EST_Handle    estHandle;        //!< the handle for the estimator

HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
HAL_Obj       hal;            //!< the hardware abstraction layer object

IPARK_Handle  iparkHandle;      //!< the handle for the inverse Park transform
IPARK_Obj     ipark;            //!< the inverse Park transform object

PARK_Handle   parkHandle_I;     //!< the handle for the Park object
PARK_Obj      park_I;           //!< the Park transform object

PI_Handle     piHandle_Id;      //!< the handle for the Id PI controller
PI_Obj        pi_Id;            //!< the Id PI controller object

PI_Handle     piHandle_Iq;      //!< the handle for the Iq PI controller
PI_Obj        pi_Iq;            //!< the Iq PI controller object

PI_Handle     piHandle_fwc;     //!< the handle for the Iq PI controller
PI_Obj        pi_fwc;           //!< the Iq PI controller object

PI_Handle     piHandle_spd;     //!< the handle for the speed PI controller
PI_Obj        pi_spd;           //!< the speed PI controller object

SVGEN_Handle  svgenHandle;      //!< the handle for the space vector generator
SVGEN_Obj     svgen;            //!< the space vector generator object

TRAJ_Handle   trajHandle_spd; //!< the handle for the speed reference trajectory
TRAJ_Obj      traj_spd;       //!< the speed reference trajectory object

TRAJ_Handle   trajHandle_Id;    //!< the handle for the id reference trajectory
TRAJ_Obj      traj_Id;          //!< the id reference trajectory object

TRAJ_Handle   trajHandle_fwc;   //!< the handle for the id reference trajectory
TRAJ_Obj      traj_fwc;         //!< the id reference trajectory object

//!< the handles for the current offset calculation
FILTER_FO_Handle  filterHandle_I[USER_NUM_CURRENT_SENSORS];
//!< the current offset calculation
FILTER_FO_Obj     filter_I[USER_NUM_CURRENT_SENSORS];

//!< the handles for the voltage offset calculation
FILTER_FO_Handle  filterHandle_V[USER_NUM_VOLTAGE_SENSORS];
//!< the voltage offset calculation
FILTER_FO_Obj     filter_V[USER_NUM_VOLTAGE_SENSORS];

//!< the handles for Angle Generate for open loop control
ANGLE_GEN_Handle angleGenHandle;
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen;

//!< define the current setting value for current close loop
MATH_Vec2 IdqSet_A = {0.0, 0.75};

#ifdef DRV8320_SPI
//
// Watch window interface to the 8320 SPI
//
DRV8320_SPIVars_t drvSPI8320Vars;
#pragma DATA_SECTION(drvSPI8320Vars, "ctrl_data");
#endif

#ifdef PWMDAC_ENABLE
HAL_PWMDACData_t pwmDACData;
#pragma DATA_SECTION(pwmDACData, "ctrl_data");
#endif


//
// the functions
//

void main(void)
{
    uint16_t estNumber = 0;
    bool flagEstStateChanged = false;

#ifdef _HVKIT_REV1p1_
    motorVars.boardKit = BOARD_HVMTRPFC_REV1P1;
#endif // _HVKIT_REV1p1_

#ifdef _DRV8301_KIT_REVD_
    motorVars.boardKit = BOARD_DRV8301_REVD;
#endif  // _DRV8301_KIT_REVD_

#ifdef _BOOSTXL_8320RS_REVA_
    motorVars.boardKit = BOARD_BSXL8320RS_REVA;
#endif  // _BOOSTXL_8320RS_REVA_

    //
    // initialize the user parameters
    //
    USER_setParams(&userParams);

    userParams.flag_bypassMotorId = true;

    //
    // initialize the user parameters
    //
    USER_setParams_priv(&userParams);

    //
    // initialize the driver
    //
    halHandle = HAL_init(&hal, sizeof(hal));

    //
    // set the driver parameters
    //
    HAL_setParams(halHandle);

    //
    // initialize the Clarke modules
    //
    clarkeHandle_I = CLARKE_init(&clarke_I, sizeof(clarke_I));
    clarkeHandle_V = CLARKE_init(&clarke_V, sizeof(clarke_V));

    //
    // set the Clarke parameters
    //
    setupClarke_I(clarkeHandle_I, userParams.numCurrentSensors);
    setupClarke_V(clarkeHandle_V, userParams.numVoltageSensors);

    //
    // initialize the estimator
    //
    estHandle = EST_initEst(estNumber);

    //
    // set the default estimator parameters
    //
    EST_setParams(estHandle, &userParams);
    EST_setFlag_enableForceAngle(estHandle, motorVars.flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(estHandle, motorVars.flagEnableRsRecalc);

    // set the scale factor for high frequency motor
    EST_setOneOverFluxGain_sf(estHandle, &userParams, USER_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(estHandle, &userParams, USER_EST_FREQ_HF_SF);
    EST_setBemf_sf(estHandle, &userParams, USER_EST_BEMF_HF_SF);

    //
    // if motor is an induction motor, configure default state of PowerWarp
    //
    if(userParams.motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_enablePowerWarp(estHandle, motorVars.flagEnablePowerWarp);
        EST_setFlag_bypassLockRotor(estHandle, motorVars.flagBypassLockRotor);
    }

    //
    // initialize the inverse Park module
    //
    iparkHandle = IPARK_init(&ipark, sizeof(ipark));

    //
    // initialize the Park module
    //
    parkHandle_I = PARK_init(&park_I, sizeof(park_I));

    //
    // initialize the PI controllers
    //
    piHandle_Id  = PI_init(&pi_Id, sizeof(pi_Id));
    piHandle_Iq  = PI_init(&pi_Iq, sizeof(pi_Iq));
    piHandle_fwc = PI_init(&pi_fwc, sizeof(pi_fwc));
    piHandle_spd = PI_init(&pi_spd, sizeof(pi_spd));

    //
    // setup the controllers, speed, d/q-axis current pid regulator
    //
    setupControllers();

    //
    // initialize the space vector generator module
    //
    svgenHandle = SVGEN_init(&svgen, sizeof(svgen));

    //
    // initialize the speed reference trajectory
    //
    trajHandle_spd = TRAJ_init(&traj_spd, sizeof(traj_spd));

    //
    // configure the speed reference trajectory (Hz)
    //
    TRAJ_setTargetValue(trajHandle_spd, 0.0);
    TRAJ_setIntValue(trajHandle_spd, 0.0);
    TRAJ_setMinValue(trajHandle_spd, -USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxValue(trajHandle_spd, USER_MOTOR_FREQ_MAX_HZ);
    TRAJ_setMaxDelta(trajHandle_spd, (USER_MAX_ACCEL_Hzps / USER_ISR_FREQ_Hz));

    //
    // initialize the Id reference trajectory
    //
    trajHandle_Id = TRAJ_init(&traj_Id, sizeof(traj_Id));

    //
    // configure the Id reference trajectory
    //
    TRAJ_setTargetValue(trajHandle_Id, 0.0);
    TRAJ_setIntValue(trajHandle_Id, 0.0);
    TRAJ_setMinValue(trajHandle_Id, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_Id,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_Id,
                     (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize the fwc reference trajectory
    //
    trajHandle_fwc = TRAJ_init(&traj_fwc, sizeof(traj_fwc));

    //
    // configure the fwc reference trajectory
    //
    TRAJ_setTargetValue(trajHandle_fwc, 0.0);
    TRAJ_setIntValue(trajHandle_fwc, 0.0);
    TRAJ_setMinValue(trajHandle_fwc, -USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxValue(trajHandle_fwc,  USER_MOTOR_MAX_CURRENT_A);
    TRAJ_setMaxDelta(trajHandle_fwc,
                     (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz));

    //
    // initialize and configure offsets using first-order filter
    //
    {
        //
        // Sets the first-order filter denominator coefficients
        // a1, the filter coefficient value for z^(-1)
        // b0, the filter coefficient value for z^0
        // b1, the filter coefficient value for z^(-1)
        //
        uint16_t cnt = 0;
        float32_t b0 = userParams.offsetPole_rps / userParams.ctrlFreq_Hz;
        float32_t a1 = (b0 - 1.0);
        float32_t b1 = 0.0;

        //
        // For Current offset calibration filter
        //
        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            filterHandle_I[cnt] = FILTER_FO_init(&filter_I[cnt],
                                                   sizeof(filter_I[cnt]));

            FILTER_FO_setDenCoeffs(filterHandle_I[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_I[cnt], b0, b1);

            FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                        motorVars.offsets_I_A.value[cnt],
                                        motorVars.offsets_I_A.value[cnt]);
        }

        //
        // For Voltage offset calibration filter
        //
        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            filterHandle_V[cnt] = FILTER_FO_init(&filter_V[cnt],
                                                   sizeof(filter_V[cnt]));

            FILTER_FO_setDenCoeffs(filterHandle_V[cnt], a1);
            FILTER_FO_setNumCoeffs(filterHandle_V[cnt], b0, b1);

            FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                        motorVars.offsets_V_V.value[cnt],
                                        motorVars.offsets_V_V.value[cnt]);
        }

        motorVars.flagEnableOffsetCalc = true;
        offsetCalcCount = 0;
    }

    motorVars.faultMask.all = FAULT_MASK_OC_OV;

    //
    // initialize the angle generate module
    //
    angleGenHandle = ANGLE_GEN_init(&angleGen, sizeof(angleGen));
    ANGLE_GEN_setParams(angleGenHandle, userParams.ctrlPeriod_sec);


#ifdef PWMDAC_ENABLE
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &adcData.I_A.value[0];
    pwmDACData.ptrData[1] = &adcData.I_A.value[1];
    pwmDACData.ptrData[2] = &adcData.V_V.value[0];
    pwmDACData.ptrData[3] = &adcData.V_V.value[1];

    pwmDACData.offset[0] = 1.0;     // Change -1.0~1.0 to 0~1.0 for pwmDac
    pwmDACData.offset[1] = 0.5;     // Change -1.0~1.0 to 0~1.0 for pwmDac
    pwmDACData.offset[2] = 0.5;     // Change -1.0~1.0 to 0~1.0 for pwmDac
    pwmDACData.offset[3] = 0.5;     // Change -1.0~1.0 to 0~1.0 for pwmDac

    pwmDACData.gain[0] = -MATH_ONE_OVER_TWO_PI;  // Convert -PI()~PI() to 0~1.0
    pwmDACData.gain[1] = 1.0 / USER_ADC_FULL_SCALE_CURRENT_A;
    pwmDACData.gain[2] = 1.0 / USER_ADC_FULL_SCALE_VOLTAGE_V;
    pwmDACData.gain[3] = 1.0 / USER_ADC_FULL_SCALE_CURRENT_A;
#endif  // PWMDAC_ENABLE

#ifdef DATALOG_ENABLE
    // Initialize Datalog
    datalogHandle = DATALOG_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_resetDlogWithDMA();
    HAL_setupDlogWithDMA(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDlogWithDMA(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);
    #if (DATA_LOG_BUFF_NUM == 4)
    HAL_setupDlogWithDMA(halHandle, 2, &datalogBuff3[0], &datalogBuff3[1]);
    HAL_setupDlogWithDMA(halHandle, 3, &datalogBuff4[0], &datalogBuff4[1]);
    #endif  // DATA_LOG_BUFF_NUM == 4

    // set datalog parameters
    datalogObj->iptr[0] = &adcData.I_A.value[0];
    datalogObj->iptr[1] = &adcData.I_A.value[1];
    #if (DATA_LOG_BUFF_NUM == 4)
    datalogObj->iptr[2] = &adcData.V_V.value[0];
    datalogObj->iptr[3] = &adcData.V_V.value[1];
    #endif  // DATA_LOG_BUFF_NUM == 4
#endif  // DATALOG_ENABLE

    //
    // setup faults
    //
    HAL_setupFaults(halHandle);

#ifdef DRV8320_SPI
    //
    // turn on the DRV8320 if present
    //
    HAL_enableDRV(halHandle);

    //
    // initialize the DRV8320 interface
    //
    HAL_setupDRVSPI(halHandle, &drvSPI8320Vars);

    drvSPI8320Vars.Ctrl_Reg_05.VDS_LVL = DRV8320_VDS_LEVEL_1P300_V;
    drvSPI8320Vars.Ctrl_Reg_05.DEAD_TIME = DRV8320_DEADTIME_100_NS;
    drvSPI8320Vars.writeCmd = 1;
#endif

    //
    // Set some global variables
    //
    motorVars.pwmISRCount = 0;          // clear the counter
    motorVars.speedRef_Hz = 20.0;      // set reference frequency to 20.0Hz

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

    //
    // initialize the interrupt vector table
    //
    HAL_initIntVectorTable(halHandle);

    //
    // enable the ADC interrupts
    //
    HAL_enableADCInts(halHandle);

    //
    // disable global interrupts
    //
    HAL_enableGlobalInts(halHandle);

    //
    // enable debug interrupts
    //
    HAL_enableDebugInt(halHandle);

    //
    // Waiting for enable system flag to be set
    //
    while(motorVars.flagEnableSys == false)
    {

    }

    //
    // loop while the enable system flag is true
    //
    while(motorVars.flagEnableSys == true)
    {
        //
        // 1ms time base
        //
        if(HAL_getTimerStatus(halHandle, HAL_CPU_TIMER1))
        {
            motorVars.timerCnt_1ms++;

            HAL_clearTimerFlag(halHandle, HAL_CPU_TIMER1);
        }

        motorVars.mainLoopCount++;

        //
        // set the reference value for internal DACA and DACB
        //
        HAL_setDACValue(halHandle, 0, motorVars.dacaVal);
        HAL_setDACValue(halHandle, 1, motorVars.dacbVal);

        //
        // set internal DAC value for on-chip comparator for current protection
        //
        {
            uint16_t  cmpssCnt;

            for(cmpssCnt = 0; cmpssCnt < HAL_NUM_CMPSS_CURRENT; cmpssCnt++)
            {
                HAL_setCMPSSDACValueHigh(halHandle,
                                         cmpssCnt, motorVars.dacValH);

                HAL_setCMPSSDACValueLow(halHandle,
                                        cmpssCnt, motorVars.dacValL);
            }
        }

        //
        // set custom speed controller gains
        //
        PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);

        //
        // enable or disable force angle
        //
        EST_setFlag_enableForceAngle(estHandle,
                                     motorVars.flagEnableForceAngle);

        //
        // enable or disable bypassLockRotor flag
        //
        if(userParams.motor_type == MOTOR_TYPE_INDUCTION)
        {
            EST_setFlag_bypassLockRotor(estHandle,
                                        motorVars.flagBypassLockRotor);
        }

        if(HAL_getPwmEnableStatus(halHandle) == true)
        {
            if(HAL_getTripFaults(halHandle) != 0)
            {
                motorVars.faultNow.bit.moduleOverCurrent = 1;
            }
        }

        motorVars.faultUse.all =
                motorVars.faultNow.all & motorVars.faultMask.all;

        //
        // Had some faults to stop the motor
        //
        if(motorVars.faultUse.all != 0)
        {
            motorVars.flagRunIdentAndOnLine = 0;
        }

        if((motorVars.flagRunIdentAndOnLine == true) &&
           (motorVars.flagEnableOffsetCalc == false))
        {
            //
            // enable the estimator
            //
            EST_enable(estHandle);

            //
            // enable the trajectory generator
            //
            EST_enableTraj(estHandle);

            if(HAL_getPwmEnableStatus(halHandle) == false)
            {

                //
                // enable the PWM
                //
                HAL_enablePWM(halHandle);
            }

            //
            // set the reference to the trajectory of speed
            //
            TRAJ_setTargetValue(trajHandle_spd, motorVars.speedRef_Hz);

            //
            // set the acceleration to the trajectory of speed
            //
            TRAJ_setMaxDelta(trajHandle_spd,
                           (motorVars.accelerationMax_Hzps / USER_ISR_FREQ_Hz));
        }
        else if(motorVars.flagEnableOffsetCalc == false)
        {
            //
            // disable the estimator
            //
            EST_disable(estHandle);

            //
            // disable the trajectory generator
            //
            EST_disableTraj(estHandle);

            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // clear integral outputs of the controllers
            //
            PI_setUi(piHandle_Id, 0.0);
            PI_setUi(piHandle_Iq, 0.0);
            PI_setUi(piHandle_fwc, 0.0);
            PI_setUi(piHandle_spd, 0.0);

            //
            // clear current references
            //
            Idq_ref_A.value[0] = 0.0;
            Idq_ref_A.value[1] = 0.0;

            //
            // clear current offsets
            //
            Idq_offset_A.value[0] = 0.0;
            Idq_offset_A.value[1] = 0.0;

            //
            // get the magnetic current for ACIM
            //
            motorVars.IdRated_A = EST_getIdRated_A(estHandle);

            //
            // clear current and angle for FWC and MTPA
            //
            motorVars.VsRef_pu = USER_MAX_VS_MAG_PU;
            motorVars.IsRef_A = 0.0;
            motorVars.angleCurrent_rad = 0.0;

            //
            // clear the trajectory of speed
            //
            TRAJ_setTargetValue(trajHandle_spd, 0.0);
            TRAJ_setIntValue(trajHandle_spd, 0.0);
        }

        //
        // check the trajectory generator
        //
        if(EST_isTrajError(estHandle) == true)
        {
            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // set the enable system flag to false
            //
            motorVars.flagEnableSys = false;
        }
        else
        {
            //
            // update the trajectory generator state
            //
            EST_updateTrajState(estHandle);
        }

        //
        // check the estimator
        //
        if(EST_isError(estHandle) == true)
        {
            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);

            //
            // set the enable system flag to false
            //
            motorVars.flagEnableSys = false;
        }
        else        // No estimator error
        {
            motorVars.Id_target_A = EST_getIntValue_Id_A(estHandle);

            flagEstStateChanged = EST_updateState(estHandle, 0.0);

            if(flagEstStateChanged == true)
            {
                //
                // configure the trajectory generator
                //
                EST_configureTraj(estHandle);
            }
        }

        if(EST_isMotorIdentified(estHandle) == true)
        {
            motorVars.flagMotorIdentified = true;
        }

        //
        // update the global variables
        //
        updateGlobalVariables(estHandle);

#ifdef DRV8320_SPI
        //
        // DRV8320 Read/Write
        //
        HAL_writeDRVData(halHandle, &drvSPI8320Vars);

        HAL_readDRVData(halHandle, &drvSPI8320Vars);
#endif
    } // end of while() loop

    //
    // disable the PWM
    //
    HAL_disablePWM(halHandle);

} // end of main() function

__interrupt void mainISR(void)
{


    motorVars.pwmISRCount++;

    //
    // toggle status LED
    //
    counterLED++;

    if(counterLED > (uint32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLED(halHandle, HAL_GPIO_LED2);
        counterLED = 0;
    }

    //
    // acknowledge the ADC interrupt
    //
    HAL_ackADCInt(halHandle, ADC_INT_NUMBER1);

    //
    // read the ADC data with offsets
    //
    HAL_readADCDataWithOffsets(halHandle, &adcData);

    //
    // remove offsets
    //
    adcData.I_A.value[0] -= motorVars.offsets_I_A.value[0];
    adcData.I_A.value[1] -= motorVars.offsets_I_A.value[1];
    adcData.I_A.value[2] -= motorVars.offsets_I_A.value[2];
    adcData.V_V.value[0] -= motorVars.offsets_V_V.value[0] * adcData.dcBus_V;
    adcData.V_V.value[1] -= motorVars.offsets_V_V.value[1] * adcData.dcBus_V;
    adcData.V_V.value[2] -= motorVars.offsets_V_V.value[2] * adcData.dcBus_V;

    if(motorVars.flagEnableOffsetCalc == false)
    {
        float32_t outMax_V;
        MATH_Vec2 phasor;

        //
        // run Clarke transform on current
        //
        CLARKE_run(clarkeHandle_I, &(adcData.I_A), &(estInputData.Iab_A));

        //
        // run Clarke transform on voltage
        //
        CLARKE_run(clarkeHandle_V, &(adcData.V_V), &(estInputData.Vab_V));

        counterTrajSpeed++;

        if(counterTrajSpeed >= userParams.numIsrTicksPerTrajTick)
        {
            //
            // clear counter
            //
            counterTrajSpeed = 0;

            //
            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            //
            TRAJ_run(trajHandle_spd);

            motorVars.speedTraj_Hz = TRAJ_getIntValue(trajHandle_spd);
        }

        estInputData.dcBus_V = adcData.dcBus_V;
        estInputData.speed_ref_Hz = motorVars.speedTraj_Hz;
        estOutputData.oneOverDcBus_invV = 1.0 / estInputData.dcBus_V;

        //
        // run the estimator
        //
        EST_run(estHandle, &estInputData, &estOutputData);

        //
        // get Idq, reutilizing a Park transform used inside the estimator.
        // This is optional, user's Park works as well
        //
        EST_getIdq_A(estHandle, (MATH_Vec2 *)(&(Idq_in_A)));

        //
        // compute angle with delay compensation
        //
        angleDelta_rad = userParams.angleDelayed_sf_sec *
                         estOutputData.fm_lp_rps;

        angleEst_rad = MATH_incrAngle(estOutputData.angle_rad,
                                            angleDelta_rad);

        ANGLE_GEN_run(angleGenHandle, estInputData.speed_ref_Hz);
        angleFoc_rad = ANGLE_GEN_getAngle(angleGenHandle);

        Idq_ref_A.value[0] = IdqSet_A.value[0];
        Idq_ref_A.value[1] = IdqSet_A.value[1];

        //
        // compute the sin/cos phasor using fast RTS function, callable assembly
        //
        phasor.value[0] = cosf(angleFoc_rad);
        phasor.value[1] = sinf(angleFoc_rad);

        //
        // set the phasor in the Park transform
        //
        PARK_setPhasor(parkHandle_I, &phasor);

        //
        // run the Park module
        //
        PARK_run(parkHandle_I, &(estInputData.Iab_A), &(Idq_in_A));

        //
        // run the Id controller
        //
        PI_run_series(piHandle_Id,
                      Idq_ref_A.value[0] + Idq_offset_A.value[0],
                      Idq_in_A.value[0],
                      0.0,
                      &(Vdq_out_V.value[0]));

        //
        // calculate Iq controller limits, and run Iq controller using fast RTS
        // function, callable assembly
        //
        outMax_V = sqrt((userParams.maxVsMag_V * userParams.maxVsMag_V) -
                        (Vdq_out_V.value[0] * Vdq_out_V.value[0]));

        PI_setMinMax(piHandle_Iq, (-outMax_V), outMax_V);
        PI_run_series(piHandle_Iq,
                      Idq_ref_A.value[1] + Idq_offset_A.value[1],
                      Idq_in_A.value[1],
                      0.0,
                      &(Vdq_out_V.value[1]));

        //
        // set the phasor in the inverse Park transform
        //
        IPARK_setPhasor(iparkHandle, &phasor);

        //
        // run the inverse Park module
        //
        IPARK_run(iparkHandle, &Vdq_out_V, &Vab_out_V);

        //
        // setup the space vector generator (SVGEN) module
        //
        SVGEN_setup(svgenHandle, estOutputData.oneOverDcBus_invV);

        //
        // run the space vector generator (SVGEN) module
        //
        SVGEN_run(svgenHandle, &Vab_out_V, &(pwmData.Vabc_pu));
    }
    else if(motorVars.flagEnableOffsetCalc == true)
    {
        runOffsetsCalculation();
    }

    if(HAL_getPwmEnableStatus(halHandle) == false)
    {
        //
        // clear PWM data
        //
        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;
    }

    //
    // write the PWM compare values
    //
    HAL_writePWMData(halHandle, &pwmData);

#ifdef PWMDAC_ENABLE
    //
    // connect inputs of the PWMDAC module.
    //
    HAL_writePWMDACData(halHandle, &pwmDACData);
#endif  // PWMDAC_ENABLE

#ifdef DATALOG_ENABLE
    //
    // call datalog
    //
    DATALOG_updateWithDMA(datalogHandle);

    //
    // Force trig DMA channel to save the data
    //
    HAL_trigDlogWithDMA(halHandle, 0);
    HAL_trigDlogWithDMA(halHandle, 1);
    HAL_trigDlogWithDMA(halHandle, 2);
    HAL_trigDlogWithDMA(halHandle, 3);
#endif  //  DATALOG_ENABLE



    return;
} // end of mainISR() function

void runOffsetsCalculation(void)
{
    float32_t invVdcbus;
    uint16_t cnt;

    if(motorVars.flagEnableSys == true)
    {
        //
        // enable the PWM
        //
        HAL_enablePWM(halHandle);

        //
        // set the 3-phase output PWMs to 50% duty cycle
        //
        pwmData.Vabc_pu.value[0] = 0.0;
        pwmData.Vabc_pu.value[1] = 0.0;
        pwmData.Vabc_pu.value[2] = 0.0;

        //
        // set to the inverse dc bus voltage
        //
        invVdcbus = 1.0f / adcData.dcBus_V;

        for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
        {
            //
            // reset current offsets used
            //
            motorVars.offsets_I_A.value[cnt] = 0.0;

            //
            // run current offset estimation
            //
            FILTER_FO_run(filterHandle_I[cnt],
                          adcData.I_A.value[cnt]);
        }

        for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
        {
            //
            // reset voltage offsets used
            //
            motorVars.offsets_V_V.value[cnt] = 0.0;

            //
            // run voltage offset estimation
            //
            FILTER_FO_run(filterHandle_V[cnt],
                          adcData.V_V.value[cnt] * invVdcbus);
        }

        offsetCalcCount++;

        if(offsetCalcCount >= offsetCalcWaitTime)
        {
            for(cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++)
            {
                //
                // get calculated current offsets from filter
                //
                motorVars.offsets_I_A.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_I[cnt]);

                //
                // clear current filters
                //
                FILTER_FO_setInitialConditions(filterHandle_I[cnt],
                                              motorVars.offsets_I_A.value[cnt],
                                              motorVars.offsets_I_A.value[cnt]);
            }

            for(cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++)
            {
                //
                // get calculated voltage offsets from filter
                //
                motorVars.offsets_V_V.value[cnt] =
                        FILTER_FO_get_y1(filterHandle_V[cnt]);

                //
                // clear voltage filters
                //
                FILTER_FO_setInitialConditions(filterHandle_V[cnt],
                                              motorVars.offsets_V_V.value[cnt],
                                              motorVars.offsets_V_V.value[cnt]);
            }

            offsetCalcCount = 0;
            motorVars.flagEnableOffsetCalc = false;

            //
            // disable the PWM
            //
            HAL_disablePWM(halHandle);
        }
    }

    return;
} // end of runOffsetsCalculation() function

//
// end of file
//
