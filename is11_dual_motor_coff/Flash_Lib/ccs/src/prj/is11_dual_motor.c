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

//! \file   solutions/common/sensorless_foc/source/is11_dual_motor.c
//! \brief  This lab is used to control two inverters independently from one MCU.
//!         Only support LaunchXL-F280049C + BoostXL-DRV8320RS
//!

// **************************************************************************

// solutions
#include "labs_dm.h"

#pragma CODE_SECTION(mainISR, ".TI.ramfunc");
#pragma CODE_SECTION(runOffsetsCalculation, ".TI.ramfunc");

uint16_t counterLED = 0;  //!< Counter used to divide down the ISR rate for
                           //!< visually blinking an LED

uint32_t offsetCalcWaitTime = 50000;   //!< Wait time setting for current/voltage
                                        //!< offset calibration, unit: ISR cycles

// the globals
HAL_ADCData_t adcData[2] = { {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0},
                              {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0} };

HAL_PWMData_t pwmData[2] = { {0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0} };

uint16_t counterSpeed[2] = {0, 0};
uint16_t counterTrajSpeed[2] = {0, 0};
uint16_t counterTrajId[2] = {0, 0};
uint32_t offsetCalcCount[2] = {0, 0};   //!< Counter used to count the wait time
                                        //!< for offset calibration, unit: ISR cycles

EST_InputData_t estInputData[2] = { {0, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0},
                                  {0, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0} };

EST_OutputData_t estOutputData[2] = { {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     {0.0, 0.0}, {0.0, 0.0}, 0, 0.0} ,
                                    {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     {0.0, 0.0}, {0.0, 0.0}, 0, 0.0} };

float32_t angleDelta_rad[2]= {0.0, 0.0};     //!< the rotor angle compensation value
float32_t angleEst_rad[2] = {0.0, 0.0};      //!< the rotor angle from FAST estimator
float32_t angleFoc_rad[2]= {0.0, 0.0};       //!< the rotor angle for FOC modules

MATH_Vec2 Idq_in_A[2] = {{0.0, 0.0}, {0.0, 0.0}};   //!< the d&q axis current are converter from
                                                    //!< 3-phase sampling input current of motor
MATH_Vec2 Idq_ref_A[2] = {{0.0, 0.0}, {0.0, 0.0}};  //!< the reference current on d&q rotation axis
MATH_Vec2 Idq_offset_A[2] = {{0.0, 0.0}, {0.0, 0.0}};   //!< the offsetting current on d&q rotation axis

MATH_Vec2 Vab_out_V[2] = {{0.0, 0.0}, {0.0, 0.0}};  //!< the output control voltage on alpha&beta axis
MATH_Vec2 Vdq_out_V[2] = {{0.0, 0.0}, {0.0, 0.0}};  //!< the output control voltage on d&q axis

USER_Params userParams[2];
#pragma DATA_SECTION(userParams,"ctrl_data");

volatile MOTOR_Vars_t motorVars[2] = {MOTOR1_VARS_INIT, MOTOR2_VARS_INIT};

HAL_Handle    halHandle;          //!< the handle for the hardware abstraction layer
HAL_Obj       hal;                //!< the hardware abstraction layer object

HAL_MTR_Handle halMtrHandle[2];   //!< the handle for the hardware abstraction
                                  //!< layer to motor control
HAL_MTR_Obj    halMtr[2];         //!< the hardware abstraction layer object
                                  //!< to motor control

EST_Handle    estHandle[2];       //!< the handle for the estimator

CLARKE_Handle clarkeHandle_I[2];  //!< the handle for the current Clarke transform
CLARKE_Obj    clarke_I[2];        //!< the current Clarke transform object

CLARKE_Handle clarkeHandle_V[2];  //!< the handle for the voltage Clarke transform
CLARKE_Obj    clarke_V[2];        //!< the voltage Clarke transform object

IPARK_Handle  iparkHandle_V[2];   //!< the handle for the inverse Park transform
IPARK_Obj     ipark_V[2];         //!< the inverse Park transform object

PARK_Handle   parkHandle_I[2];    //!< the handle for the Park object
PARK_Obj      park_I[2];          //!< the Park transform object

PARK_Handle   parkHandle_V[2];    //!< the handle for the Park object
PARK_Obj      park_V[2];          //!< the Park transform object

PI_Handle     piHandle_Id[2];     //!< the handle for the Id PI controller
PI_Obj        pi_Id[2];           //!< the Id PI controller object

PI_Handle     piHandle_Iq[2];     //!< the handle for the Iq PI controller
PI_Obj        pi_Iq[2];           //!< the Iq PI controller object

PI_Handle     piHandle_fwc[2];    //!< the handle for the fwc PI controller
PI_Obj        pi_fwc[2];          //!< the fwc PI controller object

PI_Handle     piHandle_spd[2];    //!< the handle for the speed PI controller
PI_Obj        pi_spd[2];          //!< the speed PI controller object

SVGEN_Handle  svgenHandle[2];     //!< the handle for the space vector generator
SVGEN_Obj     svgen[2];           //!< the space vector generator object

SVGENCURRENT_Obj svgencurrent[2];   //!< the handle for the space vector generator current
SVGENCURRENT_Handle svgencurrentHandle[2];  //<! the space vector generator current object

TRAJ_Handle   trajHandle_spd[2];  //!< the handle for the speed reference trajectory
TRAJ_Obj      traj_spd[2];        //!< the speed reference trajectory object

TRAJ_Handle   trajHandle_Id[2];   //!< the handle for the id reference trajectory
TRAJ_Obj      traj_Id[2];         //!< the id reference trajectory object

TRAJ_Handle   trajHandle_Iq[2];   //!< the handle for the iq reference trajectory
TRAJ_Obj      traj_Iq[2];         //!< the iq reference trajectory object

TRAJ_Handle   trajHandle_fwc[2];  //!< the handle for the fwc reference trajectory
TRAJ_Obj      traj_fwc[2];        //!< the fwc reference trajectory object

//!< the handles for the current offset calculation
FILTER_FO_Handle  filterHandle_I[2][USER_M1_NUM_CURRENT_SENSORS];

//!< the current offset calculation
FILTER_FO_Obj     filter_I[2][USER_M1_NUM_CURRENT_SENSORS];

//!< the handles for the voltage offset calculation
FILTER_FO_Handle  filterHandle_V[2][USER_M1_NUM_CURRENT_SENSORS];

//!< the voltage offset calculation
FILTER_FO_Obj     filter_V[2][USER_M1_NUM_VOLTAGE_SENSORS];

#ifdef DRV8320_SPI
// Watch window interface to the 8320 SPI
DRV8320_SPIVars_t drvSPI8320Vars[2];
#pragma DATA_SECTION(drvSPI8320Vars,"ctrl_data");
#endif

HAL_MotorNum_e ctrlNum = HAL_MTR_1;
HAL_MotorNum_e isrNum  = HAL_MTR_1;

MATH_Vec2 fwcPhasor[2] = {{0.0, 0.0}, {0.0, 0.0}};
uint16_t faultFlag[2] = {0, 0};

//
volatile SYSTEM_Vars_t systemVars = SYSTEM_VARS_INIT;

#ifdef PWMDAC_ENABLE
HAL_PWMDACData_t pwmDACData;

#pragma DATA_SECTION(pwmDACData,"ctrl_data");
#endif  // PWMDAC_ENABLE


// **************************************************************************
// the functions

void main(void)
{

    for(ctrlNum = HAL_MTR_1; ctrlNum <= HAL_MTR_2; ctrlNum++)
    {
        // initialize the user parameters
        USER_setDualMotorParams(&userParams[ctrlNum], ctrlNum);

        #ifdef _HVKIT_REV1p1_
        motorVars[ctrlNum].boardKit = BOARD_HVMTRPFC_REV1P1;
        #endif // _HVKIT_REV1p1_

        #ifdef _DRV8301_KIT_REVD_
        motorVars[ctrlNum].boardKit = BOARD_DRV8301_REVD;
        #endif  // _DRV8301_KIT_REVD_

        #ifdef _BOOSTXL_8320RS_REVA_
        motorVars[ctrlNum].boardKit = BOARD_BSXL8320RS_REVA;
        #endif  // _BOOSTXL_8320RS_REVA_

        userParams[ctrlNum].flag_bypassMotorId = true;

        // initialize the user parameters
        USER_setParams_priv(&userParams[ctrlNum]);
    }

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    for(ctrlNum = HAL_MTR_1; ctrlNum <= HAL_MTR_2; ctrlNum++)
    {
        // initialize the driver
        halMtrHandle[ctrlNum] = HAL_MTR_init(&halMtr[ctrlNum],
                                              sizeof(halMtr[ctrlNum]), ctrlNum);

        // set the driver parameters
        HAL_MTR_setParams(halMtrHandle[ctrlNum], ctrlNum);

        // initialize the Clarke modules
        clarkeHandle_I[ctrlNum] = CLARKE_init(&clarke_I[ctrlNum],
                                               sizeof(clarke_I[ctrlNum]));
        clarkeHandle_V[ctrlNum] = CLARKE_init(&clarke_V[ctrlNum],
                                               sizeof(clarke_V[ctrlNum]));

        // set the Clarke parameters
        setupClarke_I(clarkeHandle_I[ctrlNum],
                      userParams[ctrlNum].numCurrentSensors);
        setupClarke_V(clarkeHandle_V[ctrlNum],
                      userParams[ctrlNum].numVoltageSensors);

        // initialize the estimator
        estHandle[ctrlNum] = EST_initEst(ctrlNum);

        // set the default estimator parameters
        EST_setParams(estHandle[ctrlNum],&userParams[ctrlNum]);
        EST_setFlag_enableForceAngle(estHandle[ctrlNum],
                                     motorVars[ctrlNum].flagEnableForceAngle);
        EST_setFlag_enableRsRecalc(estHandle[ctrlNum],
                                   motorVars[ctrlNum].flagEnableRsRecalc);

        if(ctrlNum == HAL_MTR_1)
        {
            // set the scale factor for high frequency motor
            EST_setOneOverFluxGain_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M1_EST_FLUX_HF_SF);
            EST_setFreqLFP_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M1_EST_FREQ_HF_SF);
            EST_setBemf_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M1_EST_BEMF_HF_SF);
        }
        else if(ctrlNum == HAL_MTR_2)
        {
            // set the scale factor for high frequency motor
            EST_setOneOverFluxGain_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M2_EST_FLUX_HF_SF);
            EST_setFreqLFP_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M2_EST_FREQ_HF_SF);
            EST_setBemf_sf(estHandle[ctrlNum],
                           &userParams[ctrlNum], USER_M2_EST_BEMF_HF_SF);
        }


        // if motor is an induction motor, configure default state of PowerWarp
        if(userParams[ctrlNum].motor_type == MOTOR_TYPE_INDUCTION)
        {
            EST_setFlag_enablePowerWarp(estHandle[ctrlNum],
                                        motorVars[ctrlNum].flagEnablePowerWarp);
            EST_setFlag_bypassLockRotor(estHandle[ctrlNum],
                                        motorVars[ctrlNum].flagBypassLockRotor);
        }

        // initialize the inverse Park module
        iparkHandle_V[ctrlNum] = IPARK_init(&ipark_V[ctrlNum],
                                             sizeof(ipark_V[ctrlNum]));

        // initialize the Park module
        parkHandle_I[ctrlNum] = PARK_init(&park_I[ctrlNum],
                                           sizeof(park_I[ctrlNum]));
        parkHandle_V[ctrlNum] = PARK_init(&park_V[ctrlNum],
                                           sizeof(park_V[ctrlNum]));

        // initialize the PI controllers
        piHandle_Id[ctrlNum]  = PI_init(&pi_Id[ctrlNum], sizeof(pi_Id[ctrlNum]));
        piHandle_Iq[ctrlNum]  = PI_init(&pi_Iq[ctrlNum], sizeof(pi_Iq[ctrlNum]));
        piHandle_fwc[ctrlNum] = PI_init(&pi_fwc[ctrlNum], sizeof(pi_fwc[ctrlNum]));
        piHandle_spd[ctrlNum] = PI_init(&pi_spd[ctrlNum], sizeof(pi_spd[ctrlNum]));

        // setup the controllers, speed, d/q-axis current pid regulator
        setupControllers(ctrlNum);

        // initialize the space vector generator module
        svgenHandle[ctrlNum] = SVGEN_init(&svgen[ctrlNum],
                                           sizeof(svgen[ctrlNum]));

        // initialize the speed reference trajectory
        trajHandle_spd[ctrlNum] = TRAJ_init(&traj_spd[ctrlNum],
                                             sizeof(traj_spd[ctrlNum]));

        // initialize the Id reference trajectory
        trajHandle_Id[ctrlNum] = TRAJ_init(&traj_Id[ctrlNum],
                                            sizeof(traj_Id[ctrlNum]));

        // initialize the Iq reference trajectory
        trajHandle_Iq[ctrlNum] = TRAJ_init(&traj_Iq[ctrlNum],
                                            sizeof(traj_Iq[ctrlNum]));

        // initialize the fwc reference trajectory
        trajHandle_fwc[ctrlNum] = TRAJ_init(&traj_fwc[ctrlNum],
                                             sizeof(traj_fwc[ctrlNum]));

        // configure the speed reference trajectory (Hz)
        TRAJ_setTargetValue(trajHandle_spd[ctrlNum], 0.0);
        TRAJ_setIntValue(trajHandle_spd[ctrlNum], 0.0);
        TRAJ_setMinValue(trajHandle_spd[ctrlNum],
                         -userParams[ctrlNum].maxFrequency_Hz);         // Maximum Frequency
        TRAJ_setMaxValue(trajHandle_spd[ctrlNum],
                         userParams[ctrlNum].maxFrequency_Hz);          // Maximum Frequency
        TRAJ_setMaxDelta(trajHandle_spd[ctrlNum],
                         (userParams[ctrlNum].maxAccel_Hzps / userParams[ctrlNum].ctrlFreq_Hz));

        // configure the Id reference trajectory
        TRAJ_setTargetValue(trajHandle_Id[ctrlNum], 0.0);
        TRAJ_setIntValue(trajHandle_Id[ctrlNum], 0.0);
        TRAJ_setMinValue(trajHandle_Id[ctrlNum],
                         -userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxValue(trajHandle_Id[ctrlNum],
                         userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxDelta(trajHandle_Id[ctrlNum],
                         (userParams[ctrlNum].maxCurrent_resEst_A / userParams[ctrlNum].ctrlFreq_Hz));

        // configure the Iq reference trajectory
        TRAJ_setTargetValue(trajHandle_Iq[ctrlNum], 0.0);
        TRAJ_setIntValue(trajHandle_Iq[ctrlNum], 0.0);
        TRAJ_setMinValue(trajHandle_Iq[ctrlNum],
                         -userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxValue(trajHandle_Iq[ctrlNum],
                         userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxDelta(trajHandle_Iq[ctrlNum],
                         (userParams[ctrlNum].maxCurrent_resEst_A / userParams[ctrlNum].ctrlFreq_Hz));

        // configure the fwc reference trajectory
        TRAJ_setTargetValue(trajHandle_fwc[ctrlNum], 0.0);
        TRAJ_setIntValue(trajHandle_fwc[ctrlNum], 0.0);
        TRAJ_setMinValue(trajHandle_fwc[ctrlNum],
                         -userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxValue(trajHandle_fwc[ctrlNum],
                         userParams[ctrlNum].maxCurrent_A);
        TRAJ_setMaxDelta(trajHandle_fwc[ctrlNum],
                         (userParams[ctrlNum].maxCurrent_resEst_A / userParams[ctrlNum].ctrlFreq_Hz));

        // initialize and configure offsets using filters
        {
            uint16_t filterCnt = 0;
            float32_t b0 = userParams[ctrlNum].offsetPole_rps / userParams[ctrlNum].ctrlFreq_Hz;
            float32_t a1 = (b0 - 1.0);
            float32_t b1 = 0.0;

            // For Current offset calibration filter
            for(filterCnt = 0; filterCnt<USER_M1_NUM_CURRENT_SENSORS; filterCnt++)
            {
                filterHandle_I[ctrlNum][filterCnt] = FILTER_FO_init(&filter_I[ctrlNum][filterCnt],
                                                         sizeof(filter_I[ctrlNum][filterCnt]));

                FILTER_FO_setDenCoeffs(filterHandle_I[ctrlNum][filterCnt], a1);
                FILTER_FO_setNumCoeffs(filterHandle_I[ctrlNum][filterCnt], b0, b1);

                FILTER_FO_setInitialConditions(filterHandle_I[ctrlNum][filterCnt],
                                               motorVars[ctrlNum].offsets_I_A.value[filterCnt],
                                               motorVars[ctrlNum].offsets_I_A.value[filterCnt]);
            }

            // For Voltage offset calibration filter
            for(filterCnt=0; filterCnt<USER_M1_NUM_VOLTAGE_SENSORS; filterCnt++)
            {
                filterHandle_V[ctrlNum][filterCnt] = FILTER_FO_init(&filter_V[ctrlNum][filterCnt],
                                                         sizeof(filter_V[ctrlNum][filterCnt]));

                FILTER_FO_setDenCoeffs(filterHandle_V[ctrlNum][filterCnt], a1);
                FILTER_FO_setNumCoeffs(filterHandle_V[ctrlNum][filterCnt], b0, b1);

                FILTER_FO_setInitialConditions(filterHandle_V[ctrlNum][filterCnt],
                                               motorVars[ctrlNum].offsets_V_V.value[filterCnt],
                                               motorVars[ctrlNum].offsets_V_V.value[filterCnt]);
            }

            motorVars[ctrlNum].flagEnableOffsetCalc = false;
            offsetCalcCount[ctrlNum] = 0;
        }

        motorVars[ctrlNum].offset_invVbus_invV = 0.5;   // the scale factor of half of dc bus

        motorVars[ctrlNum].faultMask.all = FAULT_MASK_OC_FU_OT;

        motorVars[ctrlNum].flagEnableForceAngle = true;
        motorVars[ctrlNum].flagEnableRsRecalc = false;
        motorVars[ctrlNum].flagEnableRsOnLine = false;

#ifdef _PGA_GAIN_6_EN_
        motorVars[ctrlNum].dacaVal = 2045U;
        motorVars[ctrlNum].dacbVal = 2045U;
#endif

#ifdef _PGA_GAIN_12_EN_
        motorVars[ctrlNum].dacaVal = 1023U;
        motorVars[ctrlNum].dacbVal = 1023U;
#endif

        // setup faults
        HAL_setupFaults(halMtrHandle[ctrlNum], ctrlNum);

  #ifdef DRV8320_SPI
        // turn on the DRV8320 if present
        HAL_enableDRV(halMtrHandle[ctrlNum]);

        // initialize the DRV8320 interface
        HAL_setupDRVSPI(halMtrHandle[ctrlNum], &drvSPI8320Vars[ctrlNum]);

        drvSPI8320Vars[ctrlNum].Ctrl_Reg_05.VDS_LVL = DRV8320_VDS_LEVEL_1P300_V;
        drvSPI8320Vars[ctrlNum].Ctrl_Reg_05.DEAD_TIME = DRV8320_DEADTIME_100_NS;
        drvSPI8320Vars[ctrlNum].writeCmd = 1;
  #endif

        // Set some global variables
        motorVars[ctrlNum].pwmISRCount = 0;

        // set the default target frequency to 20.0Hz
        motorVars[ctrlNum].speedRef_Hz = 20.0;

        // disable the PWM
        HAL_disablePWM(halMtrHandle[ctrlNum]);
    }


#ifdef PWMDAC_ENABLE
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &svgen[MTR_NUM_1].Ta;
    pwmDACData.ptrData[1] = &svgen[MTR_NUM_1].Tb;
    pwmDACData.ptrData[2] = &svgen[MTR_NUM_1].Tc;
    pwmDACData.ptrData[3] = &angleFoc_rad[MTR_NUM_1];

    pwmDACData.offset[0] = 0.5;
    pwmDACData.offset[1] = 0.5;
    pwmDACData.offset[2] = 0.5;
    pwmDACData.offset[3] = 1.0;


    pwmDACData.gain[0] = 2.0;
    pwmDACData.gain[1] = 2.0;
    pwmDACData.gain[2] = 2.0;
    pwmDACData.gain[3] = -MATH_ONE_OVER_TWO_PI;
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
    datalogObj->iptr[0] = &adcData[HAL_MTR_1].I_A.value[0];
    datalogObj->iptr[1] = &adcData[HAL_MTR_1].I_A.value[1];
    #if (DATA_LOG_BUFF_NUM == 4)
    datalogObj->iptr[2] = &adcData[HAL_MTR_1].V_V.value[0];
    datalogObj->iptr[3] = &adcData[HAL_MTR_1].V_V.value[1];
    #endif  // DATA_LOG_BUFF_NUM == 4
#endif  // DATALOG_ENABLE

    systemVars.flagEnableSystem = false;

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC interrupts
    HAL_enableADCInts(halHandle);

    // disable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {

    }

    motorVars[HAL_MTR_1].flagEnableSys = systemVars.flagEnableSystem;
    motorVars[HAL_MTR_2].flagEnableSys = systemVars.flagEnableSystem;

    //
    // loop while the enable system flag is true
    //
    while(systemVars.flagEnableSystem == true)
    {
        if(systemVars.flagEnableSynControl == true)
        {
            motorVars[HAL_MTR_1].flagEnableRunAndIdentify = systemVars.flagEnableRun;
            motorVars[HAL_MTR_1].speedRef_Hz = systemVars.speedSet_Hz;
            motorVars[HAL_MTR_1].accelerationMax_Hzps = systemVars.accelerationMaxSet_Hzps;

            motorVars[HAL_MTR_2].flagEnableRunAndIdentify = systemVars.flagEnableRun;
            motorVars[HAL_MTR_2].speedRef_Hz = systemVars.speedSet_Hz;
            motorVars[HAL_MTR_2].accelerationMax_Hzps = systemVars.accelerationMaxSet_Hzps;
        }

        //
        // 1ms time base
        //
        if(HAL_getTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            motorVars[0].timerCnt_1ms++;

            HAL_clearTimerFlag(halHandle, HAL_CPU_TIMER0);
        }


        for(ctrlNum = HAL_MTR_1; ctrlNum <= HAL_MTR_2; ctrlNum++)
        {
            if(motorVars[ctrlNum].flagEnableRunAndIdentify == true)
            {
                if(motorVars[ctrlNum].flagRunIdentAndOnLine == false)
                {
                    motorVars[ctrlNum].flagRunIdentAndOnLine = true;
                    motorVars[ctrlNum].faultNow.all = 0;
                }
            }
            else
            {
                motorVars[ctrlNum].flagRunIdentAndOnLine = false;
            }

            //
            // CPU Timer 2 reserve for testing
            //

            motorVars[ctrlNum].mainLoopCount++;

            //
            // set internal DAC value for on-chip comparator for current protection
            //
            {
                uint_least8_t  cmpssCnt;

                for(cmpssCnt=0; cmpssCnt<HAL_NUM_CMPSS_CURRENT; cmpssCnt++)
                {
                    HAL_setCMPSSDACValueHigh(halMtrHandle[ctrlNum],
                                             cmpssCnt, motorVars[ctrlNum].dacValH);

                    HAL_setCMPSSDACValueLow(halMtrHandle[ctrlNum],
                                            cmpssCnt, motorVars[ctrlNum].dacValL);
                }
            }

            // enable or disable force angle
            EST_setFlag_enableForceAngle(estHandle[ctrlNum],
                                         motorVars[ctrlNum].flagEnableForceAngle);

            EST_setFlag_enableRsRecalc(estHandle[ctrlNum],
                                       motorVars[ctrlNum].flagEnableRsRecalc);

            EST_setFlag_enableRsOnLine(estHandle[ctrlNum],
                                       motorVars[ctrlNum].flagEnableRsOnLine);

            // enable or disable bypassLockRotor flag
            if(userParams[ctrlNum].motor_type == MOTOR_TYPE_INDUCTION)
            {
                EST_setFlag_bypassLockRotor(estHandle[ctrlNum],
                                            motorVars[ctrlNum].flagBypassLockRotor);
            }

            if(HAL_getPwmEnableStatus(halMtrHandle[ctrlNum]) == true)
            {
                if(HAL_getTripFaults(halMtrHandle[ctrlNum]) !=0)
                {
                    motorVars[ctrlNum].faultNow.bit.moduleOverCurrent = 1;
                    faultFlag[ctrlNum] = HAL_getTripFaults(halMtrHandle[ctrlNum]);
                }
            }

            motorVars[ctrlNum].faultUse.all = motorVars[ctrlNum].faultNow.all &
                                                motorVars[ctrlNum].faultMask.all;

            // Had some faults to stop the motor
            if(motorVars[ctrlNum].faultUse.all != 0)
            {
                motorVars[ctrlNum].flagEnableRunAndIdentify = false;
                motorVars[ctrlNum].flagRunIdentAndOnLine = false;
            }

            if((motorVars[ctrlNum].flagRunIdentAndOnLine == true) &&
                    (motorVars[ctrlNum].flagEnableOffsetCalc == false))
            {
                if(HAL_getPwmEnableStatus(halMtrHandle[ctrlNum]) == false)
                {
                    // enable the estimator
                    EST_enable(estHandle[ctrlNum]);

                    // enable the trajectory generator
                    EST_enableTraj(estHandle[ctrlNum]);

                    // enable the PWM
                    HAL_enablePWM(halMtrHandle[ctrlNum]);
                }

                TRAJ_setTargetValue(trajHandle_spd[ctrlNum], motorVars[ctrlNum].speedRef_Hz);
                TRAJ_setMaxDelta(trajHandle_spd[ctrlNum],
                                 (motorVars[ctrlNum].accelerationMax_Hzps / USER_M1_ISR_FREQ_Hz));
            }
            else if(motorVars[ctrlNum].flagEnableOffsetCalc == false)
            {
                // disable the estimator
                EST_disable(estHandle[ctrlNum]);

                // disable the trajectory generator
                EST_disableTraj(estHandle[ctrlNum]);

                // disable the PWM
                HAL_disablePWM(halMtrHandle[ctrlNum]);

                // clear integral outputs of the controllers
                PI_setUi(piHandle_Id[ctrlNum], 0.0);
                PI_setUi(piHandle_Iq[ctrlNum], 0.0);
                PI_setUi(piHandle_fwc[ctrlNum], 0.0);
                PI_setUi(piHandle_spd[ctrlNum], 0.0);

                // clear current references
                Idq_ref_A[ctrlNum].value[0] = 0.0;
                Idq_ref_A[ctrlNum].value[1] = 0.0;

                Idq_offset_A[ctrlNum].value[0] = 0.0;
                Idq_offset_A[ctrlNum].value[1] = 0.0;

                motorVars[ctrlNum].IdRated_A = EST_getIdRated_A(estHandle[ctrlNum]);

                motorVars[ctrlNum].IsRef_A = 0.0;
                motorVars[ctrlNum].angleCurrent_rad = 0.0;

                TRAJ_setTargetValue(trajHandle_spd[ctrlNum], 0.0);
                TRAJ_setIntValue(trajHandle_spd[ctrlNum], 0.0);
            }

            // check the trajectory generator
            if(EST_isTrajError(estHandle[ctrlNum]) == true)
            {
                // disable the PWM
                HAL_disablePWM(halMtrHandle[ctrlNum]);

                // set the enable system flag to false
                motorVars[ctrlNum].flagEnableSys = false;
            }
            else
            {
                // update the trajectory generator state
                EST_updateTrajState(estHandle[ctrlNum]);
            }

            // check the estimator
            if(EST_isError(estHandle[ctrlNum]) == true)
            {
                // disable the PWM
                HAL_disablePWM(halMtrHandle[ctrlNum]);

                // set the enable system flag to false
                motorVars[ctrlNum].flagEnableSys = false;
            }
            else        // No any estimator error
            {
                motorVars[ctrlNum].Id_target_A = EST_getIntValue_Id_A(estHandle[ctrlNum]);

                bool flagEstStateChanged = EST_updateState(estHandle[ctrlNum],0.0);

                if(flagEstStateChanged == true)
                {
                    // configure the trajectory generator
                    EST_configureTraj(estHandle[ctrlNum]);

                    if(EST_isLockRotor(estHandle[ctrlNum]) ||
                            (EST_isMotorIdentified(estHandle[ctrlNum])
                                    && EST_isIdle(estHandle[ctrlNum])))
                    {
                        motorVars[ctrlNum].flagMotorIdentified = true;

                        // clear the flag
                        motorVars[ctrlNum].flagRunIdentAndOnLine = false;
                    }
                }
            }

            if(EST_isMotorIdentified(estHandle[ctrlNum]) == true)
            {
                if(motorVars[ctrlNum].flagSetupController == true)
                {
                    // update the controller
                    updateControllers(ctrlNum);
                }
                else
                {
                    motorVars[ctrlNum].flagMotorIdentified = true;
                    motorVars[ctrlNum].flagSetupController = true;

                    setupControllers(ctrlNum);
                }
            }

            // update the global variables
            updateGlobalVariables(estHandle[ctrlNum], ctrlNum);

            #ifdef DRV8320_SPI
            HAL_writeDRVData(halMtrHandle[ctrlNum], &drvSPI8320Vars[ctrlNum]);

            HAL_readDRVData(halMtrHandle[ctrlNum], &drvSPI8320Vars[ctrlNum]);
            #endif

        } // end of for(ctrlNum;)
    } // end of while() loop

    // disable the PWM
    HAL_disablePWM(halMtrHandle[0]);
    HAL_disablePWM(halMtrHandle[1]);
} // end of main() function

__interrupt void mainISR(void)
{
    // check ISR executing time
    HAL_setGPIOHigh(halHandle, HAL_GPIO_ISR);

    // toggle status LED
    counterLED++;

    if(counterLED > (uint32_t)(USER_M1_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLED(halHandle,HAL_GPIO_LED2);
        counterLED = 0;
    }

    // acknowledge the ADC interrupt
    HAL_ackADCInt(halHandle,ADC_INT_NUMBER1);


    for(isrNum = HAL_MTR_1; isrNum <= HAL_MTR_2; isrNum++)
    {
        // read the ADC data with offsets
        HAL_readADCDataWithOffsets(halHandle, halMtrHandle[isrNum],
                                   &adcData[isrNum], isrNum);

        // calculate Vbus scale factor to scale offsets with Vbus
        motorVars[isrNum].Vbus_sf = adcData[isrNum].dcBus_V *
                                       motorVars[isrNum].offset_invVbus_invV;

        // remove offsets
        adcData[isrNum].I_A.value[0] -= motorVars[isrNum].offsets_I_A.value[0];
        adcData[isrNum].I_A.value[1] -= motorVars[isrNum].offsets_I_A.value[1];
        adcData[isrNum].I_A.value[2] -= motorVars[isrNum].offsets_I_A.value[2];
        adcData[isrNum].V_V.value[0] -= motorVars[isrNum].offsets_V_V.value[0] *
                                                     motorVars[isrNum].Vbus_sf;
        adcData[isrNum].V_V.value[1] -= motorVars[isrNum].offsets_V_V.value[1] *
                                                     motorVars[isrNum].Vbus_sf;
        adcData[isrNum].V_V.value[2] -= motorVars[isrNum].offsets_V_V.value[2] *
                                                     motorVars[isrNum].Vbus_sf;
    }

    // Dual Motor Control
    for(isrNum = HAL_MTR_1; isrNum <= HAL_MTR_2; isrNum++)
    {
        // Verify close speed loop sensorless by low speed with FAST,
        // and high speed with Field Weakening Control & OVM
        if(motorVars[isrNum].flagEnableOffsetCalc == false)
        {
            float32_t outMax_V;
            MATH_Vec2 phasor;

            // run Clarke transform on current
            CLARKE_run(clarkeHandle_I[isrNum],
                       &adcData[isrNum].I_A, &(estInputData[isrNum].Iab_A));

            // run Clarke transform on voltage
            CLARKE_run(clarkeHandle_V[isrNum],
                       &adcData[isrNum].V_V, &(estInputData[isrNum].Vab_V));

            counterTrajSpeed[isrNum]++;

            if(counterTrajSpeed[isrNum] >= userParams[isrNum].numIsrTicksPerTrajTick)
            {
                // clear counter
                counterTrajSpeed[isrNum] = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(trajHandle_spd[isrNum]);

                motorVars[isrNum].speedTraj_Hz = TRAJ_getIntValue(trajHandle_spd[isrNum]);
            }

            // run the trajectories
            EST_runTraj(estHandle[isrNum]);

            // store the input data into a buffer
            estInputData[isrNum].dcBus_V = adcData[isrNum].dcBus_V;

            if(EST_getState(estHandle[isrNum]) != EST_STATE_ONLINE)
            {
                Idq_ref_A[isrNum].value[0] = EST_getIntValue_Id_A(estHandle[isrNum]);
                estInputData[isrNum].speed_ref_Hz = EST_getIntValue_spd_Hz(estHandle[isrNum]);
                estInputData[isrNum].speed_int_Hz = EST_getIntValue_spd_Hz(estHandle[isrNum]);
            }
            else
            {
                Idq_ref_A[isrNum].value[0] = EST_getIdRated_A(estHandle[isrNum]);
                estInputData[isrNum].speed_ref_Hz = motorVars[isrNum].speedTraj_Hz;
                estInputData[isrNum].speed_int_Hz = motorVars[isrNum].speedTraj_Hz;
            }

            // run the estimator
            EST_run(estHandle[isrNum],
                    &estInputData[isrNum],
                    &estOutputData[isrNum]);

            // get Idq, reutilizing a Park transform used inside the estimator.
            // This is optional, user's Park works as well
            EST_getIdq_A(estHandle[isrNum], (MATH_Vec2 *)(&(Idq_in_A[isrNum])));

            // run the speed controller
            // run the speed controller
            if(EST_doSpeedCtrl(estHandle[isrNum]))
            {
                counterSpeed[isrNum]++;

                if(counterSpeed[isrNum] >= userParams[isrNum].numCtrlTicksPerSpeedTick)
                {
                    counterSpeed[isrNum] = 0;

                    PI_run_series(piHandle_spd[isrNum],
                                  estInputData[isrNum].speed_ref_Hz,
                                  estOutputData[isrNum].fm_lp_rps * MATH_ONE_OVER_TWO_PI,
                                  0.0,
                                  (float32_t *)(&(motorVars[isrNum].IsRef_A)));

                    PI_run_series(piHandle_fwc[isrNum],
                                  motorVars[isrNum].VsRef_V,
                                  motorVars[isrNum].Vs_V,
                                  0.0,
                                  (float32_t *)(&(motorVars[isrNum].angleCurrent_rad)));

                    // compute the sin/cos phasor using fast RTS function, callable assembly
                    fwcPhasor[isrNum].value[0] = sinf(motorVars[isrNum].angleCurrent_rad);
                    fwcPhasor[isrNum].value[1] = cosf(motorVars[isrNum].angleCurrent_rad);

                    Idq_ref_A[isrNum].value[0] = motorVars[isrNum].IsRef_A *
                                                    fwcPhasor[isrNum].value[0];

                    Idq_ref_A[isrNum].value[1] = motorVars[isrNum].IsRef_A *
                                                    fwcPhasor[isrNum].value[1];
                }
            }
            else
            {
                Idq_ref_A[isrNum].value[1] = 0.0;
            }

            // update Id reference for Rs OnLine
            EST_updateId_ref_A(estHandle[isrNum],
                               (float32_t *)&(Idq_ref_A[isrNum].value[0]));

            // Maximum voltage output
            userParams[isrNum].maxVsMag_V = userParams[isrNum].maxVsMag_pu *
                                                 adcData[isrNum].dcBus_V;
            PI_setMinMax(piHandle_Id[isrNum],
                         -userParams[isrNum].maxVsMag_V, userParams[isrNum].maxVsMag_V);

            // run the Id controller
            PI_run_series(piHandle_Id[isrNum],
                          Idq_ref_A[isrNum].value[0] + Idq_offset_A[isrNum].value[0],
                          Idq_in_A[isrNum].value[0],
                          0.0,
                          &(Vdq_out_V[isrNum].value[0]));

            // calculate Iq controller limits, and run Iq controller using fast RTS
            // function, callable assembly
            #ifdef __TMS320C28XX_TMU__
            outMax_V = sqrt((userParams[isrNum].maxVsMag_V * userParams[isrNum].maxVsMag_V) -
                            (Vdq_out_V[isrNum].value[0] * Vdq_out_V[isrNum].value[0]));
            #else
            outMax_V = sqrt_fastRTS((userParams[isrNum].maxVsMag_V * userParams[isrNum].maxVsMag_V) -
                                    (Vdq_out_V[isrNum].value[0] * Vdq_out_V[isrNum].value[0]));
            #endif

            PI_setMinMax(piHandle_Iq[isrNum], -outMax_V, outMax_V);
            PI_run_series(piHandle_Iq[isrNum],
                          Idq_ref_A[isrNum].value[1] + Idq_offset_A[isrNum].value[1],
                          Idq_in_A[isrNum].value[1],
                          0.0,
                          &(Vdq_out_V[isrNum].value[1]));

            // compute angle with delay compensation
            angleDelta_rad[isrNum] = userParams[isrNum].angleDelayed_sf_sec *
                             estOutputData[isrNum].fm_lp_rps;

            angleEst_rad[isrNum] = MATH_incrAngle(estOutputData[isrNum].angle_rad,
                                          angleDelta_rad[isrNum]);

            angleFoc_rad[isrNum] = angleEst_rad[isrNum];

            // compute the sin/cos phasor using fast RTS function, callable assembly
            phasor.value[0] = cosf(angleFoc_rad[isrNum]);
            phasor.value[1] = sinf(angleFoc_rad[isrNum]);

            // set the phasor in the inverse Park transform
            IPARK_setPhasor(iparkHandle_V[isrNum], &phasor);

            // run the inverse Park module
            IPARK_run(iparkHandle_V[isrNum],
                      &Vdq_out_V[isrNum], &Vab_out_V[isrNum]);

            // setup the space vector generator (SVGEN) module
            SVGEN_setup(svgenHandle[isrNum],
                        estOutputData[isrNum].oneOverDcBus_invV);

            // run the space vector generator (SVGEN) module
            SVGEN_run(svgenHandle[isrNum],
                      &Vab_out_V[isrNum], &(pwmData[isrNum].Vabc_pu));
        }
        else if(motorVars[isrNum].flagEnableOffsetCalc == true)
        {
            runOffsetsCalculation(isrNum);
        }

        if(HAL_getPwmEnableStatus(halMtrHandle[isrNum]) == false)
        {
            // clear PWM data
            pwmData[isrNum].Vabc_pu.value[0] = 0.0;
            pwmData[isrNum].Vabc_pu.value[1] = 0.0;
            pwmData[isrNum].Vabc_pu.value[2] = 0.0;
        }

        // write the PWM compare values
        HAL_writePWMData(halMtrHandle[isrNum], &pwmData[isrNum]);
    }

#ifdef PWMDAC_ENABLE
    // connect inputs of the PWMDAC module.
    pwmDACData.value[0] = (*pwmDACData.ptrData[0]);
    pwmDACData.value[1] = (*pwmDACData.ptrData[1]);
    pwmDACData.value[2] = (*pwmDACData.ptrData[2]);
    pwmDACData.value[3] = (*pwmDACData.ptrData[3]);

    HAL_writePWMDACData(halHandle,&pwmDACData);
#endif  // PWMDAC_ENABLE

#ifdef DATALOG_ENABLE
    DATALOG_updateWithDMA(datalogHandle);

    // Force trig DMA channel to save the data
    HAL_trigDlogWithDMA(halHandle, 0);
    HAL_trigDlogWithDMA(halHandle, 1);
    HAL_trigDlogWithDMA(halHandle, 2);
    HAL_trigDlogWithDMA(halHandle, 3);
#endif  // DATALOG_ENABLE

    return;
} // end of mainISR() function


void runOffsetsCalculation(const uint16_t motorNum)
{
    uint16_t filterCnt;

    if(motorVars[motorNum].flagEnableSys == true)
    {
        // enable the PWM
        HAL_enablePWM(halMtrHandle[motorNum]);

        float32_t Vin;
        float32_t invVdcbus;

        // half of inversus dc bus voltage
        invVdcbus = 2.0 / adcData[motorNum].dcBus_V;

        // Set the 3-phase output PWMs to 50% duty cycle
        pwmData[motorNum].Vabc_pu.value[0] = 0.0;
        pwmData[motorNum].Vabc_pu.value[1] = 0.0;
        pwmData[motorNum].Vabc_pu.value[2] = 0.0;

        for(filterCnt=0; filterCnt< USER_M1_NUM_CURRENT_SENSORS; filterCnt++)
        {
            // reset current offsets used
            motorVars[motorNum].offsets_I_A.value[filterCnt] = 0.0;

            // run current offset estimation
            FILTER_FO_run(filterHandle_I[motorNum][filterCnt],
                          adcData[motorNum].I_A.value[filterCnt]);
        }

        for(filterCnt=0; filterCnt< USER_M1_NUM_VOLTAGE_SENSORS; filterCnt++)
        {
            // reset voltage offsets used
            motorVars[motorNum].offsets_V_V.value[filterCnt] = 0.0;

            Vin = adcData[motorNum].V_V.value[filterCnt] * invVdcbus;

            // run voltage offset estimation
            FILTER_FO_run(filterHandle_V[motorNum][filterCnt], Vin);
        }

        offsetCalcCount[motorNum]++;

        if(offsetCalcCount[motorNum] >= offsetCalcWaitTime)
        {
            for(filterCnt=0; filterCnt<USER_M1_NUM_CURRENT_SENSORS; filterCnt++)
            {
                // get calculated current offsets from filter
                motorVars[motorNum].offsets_I_A.value[filterCnt] =
                        FILTER_FO_get_y1(filterHandle_I[motorNum][filterCnt]);


                // clear current filters
                FILTER_FO_setInitialConditions(filterHandle_I[motorNum][filterCnt],
                                               motorVars[motorNum].offsets_I_A.value[filterCnt],
                                               motorVars[motorNum].offsets_I_A.value[filterCnt]);
            }

            for(filterCnt = 0; filterCnt < USER_M1_NUM_VOLTAGE_SENSORS; filterCnt++)
            {
                // get calculated voltage offsets from filter
                motorVars[motorNum].offsets_V_V.value[filterCnt] =
                        FILTER_FO_get_y1(filterHandle_V[motorNum][filterCnt]);

                // clear voltage filters
                FILTER_FO_setInitialConditions(filterHandle_V[motorNum][filterCnt],
                              motorVars[motorNum].offsets_V_V.value[filterCnt],
                              motorVars[motorNum].offsets_V_V.value[filterCnt]);
            }

            offsetCalcCount[motorNum] = 0;
            motorVars[motorNum].flagEnableOffsetCalc = false;

            // disable the PWM
            HAL_disablePWM(halMtrHandle[motorNum]);
        }
    }

    return;
} // end of runOffsetsCalculation() function

// end of file
