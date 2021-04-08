//#############################################################################
// $TI Release: MotorControl SDK v3.01.00.00 $
// $Release Date: Mon Jan 11 11:23:03 CST 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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

//! \file
//! \brief Defines
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

#ifndef _FLUX_HF_H_
#define _FLUX_HF_H_

// **************************************************************************
// the includes

// modules
#include "ti_math.h"

//!
//!
//! \defgroup FLUX_HF FLUX_HF
//!
//! @{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the function prototypes

//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pUserParams A pointer to the user parameters structure
//! \param[in] oneOverFluxGain_sf The gain coefficient for flux estimation
extern void EST_setOneOverFluxGain_sf(EST_Handle handle, USER_Params *pUserParams,
                                      const float32_t oneOverFluxGain_sf);


//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pUserParams A pointer to the user parameters structure
//! \param[in] freqLFP_sf The low pass filter coefficient
extern void EST_setFreqLFP_sf(EST_Handle handle, USER_Params *pUserParams,
                              const float32_t freqLFP_sf);


//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pUserParams A pointer to the user parameters structure
//! \param[in] bemf_sf     The bemf coefficient
extern void EST_setBemf_sf(EST_Handle handle, USER_Params *pUserParams,
                              const float32_t bemf_sf);


// **************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of _FLUX_HF_H_ definition

