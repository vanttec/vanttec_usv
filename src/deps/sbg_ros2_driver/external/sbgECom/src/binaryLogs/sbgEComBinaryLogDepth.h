/*!
 * \file		sbgEComBinaryLogDepth.h
 *  \author		SBG Systems
 * \date		20 February 2019
 *
 * \brief		This file is used to store depth measurements.
 *
 * Depth sensor are used for subsea navigation to improve height.
 *
 * \section CodeCopyright Copyright Notice 
 *  The MIT license
 *  
 *  Copyright (C) 2007-2020, SBG Systems SAS. All rights reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#ifndef SBG_ECOM_BINARY_LOG_DEPTH_H
#define SBG_ECOM_BINARY_LOG_DEPTH_H

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log Air Data status definitions                                    -//
//----------------------------------------------------------------------//

/*!
 * Air Data sensor status mask definitions
 */
#define SBG_ECOM_DEPTH_TIME_IS_DELAY				(0x0001u << 0)		/*!< Set to 1 if the time stamp field represents a delay instead of an absolute time stamp. */
#define SBG_ECOM_DEPTH_PRESSURE_ABS_VALID			(0x0001u << 1)		/*!< Set to 1 if the pressure field is filled and valid. */
#define SBG_ECOM_DEPTH_ALTITUDE_VALID				(0x0001u << 2)		/*!< Set to 1 if the depth altitude field is filled and valid. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for Depth sensor measurement (subsea).
 */
typedef struct _SbgLogDepth
{
	uint32_t	timeStamp;						/*!< Time in us since the sensor power up OR measurement delay in us. */
	uint16_t	status;							/*!< Airdata sensor status bitmask. */
	float	pressureAbs;					/*!< Raw absolute pressure measured by the depth sensor in Pascals. */
	float	altitude;						/*!< Altitude computed from depth sensor in meters and positive upward. */
} SbgLogDepth;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_DEPTH message and fill the corresponding structure.
 *
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDepth(SbgStreamBuffer *pInputStream, SbgLogDepth *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_DEPTH message to the output stream buffer from the provided structure.
 *
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDepth(SbgStreamBuffer *pOutputStream, const SbgLogDepth *pInputData);

#endif
