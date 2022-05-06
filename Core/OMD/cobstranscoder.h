/*
 * cobstranscoder.h
 *
 *  Created on: Mar 9, 2022
 *      Author: Valerio.Mazzoni
 */

#ifndef OMD_COBSTRANSCODER_H_
#define OMD_COBSTRANSCODER_H_

#include <stdlib.h>
#include <stdio.h>
#include <vc_vector.h>

/****************************************
 * 			Enumerators
 ***************************************/

typedef enum
{
    COBS_ENCODE_OK                  = 0x00,
    COBS_ENCODE_NULL_POINTER        = 0x01,
    COBS_ENCODE_OUT_BUFFER_OVERFLOW = 0x02
} cobs_encode_status;

typedef struct
{
    size_t              out_len;
    cobs_encode_status  status;
} cobs_encode_result;


typedef enum
{
    COBS_DECODE_OK                  = 0x00,
    COBS_DECODE_NULL_POINTER        = 0x01,
    COBS_DECODE_OUT_BUFFER_OVERFLOW = 0x02,
    COBS_DECODE_ZERO_BYTE_IN_INPUT  = 0x04,
    COBS_DECODE_INPUT_TOO_SHORT     = 0x08
} cobs_decode_status;

typedef struct
{
    size_t              out_len;
    cobs_decode_status  status;
} cobs_decode_result;

typedef enum
{
	COBS_DECODE_SUCCESS,
	ERROR_ZERO_BYTE_NOT_EXPECTED
} DecodeStatus;

/****************************************
 * 			COBS Transcoding
 ***************************************/
/*
 * @param *dst_buf_ptr:    The buffer into which the result will be written
 * @param dst_buf_len:    Length of the buffer into which the result will be written
 * @param *src_ptr:        The byte string to be encoded
 * @param src_len:        Length of the byte string to be encoded
 *
 * @returns:       A struct containing the success status of the encoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobs_encode_result omdEncodeBuf(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len);

/*
 * @param *dst_buf_ptr:    The buffer into which the result will be written
 * @param dst_buf_len:    Length of the buffer into which the result will be written
 * @param *src_ptr:        The byte string to be decoded
 * @param src_len:        Length of the byte string to be decoded
 *
 * @returns:       A struct containing the success status of the decoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobs_decode_result omdDecodeBuf(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len);

/// \details    The encoding process cannot fail.
void omdEncode(vc_vector* rawData, vc_vector* encodedData);

/// \brief      Decode data using "Consistent Overhead Byte Stuffing" (COBS).
/// \details    Provided encodedData is expected to be a single, valid COBS encoded packet. If not, method
///             will return #DecodeStatus::ERROR_ZERO_BYTE_NOT_EXPECTED.
///             #decodedData is emptied of any pre-existing data. If the decode fails, decodedData is left empty.
DecodeStatus omdDecode(vc_vector* encodedData, vc_vector* decodedData);

#endif /* OMD_COBSTRANSCODER_H_ */
