/*
 * cobstranscoder.c
 *
 *  Created on: Mar 9, 2022
 *      Author: Valerio.Mazzoni
 */

#include "cobstranscoder.h"

/*
 * dst_buf_ptr:    The buffer into which the result will be written
 * dst_buf_len:    Length of the buffer into which the result will be written
 * src_ptr:        The byte string to be encoded
 * src_len         Length of the byte string to be encoded
 *
 * returns:        A struct containing the success status of the encoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobs_encode_result omdEncodeBuf(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len)
{
    cobs_encode_result  result              = { 0, COBS_ENCODE_OK };
    const uint8_t *     src_read_ptr        = src_ptr;
    const uint8_t *     src_end_ptr         = src_read_ptr + src_len;
    uint8_t *           dst_buf_start_ptr   = dst_buf_ptr;
    uint8_t *           dst_buf_end_ptr     = dst_buf_start_ptr + dst_buf_len;
    uint8_t *           dst_code_write_ptr  = dst_buf_ptr;
    uint8_t *           dst_write_ptr       = dst_code_write_ptr + 1;
    uint8_t             src_byte            = 0;
    uint8_t             search_len          = 1;

    /* First, do a NULL pointer check and return immediately if it fails. */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL))
    {
        result.status = COBS_ENCODE_NULL_POINTER;
        return result;
    }

    if (src_len != 0)
    {
        /* Iterate over the source bytes */
        for (;;)
        {
            /* Check for running out of output buffer space */
            if (dst_write_ptr >= dst_buf_end_ptr)
            {
                result.status |= COBS_ENCODE_OUT_BUFFER_OVERFLOW;
                break;
            }

            src_byte = *src_read_ptr++;
            if (src_byte == 0)
            {
                /* We found a zero byte */
                *dst_code_write_ptr = search_len;
                dst_code_write_ptr = dst_write_ptr++;
                search_len = 1;
                if (src_read_ptr >= src_end_ptr)
                {
                    break;
                }
            }
            else
            {
                /* Copy the non-zero byte to the destination buffer */
                *dst_write_ptr++ = src_byte;
                search_len++;
                if (src_read_ptr >= src_end_ptr)
                {
                    break;
                }
                if (search_len == 0xFF)
                {
                    /* We have a long string of non-zero bytes, so we need
                     * to write out a length code of 0xFF. */
                    *dst_code_write_ptr = search_len;
                    dst_code_write_ptr = dst_write_ptr++;
                    search_len = 1;
                }
            }
        }
    }

    /* We've reached the end of the source data (or possibly run out of output buffer)
     * Finalise the remaining output. In particular, write the code (length) byte.
     * Update the pointer to calculate the final output length.
     */
    if (dst_code_write_ptr >= dst_buf_end_ptr)
    {
        /* We've run out of output buffer to write the code byte. */
        result.status |= COBS_ENCODE_OUT_BUFFER_OVERFLOW;
        dst_write_ptr = dst_buf_end_ptr;
    }
    else
    {
        /* Write the last code (length) byte. */
        *dst_code_write_ptr = search_len;
    }

    /* Calculate the output length, from the value of dst_code_write_ptr */
    result.out_len = dst_write_ptr - dst_buf_start_ptr;

    return result;
}


/*
 * dst_buf_ptr:    The buffer into which the result will be written
 * dst_buf_len:    Length of the buffer into which the result will be written
 * src_ptr:        The byte string to be decoded
 * src_len         Length of the byte string to be decoded
 *
 * returns:        A struct containing the success status of the decoding
 *                 operation and the length of the result (that was written to
 *                 dst_buf_ptr)
 */
cobs_decode_result omdDecodeBuf(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len)
{
    cobs_decode_result  result              = { 0, COBS_DECODE_OK };
    const uint8_t *     src_read_ptr        = src_ptr;
    const uint8_t *     src_end_ptr         = src_read_ptr + src_len;
    uint8_t *           dst_buf_start_ptr   = dst_buf_ptr;
    uint8_t *           dst_buf_end_ptr     = dst_buf_start_ptr + dst_buf_len;
    uint8_t *           dst_write_ptr       = dst_buf_ptr;
    size_t              remaining_bytes;
    uint8_t             src_byte;
    uint8_t             i;
    uint8_t             len_code;


    /* First, do a NULL pointer check and return immediately if it fails. */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL))
    {
        result.status = COBS_DECODE_NULL_POINTER;
        return result;
    }

    if (src_len != 0)
    {
        for (;;)
        {
            len_code = *src_read_ptr++;
            if (len_code == 0)
            {
                result.status |= COBS_DECODE_ZERO_BYTE_IN_INPUT;
                break;
            }
            len_code--;

            /* Check length code against remaining input bytes */
            remaining_bytes = src_end_ptr - src_read_ptr;
            if (len_code > remaining_bytes)
            {
                result.status |= COBS_DECODE_INPUT_TOO_SHORT;
                len_code = remaining_bytes;
            }

            /* Check length code against remaining output buffer space */
            remaining_bytes = dst_buf_end_ptr - dst_write_ptr;
            if (len_code > remaining_bytes)
            {
                result.status |= COBS_DECODE_OUT_BUFFER_OVERFLOW;
                len_code = remaining_bytes;
            }

            for (i = len_code; i != 0; i--)
            {
                src_byte = *src_read_ptr++;
                if (src_byte == 0)
                {
                    result.status |= COBS_DECODE_ZERO_BYTE_IN_INPUT;
                }
                *dst_write_ptr++ = src_byte;
            }

            if (src_read_ptr >= src_end_ptr)
            {
                break;
            }

            /* Add a zero to the end */
            if (len_code != 0xFE)
            {
                if (dst_write_ptr >= dst_buf_end_ptr)
                {
                    result.status |= COBS_DECODE_OUT_BUFFER_OVERFLOW;
                    break;
                }
                *dst_write_ptr++ = 0;
            }
        }
    }

    result.out_len = dst_write_ptr - dst_buf_start_ptr;

    return result;
}


void omdEncode(vc_vector* rawData, vc_vector* encodedData)
{
    int32_t startOfCurrBlock = 0;
    uint8_t numElementsInCurrBlock = 0;
    uint8_t tempVal = 0;

    uint8_t* it = vc_vector_begin(rawData);

    // Create space for first (this will be
    // overwritten once count to next 0x00 is known)
    char zeroVal = 0x00;
    vc_vector_push_back(encodedData, &zeroVal);

    while (it != vc_vector_end(rawData))
    {
        if (*it == 0x00)
        {
            // Save the number of elements before the next 0x00 into
            // the output
        	tempVal = (numElementsInCurrBlock + 1);
        	vc_vector_replace(encodedData, startOfCurrBlock, &tempVal);

            // Add placeholder at start of next block
            vc_vector_push_back(encodedData, &zeroVal);

            startOfCurrBlock = vc_vector_size(encodedData) - 1;

            // Reset count of num. elements in current block
            numElementsInCurrBlock = 0;

        }
        else
        {
        	vc_vector_push_back(encodedData, it);
            numElementsInCurrBlock++;

            if(numElementsInCurrBlock == 254) {
            	tempVal = (numElementsInCurrBlock + 1);
            	vc_vector_replace(encodedData, startOfCurrBlock, &tempVal);

                // Add placeholder at start of next block
            	vc_vector_push_back(encodedData, &zeroVal);

            	startOfCurrBlock = vc_vector_size(encodedData) - 1;

                // Reset count of num. elements in current block
                numElementsInCurrBlock = 0;
            }

        }
        it++;
    }

    // Finish the last block
    // Insert pointer to the terminating 0x00 character
    tempVal = (numElementsInCurrBlock + 1);
    vc_vector_replace(encodedData, startOfCurrBlock, &tempVal);
    vc_vector_push_back(encodedData, &zeroVal);
}

DecodeStatus omdDecode(vc_vector* encodedData, vc_vector* decodedData)
{
	vc_vector_clear(decodedData);

    size_t encodedDataPos = 0;

    while (encodedDataPos < vc_vector_size(encodedData))
    {
    	int8_t numElementsInBlock = *((uint8_t *)vc_vector_at(encodedData,encodedDataPos)) - 1;
        encodedDataPos++;

        // Copy across all bytes within block
        for (int8_t i = 0; i < numElementsInBlock; i++)
        {
            uint8_t byteOfData = *((uint8_t *)vc_vector_at(encodedData,encodedDataPos));
            if (byteOfData == 0x00)
            {
            	vc_vector_clear(decodedData);
                return ERROR_ZERO_BYTE_NOT_EXPECTED;
            }
            vc_vector_push_back(decodedData, &byteOfData);
            encodedDataPos++;
        }

        if (*((uint8_t *)vc_vector_at(encodedData,encodedDataPos)) == 0x00)
        {
            // End of packet found!
            return COBS_DECODE_SUCCESS;
            break;
        }

        // We only add a 0x00 byte to the decoded data
        // IF the num. of elements in block was less than 254.
        // If num. elements in block is max (254), then we know that
        // the block was created due to it reaching maximum size, not because
        // a 0x00 was found
        if((char)numElementsInBlock < 0xFE)
        {
        	uint8_t nullData = 0x00;
        	vc_vector_push_back(decodedData, &nullData);
        }
    }
}



