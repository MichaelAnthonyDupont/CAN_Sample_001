/*************************** C HEADER FILE ************************************
**
** Description:  Generalized Character Queue handling for serial ports or others
** Filename:     CharQueue.h
**
*******************************************************************************
**
** Copyright (c) 2014, Tahu Solutions LLC
** All rights reserved.
**
*******************************************************************************
** VERSION HISTORY:
** ----------------
** File Version:    1.0.0
** Date:            10/10/14
** Revised by:      Daniel Havener
** Description:     * RELEASE VERSION
**
** File Version:    0.1
** Date:            11/27/2013
** Revised by:      Michael A. Dupont
** Description:     * Initial Version
**
******************************************************************************/
#ifndef CHARQUEUE_INCLUDED
#define CHARQUEUE_INCLUDED
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>

/*! @defgroup CHARQUEUE CharQueue.h */
/******************************************************************************
**
** MODULES USED
**
******************************************************************************/
/******************************************************************************
**
** DEFINITIONS AND MACROS
**
******************************************************************************/
/*! @defgroup CHARQUEUE_defsmacs Definitions and Macros
 *	@ingroup CHARQUEUE
 * @{
 */

/*! @} */ /* End of defsmacs group. */
/******************************************************************************
**
** TYPEDEFS AND STRUCTURES
**
******************************************************************************/
/*! @defgroup CHARQUEUE_typestruct Typedefs and Structures
 *	@ingroup CHARQUEUE
 * @{
 */

typedef struct _QUEUE_MGT_STRUCT
{
    int16_t max_queue_size;
    int16_t enqueue_index;
    int16_t dequeue_index;
    int32_t char_count;
    uint8_t *char_queue_ptr;
    int16_t term_count;
} QUEUE_MGT_STRUCT;


#define IS_TERMINATOR(ch)  (('\n' == ch ) || ( '\r' == ch) || ('\0' == ch))

/*! @} */ /* End of typestruct group. */
/******************************************************************************
**
** PUBLIC VARIABLES
**
******************************************************************************/
#ifndef CHARQUEUE_C_SRC

#endif
/******************************************************************************
**
** PUBLIC FUNCTIONS
**
******************************************************************************/
bool CQ_AboveWaterMark(QUEUE_MGT_STRUCT *inst_ptr);
extern bool CQ_EnqueueChar(QUEUE_MGT_STRUCT *inst_ptr, uint8_t ch);
extern bool CQ_DequeueChar(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr);
extern bool CQ_Peek(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr);
extern void CQ_Flush(QUEUE_MGT_STRUCT *inst_ptr);

extern QUEUE_MGT_STRUCT *CQ_Init(QUEUE_MGT_STRUCT *inst_ptr,
                                 uint8_t *buffer_ptr,
                                 int16_t length);

extern void CQ_Test(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *string_ptr);

#ifdef __cplusplus
}
#endif
#endif /*CHARQUEUE_INCLUDED */


/******************************************************************************
**
** EOF
**
******************************************************** Template Rev: 0.0.1 */
