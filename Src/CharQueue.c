/*************************** C SOURCE FILE ************************************
**
** Description:     Generalized Character Queue handling for serial ports
** Filename:        CharQueue.c
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
#define CHARQUEUE_C_SRC
/*! @defgroup CHARQUEUE_C_SRC CharQueue.c */
/******************************************************************************
**
** MODULES USED
**
******************************************************************************/
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"


#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#include "CharQueue.h"
/******************************************************************************
**
** DEFINITIONS AND MACROS
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_defsmacs Definitions and Macros
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */

/*! @} */ /* End of defsmacs group. */
/******************************************************************************
**
** TYPEDEFS AND STRUCTURES
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_typestruct Typedefs and Structures
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */


/*! @} */ /* End of typestruct group. */
/******************************************************************************
**
** LOCAL VARIABLES
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_localvars Local Variables
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */



/*! @} */ /* End of localvars group. */

/******************************************************************************
**
** PROTOTYPES OF LOCAL FUNCTIONS
**
******************************************************************************/
#define INST_MAX_QUEUE_SIZE     inst_ptr->max_queue_size
#define INST_ENQUEUE_INDEX      inst_ptr->enqueue_index
#define INST_DEQUEUE_INDEX      inst_ptr->dequeue_index
#define INST_CHAR_COUNT         inst_ptr->char_count
#define INST_CHAR_QUEUE         inst_ptr->char_queue_ptr
#define INST_TERM_COUNT         inst_ptr->term_count

/******************************************************************************
**
** PUBLIC VARIABLES
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_pubvars Public Variables
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */

/*! @} */ /* End of pubvars group. */

/******************************************************************************
**
** PRIVATE VARIABLES
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_privvars Private Variables
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */

/*! @} */ /* End of privvars group. */
/******************************************************************************
**
** PRIVATE FUNCTIONS
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_privfuncs Private Functions
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */




/*! @} */ /* End of privfuncs group. */

/******************************************************************************
**
** PUBLIC FUNCTIONS
**
******************************************************************************/
/*! @defgroup CHARQUEUE_C_SRC_pubfuncs Public Functions
 *	@ingroup CHARQUEUE_C_SRC
 * @{
 */

/*! ** Public *****************************************************************
 *
 * \fn      void CQ_Test(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *string_ptr)
 *
 * \brief   This is a verbose way of converting a lower case string to its 
 *          upper case equivalent...just to prove that everything works.
 *          Characters in the incoming string are first capitalized and then 
 *          enqueued. The contents of the queue are then extracted and put back 
 *          into the original string (the original IS MODIFIED in this step).  
 *          The queuePeek function is used to scan for the end of the characters
 *          enqueued originally.
 *
 * \param   [inst_ptr)      QUEUE_MGT_STRUCT *
 *          [string_ptr]       uint8_t *
 *
 * \return  void
 * 
 ******************************************************************************/
void CQ_Test(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *string_ptr)
{
    bool result;
    uint8_t *p = string_ptr;
    
    if (NULL == inst_ptr)
    {
        return;
    }
    if (NULL == string_ptr)
    {
        return;
    }
    
    while ('\0' != *p)
    {
        uint8_t ch;
        
        ch = toupper(*p);
        result = CQ_EnqueueChar(inst_ptr, ch);
        p++;
    }
    
    while (1)
    {
        uint8_t ch;
        
        result = CQ_Peek(inst_ptr, &ch);
        
        if (false == result)
        {
            break;
        }
        
        result = CQ_DequeueChar(inst_ptr, &ch);
        *string_ptr = ch;
        string_ptr++;
    }
    
    CQ_Flush(inst_ptr);
}

bool CQ_AboveWaterMark(QUEUE_MGT_STRUCT *inst_ptr)
{
	return(inst_ptr->char_count >= (inst_ptr->max_queue_size/2));
}

/*! ** Public *****************************************************************
 *
 * \fn      bool CQ_EnqueueChar(QUEUE_MGT_STRUCT *inst_ptr, char ch)
 *
 * \brief   stuffs a character into the queue.
 *
 * \param  [inst_ptr]   QUEUE_MGT_STRUCT *   -- pointer to initialized instance
 *                                              struct 
 *          [ch]        char  -- character to be placed in the next
 *                               position of the queue.
 *          
 *
 * \return  bool        -- true on success
 *                      -- false on failure (queue is full)
 *
 ******************************************************************************/
bool CQ_EnqueueChar(QUEUE_MGT_STRUCT *inst_ptr, uint8_t ch)
{
    // GUARDING CHECK
    if (NULL == inst_ptr)
    {
       return(false);
    }
    
    if (INST_MAX_QUEUE_SIZE <= INST_CHAR_COUNT)
    {
        // Trying to overflow the queue?
        if (0 != INST_TERM_COUNT)
        {
            return(false);
        }
        
        // A bunch of garbage with no terminator just
        // got shoved into the queue.
        // FLUSH IT!
        __disable_irq();//portDISABLE_INTERRUPTS();//__disable_interrupt();
        CQ_Flush(inst_ptr);
        __enable_irq();//portENABLE_INTERRUPTS();//__enable_interrupt();
        
        return(false);
    }
    
    __disable_irq();//portDISABLE_INTERRUPTS();//__disable_interrupt();  // DISABLE INTERRUPTS
    INST_ENQUEUE_INDEX++;
    if (INST_MAX_QUEUE_SIZE <= INST_ENQUEUE_INDEX)
    {
        INST_ENQUEUE_INDEX = 0;
    }
    
    INST_CHAR_QUEUE[INST_ENQUEUE_INDEX] = ch;
    INST_CHAR_COUNT++;

    if (IS_TERMINATOR(ch))
    {
        INST_TERM_COUNT++;
    }
    __enable_irq();//portENABLE_INTERRUPTS();//__enable_interrupt();   // ENABLE INTERRUPTS
    
    return(true);
    
}


/*! ** Public *****************************************************************
 *
 * \fn      bool CQ_DequeueChar(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr)
 *
 * \brief   extracts a character from the queue.
 *
 * \param   [inst_ptr]  QUEUE_MGT_STRUCT * -- pointer to instance of a queue 
 *                                            struct.
 *          [ch_ptr]   char *  -- pointer to where a copy of the character 
 *                                should go.
 *          
 *
 * \return  bool        -- true on success
 *                      -- false on failure (queue is empty)
 *
 ******************************************************************************/
bool CQ_DequeueChar(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr)
{
    // GUARDING CHECK
    if (NULL == inst_ptr)
    {
        return(false);
    }
    if (0 >= INST_CHAR_COUNT)
    {
        return(false);
    }
    
    __disable_irq();//portDISABLE_INTERRUPTS();//__disable_interrupt();      // DISABLE INTERRUPTS

    INST_DEQUEUE_INDEX++;
    if (INST_MAX_QUEUE_SIZE <= INST_DEQUEUE_INDEX)
    {
        INST_DEQUEUE_INDEX = 0;
    }
    
    *ch_ptr = INST_CHAR_QUEUE[INST_DEQUEUE_INDEX];
    INST_CHAR_COUNT--;
    
    if (false != IS_TERMINATOR(*ch_ptr))
    {
       if (0 < INST_TERM_COUNT)
       {
           INST_TERM_COUNT--;
       }
    }
    
    __enable_irq();//portENABLE_INTERRUPTS();//__enable_interrupt();       // ENABLE INTERRUPTS

    return(true);
}


/*! ** Public *****************************************************************
 *
 * \fn      bool CQ_Peek(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr)
 *
 * \brief   looks for the next available character in the queue.  Do not modify
 *          queue indices or count.
 *
 * \param   [inst_ptr]  QUEUE_MGT_STRUCT * -- pointer to instance of a queue 
 *                                            struct.
 *          [ch_ptr]    char *  -- pointer to where a copy of the character 
 *                                 should go.
 *          
 *
 * \return  bool        -- true on success
 *                      -- false on failure (queue is empty)
 *
 ******************************************************************************/
bool CQ_Peek(QUEUE_MGT_STRUCT *inst_ptr, uint8_t *ch_ptr)
{
    int16_t peek_index = INST_DEQUEUE_INDEX;
    
    // GUARDING CHECK
    if (NULL == inst_ptr)
    {
       return(false);
    }
    if (0 == INST_CHAR_COUNT)
    {
       return(false);
    }
    
    peek_index++;
    if (INST_MAX_QUEUE_SIZE <= peek_index)
    {
        peek_index = 0;
    }
    
    *ch_ptr = INST_CHAR_QUEUE[peek_index];
    return(true);
}


/*! ** Public *****************************************************************
 *
 * \fn      void CQ_Flush(QUEUE_MGT_STRUCT *inst_ptr)
 *
 * \brief   resets all queue management to original state.  Dumps all the queue
 *          data (sort of).
 *
 * \param   [inst_ptr]  QUEUE_MGT_STRUCT * -- pointer to instance of a queue 
 *                      struct.
 *          
 *
 * \return  void
 *
 ******************************************************************************/
void CQ_Flush(QUEUE_MGT_STRUCT *inst_ptr)
{
    // GUARDING CHECK
    if (NULL == inst_ptr)
    {
        return;
    }
    
    __disable_irq();//portDISABLE_INTERRUPTS();//__disable_interrupt();      // DISABLE INTERRUPTS
    INST_CHAR_COUNT     = 0;
    INST_TERM_COUNT     = 0;
    INST_ENQUEUE_INDEX  = INST_MAX_QUEUE_SIZE;
    INST_DEQUEUE_INDEX  = INST_MAX_QUEUE_SIZE;
    __enable_irq();//portENABLE_INTERRUPTS();//__enable_interrupt();       // ENABLE INTERRUPTS
}


/*! ** Public *****************************************************************
 *
 * \fn      QUEUE_MGT_STRUCT *CQ_Init(QUEUE_MGT_STRUCT *inst_ptr, 
 *                                    uint8_t *buffer_ptr,
 *                                    int16_t length)
 *
 * \brief   Creates an instance of a queue using the structs and buffers passed
 *          by reference.
 *
 * \param   [inst_ptr] pointer to a pre-allocated instance stucture.
 *          [buffer_ptr] pointer to pre-allocated buffer pointer
 *          [length] integer size of the buffer
 *
 * \return  NULL if incorrect parameters passed.
 *          pointer to initialized instance (copy of caller's pointer) 
 *          on success
 *
 ******************************************************************************/

/*!
* \brief Initialize an instance of the queue handler.
*
*/

QUEUE_MGT_STRUCT *CQ_Init(QUEUE_MGT_STRUCT *inst_ptr, 
                          uint8_t *buffer_ptr, 
                          int16_t length)
{
    // GUARDING CHECK
    if (NULL == inst_ptr)
    {
        return(NULL);
    }
    if (NULL == buffer_ptr)
    {
        return(NULL);
    }
    if (0 == length)
    {
        return(NULL);
    }
    
    INST_CHAR_COUNT      = 0;
    INST_TERM_COUNT      = 0;
    INST_MAX_QUEUE_SIZE  = length;
    INST_DEQUEUE_INDEX   = length;
    INST_ENQUEUE_INDEX   = length;
    INST_CHAR_QUEUE      = buffer_ptr;
    return(inst_ptr);
}

/*! @} */ /* End of pubfuncs group. */
/******************************************************************************
**
** EOF
**
******************************************************** Template Rev: 0.0.1 */
