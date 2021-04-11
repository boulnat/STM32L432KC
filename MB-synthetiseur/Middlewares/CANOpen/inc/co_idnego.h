/*
* co_idnego.h - contains defines for LSS services
*
* Copyright (c) 2019 emotas embedded communication GmbH
*
*-------------------------------------------------------------------
* $Id: co_idnego.h 31557 2020-04-09 08:36:36Z boe $
*
*-------------------------------------------------------------------
*
*
*
*/

/**
* \brief defines for nodeid negotiation services
*
* \file co_idnego.h - contains defines for nodeid negotiation services
*/

#ifndef CO_IDNEGO_H
#define CO_IDNEGO_H 1

#include <co_datatype.h>


/* datatypes */

/**
 * \brief IDNEGO services for indication functions
 */
typedef enum {
	CO_IDNEGO_SERVICE_DOUBLE_ID, 	/**< IDNEGO service indication double id */
	CO_IDNEGO_SERVICE_USED_ID,		/**< IDNEGO service indication for used id's */
	CO_IDNEGO_SERVICE_REQ_ID		/**< IDNEGO service indication for request id */
} CO_IDNEGO_SERVICE_T;


/**
 * \brief function pointer to IDNEGO indication
 * \param service - answer for service IDNEGO
 * \param para - new bitrate / pending node id (only for CO_LSS_SERVICE_STORE)
 * 		1000, 500, ... 10	standard bitrates
 * 		0					autobaud
 * 		0					table specific, values in pErrCode and pErrSpec)
 * \param pErrCode - pointer to error code
 * \param pErrSpec - pointer to error spec
 * 
 * \retval 0 - success
 * \retval 1 - store not supported
 * \retval 2 - media access error
 * \retval 255 - implementation specific (value in parameter pErr)
 */
typedef void (* CO_EVENT_IDNEGO_T)(CO_IDNEGO_SERVICE_T service,
		UNSIGNED8* pPara);	 /*lint !e960 customer specific parameter names */



EXTERN_DECL RET_T coIdNegoInit(void);
EXTERN_DECL RET_T coEventRegister_IDNEGO(CO_EVENT_IDNEGO_T pFunction);


#endif /* CO_IDNEGO_H */
