



#ifndef __ARDUINO_MAPPING_H__
#define __ARDUINO_MAPPING_H__

#include "ioc.h"

#define MOD_DIO_0 		IOID_1
#define MOD_DIO_1		IOID_14
#define MOD_DIO_2		IOID_15
#define MOD_DIO_3		IOID_4
#define MOD_DIO_4		IOID_5
#if (cc26xx==1)
#define MOD_DIO_5		IOID_0
#else
#define MOD_DIO_5		IOID_22
#endif /* cc26xx==1 */
#define MOD_DIO_6		IOID_29
#define MOD_DIO_7		IOID_30
#define MOD_DIO_8		IOID_13
#if SEEED_MOTOR_BOARD_V2==1
#define MOD_DIO_9		IOID_29 	/* H/W muxed with MOD_DIO_6 */
#else
#define MOD_DIO_9		IOID_UNUSED
#endif /* SEEED_MOTOR_BOARD_V2==1 */
#define MOD_DIO_10		IOID_21
#define MOD_DIO_11		IOID_20
#define MOD_DIO_12		IOID_19
#define MOD_DIO_13		IOID_18
#define MOD_DIO_SDA		IOID_UNUSED /* H/W muxed with MOD_AIO_4 */
#define MOD_DIO_SCL		IOID_UNUSED /* H/W muxed with MOD_AIO_5 */

#define MOD_AIO_0		IOID_23
#define MOD_AIO_1		IOID_24
#define MOD_AIO_2		IOID_25
#define MOD_AIO_3		IOID_26
#define MOD_AIO_4		IOID_27
#define MOD_AIO_5		IOID_28

#endif /* __ARDUINO_MAPPING_H__ */