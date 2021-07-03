/*
 * Project: Lightweight millisecond tracking library
 * Author: Zak Kemble, contact@zakkemble.net
 * Copyright: (C) 2018 by Zak Kemble
 * License: GNU GPL v3 (see License_GPL-3.0.txt) or MIT (see License_MIT.txt)
 * Web: http://blog.zakkemble.net/millisecond-tracking-library-for-avr/
 */

#ifndef MILLIS_H_
#define MILLIS_H_

/**
 * Milliseconds data type \n
 * Data type				- Max time span			- Memory used \n
 * unsigned char			- 255 milliseconds		- 1 byte \n
 * unsigned int			- 65.54 seconds			- 2 bytes \n
 * unsigned long			- 49.71 days			- 4 bytes \n
 * unsigned long long	- 584.9 million years	- 8 bytes
 */
typedef unsigned long millis_t;
typedef unsigned long micros_t;

extern millis_t milliseconds;  // = 0;

#define MILLIS_TIMER0 0 /**< Use timer0. */
#define MILLIS_TIMER1 1 /**< Use timer1. */
#define MILLIS_TIMER2 2 /**< Use timer2. */

#ifndef MILLIS_TIMER
#define MILLIS_TIMER MILLIS_TIMER2 /**< Which timer to use. */
#endif

#ifndef ARDUINO
/**
 * Alias of millis_get().
 *
 * @note Not availble for Arduino since millis() is already used.
 */
#define millis() millis_get()
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise, must be called before anything else!
 *
 * @return (none)
 */
void millis_init(void);

/**
 * Get milliseconds.
 *
 * @return Milliseconds.
 */
unsigned long millis_get(void);

/**
 * Get milliseconds.
 *
 * @return Milliseconds.
 */
unsigned long micros_get(void);

/**
 * Turn on timer and resume time keeping.
 *
 * @return (none)
 */
void millis_resume(void);

/**
 * Pause time keeping and turn off timer to save power.
 *
 * @return (none)
 */
void millis_pause(void);

/**
 * Reset milliseconds count to 0.
 *
 * @return (none)
 */
void millis_reset(void);

/**
 * Add time.
 *
 * @param [ms] Milliseconds to add.
 * @return (none)
 */
void millis_add(unsigned long ms);

/**
 * Subtract time.
 *
 * @param [ms] Milliseconds to subtract.
 * @return (none)
 */
void millis_subtract(unsigned long ms);

#ifdef __cplusplus
}
#endif

#endif /* MILLIS_H_ */
