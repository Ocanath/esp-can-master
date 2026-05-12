/*
 * fds.h
 *
 *  Created on: Sep 3, 2023
 *      Author: Ocanath
 */

#ifndef INC_FDS_H_
#define INC_FDS_H_
#include "init.h"

/*block erases the last page of memory and writes whatever shit you like to it.
 *
 * Note: calling this with words = NULL and num_words = 0
 * does a page erase on the whole allocated page of memory.
 *
 * Neat!
 * */
uint32_t m_write_flash(uint64_t * words, int num_words);

/*
 * really simple block read. behaves as advertised, targeting the very last page of memory
 * */
void m_read_flash(uint32_t * p_dest, uint32_t num_words);

/*checks the range of words you're expecting and simply checks if they're erased*/
uint8_t is_page_empty(uint32_t num_words_expected);


/*compare two regions of memory*/
uint8_t m_memcompare(uint32_t * p1, uint32_t * p2, uint32_t num_words);


#endif /* INC_FDS_H_ */
