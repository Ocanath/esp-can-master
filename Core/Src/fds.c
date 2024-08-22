/*
 * fds.c
 *
 *  Created on: Sep 3, 2023
 *      Author: Ocanath
 */
#include "fds.h"
#define LAST_PAGE 63


/*
 * PROCEDURE FOR SETTING VALUES IN FLASH MANUALLY
 * (THROUGH SWD)
 *
 * 1. Connect to the device in STM32CubeProgrammer
 * 2. Hit 'Read All'
 * 3. Navigate to 0x08000000 + LAST_PAGE*0x800
 * 			= 0x800F800
 * 		-note: will have to scroll, for writing to work. sadly
 * 4. Click the address you want and manually enter the data you want, as a u32.
 *
 *
 *You can also edit the default settings in settings.c and flash an erased device.
 *
 *
 * */

/**/
uint32_t get_page_start_address(uint32_t page_number)
{
	uint32_t base_addr = 0x08000000;
	return base_addr + page_number*0x800;	//a page on stm32 has 0x800 addresses allocated
}

/**/
uint32_t m_write_flash(uint64_t * words, int num_words)
{

	uint32_t error = 0;

	HAL_FLASH_Unlock();

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;


	uint32_t StartPageAddress = get_page_start_address(LAST_PAGE);	//last page on the STM32F301x6/8 is 31, causze there only 32 pages BISSSHHH

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = LAST_PAGE;
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		error = HAL_FLASH_GetError ();
	}

	if(words == NULL)	//null pointer check
		return 0xFFFFFFFF;	//-1 for null pointer, i suppossseee
	for(int word_idx = 0; word_idx < num_words;)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, words[word_idx]) == HAL_OK)
		{
			StartPageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
			word_idx++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory*/
			error = HAL_FLASH_GetError ();
			break;
		}
	}

	HAL_FLASH_Lock();

	return error;
}


/*pretty much just a specific re-implementation of m_mcpy that uses (__IO uint32_t *) pointer type, which
 * i don't know what the compiler does differently with that but eh we doin it*/
void m_read_flash(uint32_t * p_dest, uint32_t num_words)
{
	uint32_t start_address = get_page_start_address(LAST_PAGE);
	for(int word = 0; word < num_words; word++)
	{
		*p_dest = *(__IO uint32_t *)start_address;
		start_address += sizeof(uint32_t);//i.e. 4
		p_dest++;
	}
}

uint8_t is_page_empty(uint32_t num_words_expected)
{
	uint32_t start_address = get_page_start_address(LAST_PAGE);
	for(int word = 0; word < num_words_expected; word++)
	{
		uint32_t word = *(__IO uint32_t *)start_address;
		start_address += sizeof(uint32_t);//i.e. 4
		if(word != 0xFFFFFFFF)
		{
			return 0;
		}
	}
	return 1;
}


/*compare two 'files' for equality*/
uint8_t m_memcompare(uint32_t * p1, uint32_t * p2, uint32_t num_words)
{
	for(int widx = 0; widx < num_words; widx++)
	{
		if(p1[widx] != p2[widx])
		{
			return 0;
		}
	}
	return 1;
}
