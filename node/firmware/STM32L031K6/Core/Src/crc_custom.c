#include "main.h"

int8_t calculate_CRC8(const uint8_t *data, uint8_t length, uint8_t polynomial, uint8_t* result )
{
	// Reset the CRC calculation unit
	LL_CRC_ResetCRCCalculationUnit(CRC);
	LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_8B);
	LL_CRC_SetPolynomialCoef(CRC, polynomial);

	// Feed data to CRC calculation
	for (uint32_t i = 0; i < length; i++)
	{
		LL_CRC_FeedData8(CRC, data[i]);
	}

	// Return the CRC result (only the last 8 bits are relevant for CRC8)
	*result = LL_CRC_ReadData8(CRC);

	return 0;
}
