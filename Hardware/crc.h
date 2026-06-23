#ifndef CRC_H
#define CRC_H
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);
#endif

