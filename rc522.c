

#include "rc522.h"


/*
 * Ten ham:Write_MFRC5200
 * Chuc nang: Viet 1 byte du lieu vao thanh ghi MFRC522
 * Input:addr-> DIa chi ghi, val-> Gia tri de ghi
 * Tra ve: Khong
 */
void Write_MFRC522(uchar addr, uchar val)
{
   // uchar tmp[2];

	uint8_t txDATA[2];
	/* CS LOW */
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	//tmp[0] = Read_MFRC522(addr);
	txDATA[0] = (addr<<1)&0x7E;
	txDATA[1] = val;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)txDATA, 2, 500);
	//tmp[1] = Read_MFRC522(addr);
	/* CS HIGH */
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
}


/*
 * Ten ham:Read_MFRC522
 * Chuc nang:Doc 1 byte du lieu tu 1 thanh ghi MFRC522
 * Input:addr-> dia chi doc
 * Tra ve: Gia tri trong thanh ghi doc ve
 */
uchar Read_MFRC522(uchar addr)
{

	uint8_t txDATA[1];
	uint8_t rxDATA[1];
	/* CS LOW */
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	txDATA[0] = ((addr<<1)&0x7E) | 0x80;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)txDATA, 1, 500);
	HAL_SPI_Receive(&hspi1, (uint8_t *)rxDATA, 1, 500);
	/* CS HIGH */
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
	return rxDATA[0];

}


/*
 * Ten ham:SetBitMask
 * Chuc nang:Set bit trong mot thanh ghi MFRC522
 * Input:reg--Thanh ghi cai dat; mask--gia tri set
 * Tra ve: Khong
 */
void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}


/*
 * Ten ham:ClearBitMask
 * Chuc nang:Reset bit trong thanh ghi MFRC522
 * Input:reg--Dia chi thanh ghi; mask--Gia tri bit can clear
 * Tra ve: Khong
 */
void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Ten Ham:AntennaOn
 * Chuc Nang:Mo anten, nen co it nhat 1 ms
 * Input: khong
 * Tra ve: khong
 */
void AntennaOn(void)
{
	uchar temp;

	temp = Read_MFRC522(TxControlReg);
//	if (!(temp & 0x03))
//	{
//		SetBitMask(TxControlReg, 0x03);
//	}
	SetBitMask(TxControlReg, 0x03);
}


/*
 * Ten ham:AntennaOff
 * chuc nang:Dong Anten, nen co it nhat 1 ms
 * Input:khong
 * Tra ve: khong
 */
void AntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}


/*
 * Ten ham:ResetMFRC522
 * Chuc nang:Khoi dong lai RC522
 * Input: Khong
 * Return: Khong
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * Ten ham:InitMFRC522
 * Chuc nang:Khoi tao RC522
 * Input: Khong
 * Tra va: Khong
 */
void MFRC522_Init(void)
{
	MFRC522_Reset();

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
	Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	Write_MFRC522(TReloadRegL, 30);
	Write_MFRC522(TReloadRegH, 0);

	Write_MFRC522(TxAutoReg, 0x40);		//100%ASK
	Write_MFRC522(ModeReg, 0x3D);		//CRC Gia tri ban dau 0x6363	???

	//ClearBitMask(Status2Reg, 0x08);		//MFCrypto1On=0
	//Write_MFRC522(RxSelReg, 0x86);		//RxWait = RxSelReg[5..0]
	//Write_MFRC522(RFCfgReg, 0x7F);   		//RxGain = 48dB

	AntennaOn();		//Mo Anten
}

/*
 * Ten ham:MFRC522_ToCard
 * Chuc nang:truyen thong giua RC522 va the ISO14443
 * Input:command--lenh gui den MF522,
 *			 sendData--Du lieu gui den the bang MFRC522,
 *			 sendLen--Chieu dai du lieu gui
 *			 backData--Du lieu nhan duoc tro lai
 *			 backLen--Tra ve do dai bit cua du lieu
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint8_t *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n = 0;
    uint8_t count = 0;
    uint8_t i;

    switch (command)
    {
        case PCD_AUTHENT:		//Xac nhan the gan
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE:	// Gui du lieu FIFO
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }

    Write_MFRC522(CommIEnReg, irqEn|0x80);	//Yeu cau ngat
    ClearBitMask(CommIrqReg, 0x80);			//Clear tat ca cac bit yeu cau ngat
    SetBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, Khoi tao FIFO

	Write_MFRC522(CommandReg, PCD_IDLE);	//NO action; Huy bo lenh hien hanh	???

	// Ghi du lieu vao FIFO
    for (i=0; i<sendLen; i++)
    {
		Write_MFRC522(FIFODataReg, sendData[i]);
	}

	//chay
	Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
		SetBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts
	}

	//Cho doi de nhan duoc du lieu day du
	i = 2000;	//i tuy thuoc tan so thach anh, thoi gian toi da cho the M1 la 25ms
    do
    {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    	n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);			//StartSend=0

    if (i != 0)
    {
        if(!(Read_MFRC522(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
				status = MI_NOTAGERR;			//??
			}
			if(n==0)
			{
				count++;
			}
            if (command == PCD_TRANSCEIVE)
            {
               	n = Read_MFRC522(FIFOLevelReg);
              	lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
					*backLen = (n-1)*8 + lastBits;
				}
                else
                {
					*backLen = n*8;
				}

                if (n == 0)
                {
					n = 1;
				}
                if (n > MAX_LEN)
                {
					n = MAX_LEN;
				}

				//Doc FIFO trong cac du lieu nhan duoc
                for (i=0; i<n; i++)
                {
					backData[i] = Read_MFRC522(FIFODataReg);
				}
            }
        }
        else
        {
			status = MI_ERR;
		}

    }

   // SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}

/*
 * Ten ham:MFRC522_Request
 * Chuc nang:Phat hien the, doc loai the
 * Input:reqMode--Phat hien co the,
 *			 TagType--Loai the tra ve
 *			 	0x4400 = Mifare_UltraLight
 *				0x0400 = Mifare_One(S50)
 *				0x0200 = Mifare_One(S70)
 *				0x0800 = Mifare_Pro(X)
 *				0x4403 = Mifare_DESFire
 * Return: MI_OK neu thanh cong
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
	uchar status;
	uint8_t backBits;			//cac bit du lieu nhan duoc

	Write_MFRC522(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10))
	{
		status = MI_ERR;
	}

	return status;
}


/*
 * Ten ham:MFRC522_Anticoll
 * Chuc nang:Phat hien chong va cham, chon the va doc so serial the
 * Input:serNum--Tra ve serial the 4 byte, byte 5 la ma checksum
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
	uchar serNumCheck=0;
	uint8_t unLen;


    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
    //ClearBitMask(CollReg,0x80);			//ValuesAfterColl
	Write_MFRC522(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
	{
		//Kiem tra so serial the
		for (i=0; i<4; i++)
		{
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{
			status = MI_ERR;
		}
    }

    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1

    return status;
} 


/*
 * Ten Ham:CalulateCRC
 * Chuc nang:MFRC522 tinh toan RC522
 * Input:pIndata--Du lieu CRC vao can tinh toan,len--Chieu dai du lieu,pOutData--Ket qua tinh toan CRC
 * Tra ve: Khong
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);			//Con tro FIFO
    //Write_MFRC522(CommandReg, PCD_IDLE);

	//Ghi du lieu vao FIFO
    for (i=0; i<len; i++)
    {
		Write_MFRC522(FIFODataReg, *(pIndata+i));
	}
    Write_MFRC522(CommandReg, PCD_CALCCRC);

	// Cho cho viec tinh toan CRC hoan tat
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Doc ket qua tinh toan CRC
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}


/*
 * Ten ham:MFRC522_SelectTag
 * Chuc nang:Lua chon the, doc dung luong bo nho the
 * Input:serNum--So serial the
 * Tra ve:Dung luong the tra ve thanh cong
 */
uchar MFRC522_SelectTag(uchar *serNum)
{
	uchar i;
	uchar status;
	uchar size;
	uint8_t recvBits;
	uchar buffer[9];

	//ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
	CalulateCRC(buffer, 7, &buffer[7]);		//??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
		size = buffer[0];
	}
    else
    {
		size = 0;
	}

    return size;
}


/*
 * Ten Ham:MFRC522_Auth
 * Chuc nang:Xac nhan mat khau the
 * Input:authMode--Che do xac thuc mat khau
                 0x60 = Xac nhan phim A
                 0x61 = Xac nhan phim B
             BlockAddr--Cac khoi dia chi
             Sectorkey--Khu vuc mat khau
             serNum--So serial the, 4 byte
 * Tra ve:MI_OK neu thanh cong
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint8_t recvBits;
    uchar i;
	uchar buff[12];

	//Xac nhan lenh + Khoi dia chi + mat khau + so nhanh
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
		buff[i+2] = *(Sectorkey+i);
	}
    for (i=0; i<4; i++)
    {
		buff[i+8] = *(serNum+i);
	}
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {
		status = MI_ERR;
	}

    return status;
}


/*
 * Ten ham:MFRC522_Read
 * Chuc nang: Doc khoi du lieu
 * Input:blockAddr--Cac khoi dia chi;recvData--Khoi du lieu doc ra
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint8_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Ten ham:MFRC522_Write
 * Chuc nang:Viet khoi du lieu
 * Input:blockAddr--cac khoi dia chi;writeData--du lieu ghi
 * Tra ve: MI_OK neu thanh cong
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint8_t recvBits;
    uchar i;
	uchar buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
		status = MI_ERR;
	}

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//16 byte FIFO ghi du lieu vao
        {
        	buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }

    return status;
}


/*
 * Ten ham:MFRC522_Halt
 * CHuc nang: Dua the vao ngu dong
 * Input: Khong
 * Tra ve: Khong
 */
void MFRC522_Halt(void)
{
	uint8_t unLen;
	uchar buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	CalulateCRC(buff, 2, &buff[2]);

	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}
