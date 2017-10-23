/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/main.c
  * @author  ES Team 5
  * @version V1.3.5
  * @date    October-2017
  * @brief   Main program body.
  * @note	 Uses wav files of length WaveDataFormat
  ******************************************************************************
**/


#include "main.h"
#include "mixer.h"
#include "pdm_filter.h"

extern char * WAVE_NAME;
uint8_t WaveDataLength = 0; // NOTE: Set WaveDataLength here
int16_t result[204];
/* Function Definitions */

void mixTracks(char byteArray) {
	int16_t next_track[204];
	for (int i = 0; i < 8; i++) {
		if ((byteArray & 0x01) == 1) {
			getTrack(i, next_track);
			mixer(result, next_track, result);
		}
		byteArray = byteArray >> 1;
	}
	// remove existing result wav
	// f_unlink ("result.wav");
	// create resultFile from result
//	if (f_open(&resultFile,"0:result.wav", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
//		Error_Handler();
//	}
//	// initialize header file
//	WavProcess_EncInit(44100, pHeaderBuff);
//	// write header to resultFile
//	uint32_t byteswritten = 0;
//	f_write(&resultFile, pHeaderBuff, 44, (void *)&byteswritten);
//	// Update the data length in the header of the recorded Wave
//	f_lseek(&resultFile, 0);
//	// Parse the wav file header and extract required information
//	WavProcess_HeaderUpdate(pHeaderBuff, &WaveFormat);
//	f_write(&resultFile, pHeaderBuff, 44, (void*)&byteswritten);
//
//	/* Close file and unmount MyFilesystem */
//	f_close (&resultFile);
//	f_mount(NULL, 0, 1);

	// save file to USB
}

/*
 * Takes in 3 arrays, adds the first two and saves result to the 3rd array
 */
void mixer(int16_t * track_one, int16_t * track_two, int16_t * result) {
	for (int i = 0; i < WaveDataLength; i++) {
		int a = (int) track_one[i]; // first sample (-32768..32767)
		int b = (int) track_two[i]; // second sample
		int m; // result
		// Make both samples unsigned (0..65535)
		a += 32768;
		b += 32768;

		// Pick the equation
		if ((a < 32768) || (b < 32768)) {
			// Viktor's first equation when both sources are "quiet"
			// (i.e. less than middle of the dynamic range)
			m = a * b / 32768;
		} else {
			// Viktor's second equation when one or both sources are loud
			m = 2 * (a + b) - (a * b) / 32768 - 65536;
		}

		// Output is unsigned (0..65536) so convert back to signed (-32768..32767)
		if (m == 65536) { m = 65535; };
		m -= 32768;
		result[i] = (int16_t) m;
	}
}

/* FileRead is the file with the header
 * track is the data from the file that is read
 * saves length to WaveDataLength
 */
void getTrack(uint8_t audioSampleNumber, int16_t track[]) {
	char path[] = "0:/";
	UINT bytesread = 0;
	WAVE_FormatTypeDef waveformat;

//	if(f_opendir(&Directory, path) == FR_OK)
//	 {
		char* wavefilename = "a1.wav";
		/* Open the Wave file to be read */
		if(f_open(&FileRead, wavefilename , FA_READ) != FR_OK)
		{
			 Error_Handler();
		}
		else
		{
			/* Read sizeof(WaveFormat) from the selected file */
			f_read(&FileRead, &waveformat, sizeof(waveformat), &bytesread);

			/* Set WaveDataLength to the Speech Wave length */
			WaveDataLength = waveformat.FileSize;
		}
//	 }
//	else {
//		Error_Handler();
//	}
	// fetch file from USB
	f_lseek(&FileRead, 0);
	f_read(&FileRead, &track[0], WaveDataLength, &bytesread); // want to read the whole thing to the audio array
}

// Helper Functions
/**
  * @brief  Encoder initialization.
  * @param  Freq: Sampling frequency.
  * @param  pHeader: Pointer to the WAV file header to be written.
  * @retval 0 if success, !0 else.
  */
uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t* pHeader)
{
  /* Initialize the encoder structure */
  WaveFormat.SampleRate = Freq;        /* Audio sampling frequency */
  WaveFormat.NbrChannels = 2;          /* Number of channels: 1:Mono or 2:Stereo */
  WaveFormat.BitPerSample = 16;        /* Number of bits per sample (16, 24 or 32) */
  WaveFormat.FileSize = 0x001D4C00;    /* Total length of useful audio data (payload) */
  WaveFormat.SubChunk1Size = 44;       /* The file header chunk size */
  WaveFormat.ByteRate = (WaveFormat.SampleRate * \
                        (WaveFormat.BitPerSample/8) * \
                         WaveFormat.NbrChannels);     /* Number of bytes per second  (sample rate * block align)  */
  WaveFormat.BlockAlign = WaveFormat.NbrChannels * \
                         (WaveFormat.BitPerSample/8); /* channels * bits/sample / 8 */

  /* Parse the wav file header and extract required information */
  if(WavProcess_HeaderInit(pHeader, &WaveFormat))
  {
    return 1;
  }
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeader[0] = 'R';
  pHeader[1] = 'I';
  pHeader[2] = 'F';
  pHeader[3] = 'F';

  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = 0x00;
  pHeader[5] = 0x4C;
  pHeader[6] = 0x1D;
  pHeader[7] = 0x00;

  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pHeader[8]  = 'W';
  pHeader[9]  = 'A';
  pHeader[10] = 'V';
  pHeader[11] = 'E';

  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pHeader[12]  = 'f';
  pHeader[13]  = 'm';
  pHeader[14]  = 't';
  pHeader[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pHeader[16]  = 0x10;
  pHeader[17]  = 0x00;
  pHeader[18]  = 0x00;
  pHeader[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pHeader[20]  = 0x01;
  pHeader[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pHeader[22]  = pWaveFormatStruct->NbrChannels;
  pHeader[23]  = 0x00;

  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pHeader[24]  = (uint8_t)((pWaveFormatStruct->SampleRate & 0xFF));
  pHeader[25]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 8) & 0xFF);
  pHeader[26]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 16) & 0xFF);
  pHeader[27]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 24) & 0xFF);

  /* Write the Byte Rate -----------------------------------------------------*/
  pHeader[28]  = (uint8_t)((pWaveFormatStruct->ByteRate & 0xFF));
  pHeader[29]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 8) & 0xFF);
  pHeader[30]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 16) & 0xFF);
  pHeader[31]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 24) & 0xFF);

  /* Write the block alignment -----------------------------------------------*/
  pHeader[32]  = pWaveFormatStruct->BlockAlign;
  pHeader[33]  = 0x00;

  /* Write the number of bits per sample -------------------------------------*/
  pHeader[34]  = pWaveFormatStruct->BitPerSample;
  pHeader[35]  = 0x00;

  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pHeader[36]  = 'd';
  pHeader[37]  = 'a';
  pHeader[38]  = 't';
  pHeader[39]  = 'a';

  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pHeader[40]  = 0x00;
  pHeader[41]  = 0x4C;
  pHeader[42]  = 0x1D;
  pHeader[43]  = 0x00;

  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = (uint8_t)(BufferCtl.fptr);
  pHeader[5] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[6] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[7] = (uint8_t)(BufferCtl.fptr >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  BufferCtl.fptr -=44;
  pHeader[40] = (uint8_t)(BufferCtl.fptr);
  pHeader[41] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[42] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[43] = (uint8_t)(BufferCtl.fptr >> 24);
  /* Return 0 if all operations are OK */
  return 0;
}
