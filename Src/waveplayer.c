/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @version V1.3.5
  * @date    17-February-2017
  * @brief   I2S Audio player program. 
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mixer.h"

/* Private define ------------------------------------------------------------*/
#define AUDIO_BUFFER_SIZE             64200
/* Private variables ---------------------------------------------------------*/

/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;
/* Audio Play Start variable. */
__IO uint32_t AudioPlayStart = 0;
char* WAVE_NAME = "0:a1.wav";

/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;
uint8_t audioFile[64200] = {0};
/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
int16_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 70;

/* Variable used to indicate audio mode (play, record or stop). */
extern __IO uint32_t CmdIndex;

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/* Variable used to switch play from audio sample available on USB to recorded file. */
extern uint32_t WaveRecStatus;

/* Variable to indicate USB state (start/idle) */
extern MSC_ApplicationTypeDef AppliState;

/* Private functions ---------------------------------------------------------*/

/**
  * fetches File and saves it to audioFile
  */
void fetchFileAndPlay(char byteArray)
{
  UINT bytesread = 0;
  char path[] = "0:/";
  char* wavefilename = NULL;
  WAVE_FormatTypeDef waveformat;
  for (int i = 0; i < 64200; i++) {
	  audioFile[i] = 0;
  }

  for (int i = 0; i < 8; i++) {
	if ((byteArray & 0x01) == 1) {
	/* Get the read out protection status */
	  if(f_opendir(&Directory, path) == FR_OK)
	  {
		 BSP_LED_Off(LED3);

		 // NOTE: Hardcoded for now - make separate function?
		 if (i == 0) {
			 wavefilename = "0:a0.wav";
		 }
		 else if (i == 1) {
			 wavefilename = "0:a1.wav";
		 }
		 else if (i == 2) {
			 wavefilename = "0:a2.wav";
		 }
		 else if (i == 3) {
			 wavefilename = "0:a3.wav";
		 }
		 else {
			 wavefilename = "0:a0.wav";
		 }

		/* Open the Wave file to be played */
		if(f_open(&FileRead, wavefilename , FA_READ) != FR_OK)
		{
		  BSP_LED_On(LED5);
		}
		else
		{
		  /* Read sizeof(WaveFormat) from the selected file */
		  f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);

		  /* Set WaveDataLength to the Speech Wave length */
		  WaveDataLength = waveformat.FileSize;

		  /* Play the Wave */
		  UINT bytesread = 0;
		  f_lseek(&FileRead, 0);
		  uint8_t tempFile[WaveDataLength/2];
		  f_read(&FileRead, &tempFile, WaveDataLength/2, &bytesread);
		  mixFiles(audioFile, tempFile);

		  f_read(&FileRead, &tempFile, WaveDataLength/2, &bytesread);
		  mixFiles(&audioFile[WaveDataLength/2], tempFile);

		  /* Close file */
		  f_close(&FileRead);
		}
	  }
	  else {
		  BSP_LED_On(LED3);
	  }
	}
	byteArray = byteArray >> 1;
  }
  playFile(audioFile);
}

// plays array passed through as a parameter
void playFile(uint8_t * audioFile) {
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, Volume, 32000) == AUDIO_OK)
	{
	  BSP_AUDIO_OUT_Play((uint16_t*)&audioFile[0], WaveDataLength);
	}
}

void mixFiles(uint8_t * audioFile, uint8_t * tempFile) {
	for (int i = 0; i < WaveDataLength/2; i++) {
		int a = (int) audioFile[i]; // first sample (-32768..32767)
		int b = (int) tempFile[i]; // second sample
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
			m = 2 * (a + b) - (a * b) / 32768 - 65535;
		}

		// Output is unsigned (0..65536) so convert back to signed (-32768..32767)
		if (m == 65535) { m = 65535; };
		m -= 32768;
		audioFile[i] = (uint8_t) m;
	}
}

/* --------Callbacks implementation---------- */

/**
  * Manages the DMA Half Transfer complete interrupt.
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* Calculates the remaining file size and new position of the pointer.
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
//  BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&audioFile[0], WaveDataLength);
}

/**
* Manages the DMA FIFO error interrupt.
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
}

// Wave Player Callback used by USB Process
void WavePlayer_CallBack(void)
{
  if(AppliState != APPLICATION_IDLE)
  {
    /* Reset the Wave player variables */
    RepeatState = REPEAT_ON;
    AudioPlayStart = 0;
    PauseResumeStatus = RESUME_STATUS;
    WaveDataLength =0;
    PressCount = 0;

    /* Stop the Codec */
    if(BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW) != AUDIO_OK)
    {
      Error_Handler();
    }

    /* Turn OFF LED3, LED4 and LED6 */
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED6);
  }
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
