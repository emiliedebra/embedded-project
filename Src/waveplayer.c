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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define AUDIO_BUFFER_SIZE             64200
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;
/* Audio Play Start variable. 
   Defined as external in main.c*/
__IO uint32_t AudioPlayStart = 0;
char* WAVE_NAME = "0:a1.wav";
//extern int16_t * result;
/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;
// uint8_t tempFile[AUDIO_BUFFER_SIZE]; // this is what is going to next be added to audioFile
// uint8_t audioFile[AUDIO_BUFFER_SIZE]; // this is what we play in the end
/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
int16_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 70;

/* Variable used to indicate audio mode (play, record or stop). */
/* Defined in main.c */
extern __IO uint32_t CmdIndex;

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/* Variable used to switch play from audio sample available on USB to recorded file. */
/* Defined in waverecorder.c */
extern uint32_t WaveRecStatus;

/* Variable to indicate USB state (start/idle) */
/* Defined in main.c */
extern MSC_ApplicationTypeDef AppliState;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Plays Wave from a mass storage.
  * @param  AudioFreq: Audio Sampling Frequency
  * @retval None
*/
void WavePlayBack(uint32_t AudioFreq)
{ 
  UINT bytesread = 0;
  
  /* Start playing */
  AudioPlayStart = 1;
  RepeatState = REPEAT_ON;
  
  /* Initialize Wave player (Codec, DMA, I2C) */
  if(WavePlayerInit(AudioFreq) != 0)
  {
    Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = WaveDataLength - bytesread;
  
  /* Start playing Wave */
//  BSP_AUDIO_OUT_Play((uint16_t*)&result[0], sizeof(result)/16);
   BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
//   BSP_LED_Toggle(LED6);
   PauseResumeStatus = RESUME_STATUS;
   PressCount = 0;

  /* Check if the device is connected.*/
  while((AudioRemSize != 0) && (AppliState != APPLICATION_IDLE))
  {
    /* Test on the command: Playing */
    if(CmdIndex == CMD_PLAY)
    {
      if(PauseResumeStatus == PAUSE_STATUS)
      {
        /* Stop Toggling LED2 to signal Pause */
        BSP_LED_Off(LED4);
        /* Pause playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }
      else if(PauseResumeStatus == RESUME_STATUS)
      {
//        /* Toggling LED6 to signal Play */
        BSP_LED_Toggle(LED6);
//        /* Resume playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }

      bytesread = 0;

      if(buffer_offset == BUFFER_OFFSET_HALF)
      {

        f_read(&FileRead,
               &Audio_Buffer[0],
               AUDIO_BUFFER_SIZE/2,
               (void *)&bytesread);

          buffer_offset = BUFFER_OFFSET_NONE;
      }

      if(buffer_offset == BUFFER_OFFSET_FULL)
      {
        f_read(&FileRead,
               &Audio_Buffer[AUDIO_BUFFER_SIZE/2],
               AUDIO_BUFFER_SIZE/2,
               (void *)&bytesread);

          buffer_offset = BUFFER_OFFSET_NONE;
      }
      if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2))
      {
        AudioRemSize -= bytesread;
      }
      else
      {
        AudioRemSize = 0;
      }
    }
    else
    {
      /* Stop playing Wave */
      WavePlayerStop();
      /* Close file */
      f_close(&FileRead);
      AudioRemSize = 0;
      RepeatState = REPEAT_ON;
      break;
    }
  }
#ifdef PLAY_REPEAT_DISABLED 
  RepeatState = REPEAT_OFF;
  /* Stop playing Wave */
  WavePlayerStop();
  /* Close file */
  f_close(&FileRead);
  /* Test on the command: Playing */
  if(CmdIndex == CMD_PLAY)
  {
    BSP_LED_On(LED4);
  }
#else 
  RepeatState = REPEAT_ON;
  AudioPlayStart = 0;
  /* Stop playing Wave */
  WavePlayerStop();
  /* Close file */
  f_close(&FileRead);
#endif /* PLAY_REPEAT_DISABLED */
}

/**
  * @brief  Pauses or Resumes a played Wave.
  * @param  state: Player state: Pause, Resume or Idle
  * @retval None
  */
void WavePlayerPauseResume(uint32_t wState)
{
  if(wState == PAUSE_STATUS)
  {
    BSP_AUDIO_OUT_Pause();   
  }
  else
  {
    BSP_AUDIO_OUT_Resume();   
  }
}

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}
 
/**
* @brief  Initializes the Wave player.
* @param  AudioFreq: Audio sampling frequency
* @retval None
*/
int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  return(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq));  
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
  //BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
  
  /* Could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @brief  fetches File and saves it to audioFile
  * @param  None
  * @retval None
  */
void fetchFile(char byteArray)
{
  UINT bytesread = 0;
  char path[] = "0:/";
  char* wavefilename = NULL;
  WAVE_FormatTypeDef waveformat;
  int16_t audioFile[64200];
  if(f_opendir(&Directory, path) == FR_OK)
  {
	 BSP_LED_Off(LED3);
//		 int someInt = i;
	wavefilename = "0:a0.wav";
	/* Open the Wave file to be played */
	if(f_open(&FileRead, wavefilename , FA_READ) != FR_OK)
	{
	  BSP_LED_On(LED5);
	}
	else
	{
	  /* Read sizeof(WaveFormat) from the selected file */
	  f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);

	  /* Set WaveDataLenght to the Speech Wave length */
	  WaveDataLength = waveformat.FileSize;

	  /* Play the Wave */
	  //WavePlayBack(waveformat.SampleRate);
	  UINT bytesread = 0;
	  f_lseek(&FileRead, 0);
	  f_read(&FileRead, &audioFile, WaveDataLength, &bytesread);

	}
  }
  for (int i = 0; i < 8; i++) {
	if ((byteArray & 0x01) == 1) {
	/* Get the read out protection status */
	  if(f_opendir(&Directory, path) == FR_OK)
	  {
		 BSP_LED_Off(LED3);
//		 int someInt = i;
		wavefilename = "0:a1.wav";
		/* Open the Wave file to be played */
		if(f_open(&FileRead, wavefilename , FA_READ) != FR_OK)
		{
		  BSP_LED_On(LED5);
		}
		else
		{
		  /* Read sizeof(WaveFormat) from the selected file */
		  f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);

		  /* Set WaveDataLenght to the Speech Wave length */
		  WaveDataLength = waveformat.FileSize;

		  /* Play the Wave */
		  //WavePlayBack(waveformat.SampleRate);
		  UINT bytesread = 0;
		  f_lseek(&FileRead, 0);
		  int16_t tempFile[WaveDataLength/4];
		  f_read(&FileRead, &tempFile, WaveDataLength/4, &bytesread);
		  mixFiles(audioFile, tempFile);

		  f_read(&FileRead, &tempFile, WaveDataLength/4, &bytesread);
		  mixFiles(&audioFile[WaveDataLength/4], tempFile);

		  f_read(&FileRead, &tempFile, WaveDataLength/4, &bytesread);
		  mixFiles(&audioFile[WaveDataLength/2], tempFile);

		  f_read(&FileRead, &tempFile, WaveDataLength/4, &bytesread);
		  mixFiles(&audioFile[3*WaveDataLength/4], tempFile);
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

void playFile(int16_t * audioFile) {
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, 32000) == AUDIO_OK)
	{
	  BSP_AUDIO_OUT_Play((uint16_t*)&audioFile[0], WaveDataLength);
	}
}

void mixFiles(int16_t * audioFile, int16_t * tempFile) {
	for (int i = 0; i < WaveDataLength/2; i++) {
			int a = (int) audioFile[i]; // first sample (-32768..32767)
			int b = (int) tempFile[i]; // second sample
			int m; // result
//			// Make both samples unsigned (0..65535)
			a += 32768;
			b += 32768;
//
//			// Pick the equation
			if ((a < 32768) || (b < 32768)) {
				// Viktor's first equation when both sources are "quiet"
				// (i.e. less than middle of the dynamic range)
				m = a * b / 32768;
			} else {
				// Viktor's second equation when one or both sources are loud
				m = 2 * (a + b) - (a * b) / 32768 - 65535;
			}
//
//			// Output is unsigned (0..65536) so convert back to signed (-32768..32767)
			if (m == 65535) { m = 65535; };
			m -= 32768;
			audioFile[i] = (int16_t) m;
		}
}
/**
  * @brief Wave player.
  * @param  None
  * @retval None
  */
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
