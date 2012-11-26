/**
  ******************************************************************************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Description of the USB Audio_Streaming Demo.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


Example description
===================
The USB-FS-Device Audio Streaming demo gives examples of how to use the STM32F10x5/7 
USB-OTG peripheral in Device mode to communicate with the PC host in the isochronous 
transfer mode.
It provides a demo of the correct method for configuring an isochronous
endpoint, receiving or transmitting data from/to the host.
This demo provides possibilities to configure high audio quality streaming and 
low CPU charge using:
 - The USB-OTG peripheral with its dedicated RAM. 
 - The audio-class I2S peripheral and the available dedicated PLLs.
 - The DMA for Audio transfer from USB Memory to the I2S peripheral.

NOTE:
 - LED2 and LED3 of the STM3210C-EVAL evaluation board are configured to toggle
   periodically in order to check each time that firmware is still running
   correctly.

More details about this Demo implementation is given in the User manual 
"UM0424 STM32F10xxx USB development kit", available for download from the ST
microcontrollers website: www.st.com/stm32


Directory contents
==================
 + \inc: contains the Demo firmware header files
 + \EWARM: contains preconfigured projects for EWARM toolchain
 + \RIDE: contains preconfigured projects for RIDE toolchain
 + \MDK-ARM: contains preconfigured projects for MDK-ARM toolchain 
 + \TASKING: contains preconfigured projects for TASKING toolchain
 + \TrueSTUDIO: contains preconfigured projects for TrueSTUDIO toolchain           
 + \src: contains the Demo firmware source files


Hardware environment
====================
This example runs on STMicroelectronics STM3210C-EVAL evaluation board and can be
easily tailored to any other hardware.
Basically the demo provides the configuration with 25 MHz wich allows getting 
high accuracy on both I2S and USB clock for only some audio frequencies.
For higher audio quality (in combination with an acceptable accuracy on the USB 
clock) on all audio frequencies, a specific external clock source may be used instead 
of the usual 25MHz crystal. Thus a 14.7456MHz crystal can be used to obtain this 
quality.

For each Audio output frequency (96KHz, 48KHz, ... ,8KHz) the PLL multipliers and 
dividers values shall be correctly configured in order to have optimum accuracy 
on both I2S clock and USB clock.
This demo provides the optimum configuration for the following audio frequencies
(no need for modifying the PLL configuration for these audio frequencies):

 When using 25 MHz crystal the PLLs configuration is optimum for these frequencies:
  - 16 KHz 
  -  8 KHz
 When using 14.7456 MHz crystal the PLLs configuration is optimum for these frequencies:  
  - 96 KHz
  - 48 KHz
  - 32 KHz
  - 16 KHz
  -  8 KHz
 
To select the audio frequency, uncomment the relative define in usb_conf.h file. 
 
For other frequencies, PLL1MUL, PLL2MUL, PLL3MUL, PREDIV1 and PREDIV2 values have
to be readjusted to get the optimum value on the I2S and USB clocks.


NOTE:
 The audio frequencies leading to non integer number of data (44.1KHz, 22.05KHz, 
 11.025KHz...) will not allow an optimum audio quality since one data will be lost
 every two frame.

  - STM3210C-EVAL Set-up 
     - Jumper JP17 (I2C_SCK) should be connected.
     - Jumper JP10 (I2S_MCK) should be connected in position 1-2. 

How to use it
=============
 + EWARM
    - Open the AudioStreaming.eww workspace.
    - In the workspace toolbar select the project config:
        - STM3210C-EVAL-25MHz: to configure the project for boards with 25MHz external clock.
        - STM3210C-EVAL-14MHz: to configure the project for boards with 14.7456MHz external clock.
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 + MDK-ARM
    - Open the AudioStreaming.Uv2 project
    - In the build toolbar select the project config:
        - STM3210C-EVAL-25MHz: to configure the project for boards with 25MHz external clock.
        - STM3210C-EVAL-14MHz: to configure the project for boards with 14.7456MHz external clock.
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)    

 + RIDE
    - Open the AudioStreaming.rprj project.
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210C-EVAL-25MHz: to configure the project for boards with 25MHz external clock.
        - STM3210C-EVAL-14MHz: to configure the project for boards with 14.7456MHz external clock.
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 + TASKING
    - Open TASKING toolchain.
      - Click on File->Import, select General->'Existing Projects into Workspace' 
        and then click "Next". 
      - Browse to TASKING workspace directory and select the project: 
        - STM3210C-EVAL-25MHz: to configure the project for boards with 25MHz external clock.
        - STM3210C-EVAL-14MHz: to configure the project for boards with 14.7456MHz external
    - Rebuild all project files: Select the project in the "Project explorer" 
        window then click on Project->build project menu.
    - Run program: Select the project in the "Project explorer" window then click 
        Run->Debug (F11)

 + TrueSTUDIO
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace'
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project:  
        - STM32F10C-EVAL-25MHz: to load the project for boards with 25MHz external clock.
        - STM32F10C-EVAL-14MHz: to load the project for boards with 14.7456MHz external clock.
    - Rebuild all project files: Select the project in the "Project explorer" window
      then click on Project->build project menu.
    - Run program: Select the project in the "Project explorer" window then click 
      Run->Debug (F11)

************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE******
