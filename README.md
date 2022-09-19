# D-Box: DMA-enabled Compartmentalization for Embedded Applications
This is the repository for D-Box paper presented at NDSS 2022 Symposium.

https://www.ndss-symposium.org/ndss-paper/auto-draft-258/


The security design issues of the FreeRTOS Kernel, described in this paper, 
were responsibly reported to FreeRTOS developers (AWS Amazon). 
Check the official Amazon Security update on the following link:
https://www.freertos.org/security/security_updates.html


# Citing our paper

If your research find one or several components of D-Box useful, please use the following citation:

```bibtex
@inproceedings{meraDBoxDMAenabledCompartmentalization2022,
  title = {D-{{Box}}: {{DMA-enabled Compartmentalization}} for {{Embedded Applications}}},
  shorttitle = {D-{{Box}}},
  booktitle = {Proceedings 2022 {{Network}} and {{Distributed System Security Symposium}}},
  author = {Mera, Alejandro and Chen, Yi Hui and Sun, Ruimin and Kirda, Engin and Lu, Long},
  year = {2022},
  doi = {10.14722/ndss.2022.24053},
  isbn = {1-891562-74-6}
}

```

# Getting started
D-Box requires a development board compatible wit the FreeRTOS-MPU RTOS. 
Particularly we implemented our solution for the ST NUCLEO-L152RE (https://www.st.com/en/evaluation-tools/nucleo-l152re.html).
In addition, we used the official STM32CubeIDE to compile and debug our solution (https://www.st.com/en/development-tools/stm32cubeide.html).

In our paper, we also used Percepio Tracealyzer (https://percepio.com/tracealyzer/) for measuring
the core processor usage. This analysis is included in the macrobenchmark of our evaluation section.
Using Percepio is only necessary for evaluation purposes and it is not part of our solution.
Students and scholars can get academic licenses of Percepio Tracealyzer if interested in reproducing all 
our experiments.

The dummy slave device that we used in the evaluation section uses the NUCLEO-F103rb. However, it can be replaced by any 
slave device compatible with SPI or I2C protocols.

Use the following commands to clone this repository:
```bash
git clone https://github.com/RiS3-Lab/D-Box.git
cd D-Box
```



## D-Box Directory structure
After cloning the directory structure will be as follows:

```
.
├── FreeRTOS
│   ├── Demo
│   │   ├── CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark          # Baseline Macrobenchmark 
│   │   ├── CORTEX_MPU_M3_NUCLEO_L152RE_GCC_dma_RX             # Macrobenchmark D-Box Implementation
│   │   ├── FreeRTOS-BOX-MODBUS_PID                            # Case study with D-Box Implementation*
│   │   ├── FreeRTOS-MODBUS_PID                                # Baseline case study original Firmware*
│   └── Source                                                 # Modified FreeRTOS-MPU Kernel
|       ├── container.c                                        # Main file of D-Box implementation
|       └── include
|           └── application_defined_privileged_functions.h     # D-Box syscalls extensions
|    
├── LICENSE
├── MODBUS-LIB                                                 # Library used in our PLC case study
├── README.md                                                  # This file
├── TRACE-LIB                                                  # Percepio library (optional)
└── TestI2C-F103                                               # Dummy slave source code 
    

```
### Note: 
* The source directory contains the complete FreeRTOS kernel. We only listed the files that we modified or added to implement D-Box.
* The source code of the case study is only a fragment of a complete PLC program as described in 
our paper. We cannot provide the complete PLC program since it contains proprietary routines.



## Compiling and testing
The projects under the Demo folder are compatible with STM32CubeIDE. To open a project
open the IDE and import the project selecting the "Open project from the file system" option 
under the "File" menu of the IDE.

For the Macrobenchmark baseline project the peripheral operations can be activated changing the pre-processor 
directives in the folowing file: 
https://github.com/RiS3-Lab/D-Box/blob/175b3497eba7399d9b0bcca70c210b89a3e4c082/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/Config/FreeRTOSConfig.h#L82

Similarly, for the Marcobenchmark D-Box implementation the tests can be activated in the follwing file:
https://github.com/RiS3-Lab/D-Box/blob/175b3497eba7399d9b0bcca70c210b89a3e4c082/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_dma_RX/Config/FreeRTOSConfig.h#L92

Please refer to our paper for the complete setup for measuring energy and performance metrics.

If you need support or have any question, please open an issue on this repository.

## Disclaimer
D-Box implementation is only a proof of concept. We recommend using its code with precaution and under your responsibility for production settings. For more details, also read the license file.






