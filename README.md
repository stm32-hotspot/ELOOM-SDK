# eLooM SDK based on FreeRTOS

![latest tag](https://img.shields.io/github/v/tag/stm32-hotspot/eloom-sdk.svg?color=brightgreen)


**eLooM SDK** is a collection of embedded applications based on **eLooM** framework and components.
It demonstrates how to use the eLooM framework providing ready-to-use examples.
The package is intended for expert firmware developers who already are familiar with:

- Real-time operating system. Here, FreeRTOS is used.
- Principle of object-oriented programming (like inheritance, polymorphism, virtual function).

**eLooM** stands for **embedded Light object oriented fraMework**. 
It is an application framework designed for STM32, and specifically for **soft real-time**, **event driven**, **multitasking**, and **low-power** embedded applications.

A developer can use the package to explore and learn the features and the potential of an eLooM based application for STM32 in term of code **modularity**, **reusability** and **efficiency** (memory footprint and performance).

The eLooM SDK has been built on top of eLooM application-level modules called **eLooM components**, that a final user can reuse and extend easily to build up its custom application. 
The components available are:

- DPU - Digital Processing Units
- EMData
- SensorManager
- SensorStreamerCLI

These application modules adopt state-of-the-art design patterns, and they support natively low-power modes.

The package supports different boards from different STM32 families, to demonstrate eLooM portability.
For each board there are one or more projects available from the following list:

- HelloWorld
- SensorManagerDemo
- SensorStreamerCLI
- DpuDemo

A detailed README describing the application main features is available for each example.

In the above list, demos are in difficulty order. 
If you are no already familiar with eLooM, we suggest you to exploit the SDK by running the example in the order described above (first starting from HelloWorld, than Sensor demo and finally Dpu demo).

The **HelloWorld** project shows the minimum code and MCU resources needed to port the eLooM framework on a target board.
It can also be used as a template to start a new project.

The **SensorManagerDemo** and **SensorStreamerCLI** show how to use an eLooM component to quickly develop a more complex application. 
They demonstrate the main features of **SensorManager** and **EMData** eLooM components.

The **DpuDemo** is built on top of the previous project (SensorManagerDemo and/or SensorStreamerCLI) and shows how to add processing capabilities by using another eLooM component, the **Digital Processing Unit (DPU)**. 

Firmware is designed as a set of concurrent tasks extending a common interface, where each task uses one or more services that, in turn, use one or more drivers.
We call it **Managed Task**, and this design pattern is described in the framework documentation.
All the demos across all the boards share many source code at application level thanks to the design of eLooM. 
Each demo application can be a starting point for new developments.

eLooM has been developed in the last years and used by ST customers and also in different Function Packs by ST to showcase complex use cases in AI, condition monitoring and sensor fusion domains.
Here some examples of FP based on eLooM:

- [FP-AI-MONITOR1 ](https://www.st.com/en/embedded-software/fp-ai-monitor1.html)
- [FP-AI-MONITOR2 ](https://www.st.com/en/embedded-software/fp-ai-monitor2.html)
- [FP-AI-PDMWBSOC ](https://www.st.com/en/embedded-software/fp-ai-pdmwbsoc.html)
- [FP-AI-PREDMNT2 ](https://www.st.com/en/embedded-software/fp-ai-predmnt2.html)
- [FP-SNS-DATALOG2](https://github.com/STMicroelectronics/fp-sns-datalog2)

The full description of the API exposed and the design patterns implemented by eLoom and its application modules is described into the CHM available in the Documentation folder, into this [wiki](https://wiki.st.com/stm32mcu/wiki/AI:FP-AI-MONITOR1_an_introduction_to_the_technology_behind) page and [here](http://www.stf12.org/odev/api-doc/html/)


## Known Limitations

- None

## Development Toolchains and Compilers

-   STM32CubeIDE v1.12.0

## Supported Devices and Boards

- [B-U585I-IOT02A](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html)
- [NUCLEO-G431RB](https://www.st.com/en/evaluation-tools/nucleo-g431rb.html) + [X-NUCLEO-IKS02A1](https://www.st.com/en/ecosystems/x-nucleo-iks02a1.html)
- [STEVAL-STWINKT1B](https://www.st.com/stwin)
- [STEVAL-PROTEUS1](https://www.st.com/en/evaluation-tools/steval-proteus1.html)

## Backward Compatibility

- None

## Dependencies

- None
