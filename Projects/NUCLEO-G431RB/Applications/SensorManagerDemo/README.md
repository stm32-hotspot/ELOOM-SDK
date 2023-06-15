# __NUCLEO-G431RB + X-NUCLEO-IKS02A1 SensorManagerDemo__

If you are not already familiar with eLooM-based firmwares, please consider to have a look at **HelloWorld**
example and README before running this demo.

**SensorStreamerCLI** example is built on top of the HelloWorld demo, where more task have been added and
2 more eLooM components are used w.r.t. HelloWorld: **SensorManager** and **EMData**.
The goal of the demo is to show how eLooM framework handles sensors data and configuration.
Debug messages are still available via UART through ST-LINK.

Sensor Manager is an eLooM-based application-level module that interfaces different kind sensors and offers 
their data to other application modules. It is implemented as an acquisition engine that:

- Orchestrates multiple managed tasks accesses to sensor bus data as follows:
	- One sensor for each managed task
	- Read/write requests via a queue to handle concurrency on common buses  
- Defines interfaces to avoid implementation dependencies
- Dispatches events to notify when data is ready

For the sake of this demo, **ISM330DHCX** through **I2C** are used.
Working with a sensor in eLooM framework is a two steps process:

- Add the sensor to the Application Context and initialize the sensor subsystem.
- Interact with it using the proper API.

As sensor task and bus task are **AManagedTask**, they are added to the eLooM context through **SensorTaskAlloc** + 
**ACAddTask** functions, using the very same pattern already used for **HelloWorldTask**.
For the proper API sintax for each sensor task, please refer to CHM or directly to the code.

Physical sensor is connected to the STM32 to a bus. SensorManager replicate the same design also in software,
where each sensor must be linked to the specific bus thanks to the **BusTaskConnectDevice** method.
This is normally done after all the objects have been initialized, in the **SysOnStartApplication** entry point.
SysOnStartApplication is called by INIT task when all the hardware initialization have been concluded and all
the object tasks have been created.

You are free to add and modify the sensors and the peripherals used by just modifying the **App.c** file 
(i.e.: configure and invoke I2CBusTaskAlloc, LPS22HHTaskAlloc, ...).

EMData offers a data format and API to manipulate any kind of data.
It includes also a set of events and services to handle circular buffers and to easily build up a event 
source-listener paradigm.
EMData are used by SensorManager and by sensors tasks.

To customize the example, change the clock tree, update peripherals and irq configurations, please have a
look at the **mx** folder, where there is the **.ioc** used for the available configuration and a README descibing
the full procedure to be followed.


## __Hardware and Software environment__

- This example runs on STM32G4 Devices.

- This example has been tested with STMicroelectronics [NUCLEO-G431RB](https://www.st.com/en/evaluation-tools/nucleo-g431rb.html)
  evaluation boards and can be easily tailored to any other supported
  device and development board. 


## __How to use it?__

In order to make the program work, you must do the following:

- Open the project in STM32CubeIDE

- Rebuild all files and load your image into target memory

- Run the example

