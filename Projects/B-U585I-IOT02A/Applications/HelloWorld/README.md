# __B-U585I-IOT02A HelloWorld__

**HelloWorld** is the basic example that can be used to start studying and exploring the **eLooM** main features.
It can be also used as a ready-to-use template to build up more complex applications.

The demo setups the eLooM framework and each second it plots on a terminal a HelloWorld message.
The debug message is streamed via UART through ST-LINK.

This README will guide you through an eLooM-based application as a step-by-step quick start guide.

In the **main.c** file, the application instanciates the eLooM services by just calling the **SysInit** function 
and then starts the FreeRTOS scheduler in the main loop just before the while(1){}.
From now on, the application control is delegated to the eLooM services and to the tasks allocated into the 
**App.c** file with the support from the OS scheduler.

SysInit initializes the minimum set of hardware resources and creates the **INIT** task that is the first 
and only task running during the system initialization process.
SysInit instanciates the HAL drivers and invokes the following functions:

- **SystemClock_Config**
- **SysPowerConfig**

Those functions are implemented into the **sysinit_mx.c** file.

- **SysGetPowerModeHelper**
- **SysLoadApplicationContext**

Each eLooM-based application must provide at least the SysGetPowerModeHelper and the 
SysLoadApplicationContext functions in the **App.c** file.

In HelloWorld, SysLoadApplicationContext simply implements one task (the **HelloWorldTask**).
Once the task object is allocated, **ACAddTask()** is invoked to notify and add the new application task to the 
eLooM context, so that eLooM knows there is a new task called HelloWorldTask to be controlled.

eLooM applications are based on the concept of **Power Mode State Machine**, a system-level state machine that 
can be setup at application level.
The idea is to model in a state machine the specific requirement of the application and so to build a Power Mode 
State for each a particular configuration of the system (the MCU state, which features are enabled, etc.) and events
that triggers the transaction.
SysGetPowerModeHelper instanciates this state machine implemented into the **AppPowerModeHelper.c** file.

Once all the above methods have been invoked in INIT task, the following concrete functions are invoked for any of the 
available new task added to eLooM context:

- **HardwareInit()**
- **OnCreateTask()**

HelloWorldTask is a **AManagedTask**, a base object class provided by eLooM, that implements of methods used by 
the framework to coordinate the task activities at runtime, like system initialization, power management 
and error management.
Not all the methods are implemented by the AManagedTask (A stands for Abstract). Some of them are pure virtual 
functions, so that by just extending this base classes provided by the framework and implementing the pure 
virtual functions, the application tasks are well integrated, and the application code is split from the base 
code provided by ST.

The above-mentioned HardwareInit and OnCreateTask are 2 of the available virtual function that has been implemented
in the HelloWorldTask as **HelloWorldTask_vtblHardwareInit** and **HelloWorldTask_vtblOnCreateTask**.
That's a standard nomenclature in eLooM framework.

HelloWorldTask implements also a very simple State Machine with the only **HelloWorldTaskExecuteStepRun** step
implemented.
Into this state, the task simply implement a task delay of 1 second and plots a HelloWorld message on a terminal.

To customize the example, change the clock tree, update peripherals and irq configurations, please have a
look at the **mx** folder, where there is the **.ioc** used for the available configuration and a README descibing
the full procedure to be followed.


## __Hardware and Software environment__

- This example runs on STM32U5 Devices.

- This example has been tested with STMicroelectronics [B-U585I-IOT02A](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html)
  evaluation boards and can be easily tailored to any other supported
  device and development board. 


## __How to use it?__

In order to make the program work, you must do the following:

- Open the project in STM32CubeIDE

- Rebuild all files and load your image into target memory

- Run the example

