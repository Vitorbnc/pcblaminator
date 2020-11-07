# pcblaminator
A Thermal Laminator for laminating PCBs and UV Photoresist made with an old  laserjet printer fuser 

Here you will find details on how to build a PCB laminator to use with uv-sensitive (dry-film, etc) methods. 

Inside the **src** folder there's code that runs in the STM32 MCU. The file **laminadora_ident_stm32** is the PID version and **laminadora_stm32** is the ON/OFF 
(aka bang-bang) control version.A system model was developed with matlab, although in the end I chose to keep the bang-bang version. 

The document **Relatório Laminadora** contains a detailed write-up of the procedures.The system model, PID implementation and write-up were carried on by us
as part of an assignment for our University Course.
