ros4mat allows bidirectional local or remote comunication between MATLAB and 
ROS.

To test the provided nodes, clients are provided for each one that will provide
node data and debugging information. They are builded alongside the drivers with
the rosmake command.

TODO
----

Analyser exchangestructs.h
Mettre à jour pour le dernier ROS
Clean ros4mat.c
Clean logicoAgent.cpp
Pointeur de variable globale sur une variable locale => fix this
Flush buffer at the beginning of adcDriver.cpp (fixes rare swapped channels problem?)