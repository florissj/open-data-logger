# open-data-logger

The project consists of an open-hardware data logger focused on vehicle
dynamics and human control research, especially bicycles and single-track
vehicles.

This is in an early stage of development, so here you will find the base and
[contribution](#Contribution) guidelines.


## Description

Most of the time, when we decide to run an experiment, we need to figure out the
data collection method, which usually ends in a 'new system' from scratch. This 
is time-consuming, and constrains the compatibility among different and legacy
projects. For this reason, the main idea of this project is to create a robust 
data-logger core, which is expandable through independent modules in a 
'plug-and-play' setting.

## Contribution

Since this is an open-hardware initiative, contributions are welcome. 

First, create a fork of this repository, and clone it with

```
git@github.com:$USER/open-data-logger.git
```


### Add a new module

If you develop a new module, please include the corresponding documentation and
design files, such as CAD, wiring diagrams, etc.

In general, every module should contain:
- CAD file for its housing.
- Wiring diagram.
- Code with compatibility check.
- Documentation of use, measurement format, requirements, etc.

### Code

If you add or modify a piece of code, ensure its functionality and compatibility
with existing software.

### Issues

Check currently active issues, maybe your request is highly related to something
already on discussion.



## Authors

[Benjamín González](mailto:b.gonzaleztoledo@tudelft.nl), Floris Mostert, Jason
Moore, Christoph Konrad.

## Acknowledgments

This project is possible thanks to the Open Research Hardware Stimulation Fund 
Program from TU Delft.

## Structure

The project is organized in the following file system:

- **data:** *All raw, processed, and external data used in your project as well 
as result files genertated by your analyses.*

- **designs:** *Design files of (drawings, 3D print files, schematics) of the 
devices and hardware components you create in your project.*

- **documentation:** *Put the documentation of your project here. This includes 
data management and HREC procedures.*

- **notes:** *Space for any additional notes you create.*

- **software:** *Code for processing, analysis and simulation.*

- **publication**: *The publication resulting from this project.*
