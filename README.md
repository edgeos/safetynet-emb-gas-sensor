Getting Started
===============

## SDK SETUP ##

- The Nordic SDK needs to be downloaded and installed. Installers can be found here: https://developer.nordicsemi.com/nRF5_SDK/
  The current version used by this project is v15.2.0.

- After installing, make a symbolic link to the Nordic SDK in the repository root directory. On Linux, this can be accomplished with a command similar to the following:

	ln -s /LINK/TO/SDK_ROOT_FOLDER/ /LINK/TO/GIT_REPO_FOLDER/nordic_sdk

  On Windows, follow the instructions here: https://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/

- NOTE: You will not be able to build & compile unless this link is setup correctly.

## IDE ##

- All development is done using Segger Embedded Studio (SES). This can be downloaded from the following location: https://www.segger.com/downloads/embedded-studio 

- Development can also be done in Eclipse, however SES is a much easier user experience and is highly recommended. SES requires a license for use, however they currently have a partnership with Nordic that provides a free commericial license to anyone developing on the Nordic platform. 

- Note that Makefiles and Eclipse project files are not provided in this repo - they must be configured manually if desired. 

## OTHER ##

- Download and install the Segger JLink Software if it's not already installed from here: https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack

- Make sure these tools are in your system's PATH, otherwise you will not be able to download and debug from the IDE.


Compiling
=========

- Open the SES .emProject file in the desired target directory. Double-clicking should open SES automatically. Example directory structure:

  $ROOT/target/<board>/<soft_device>/ses/<project_name>.emProject

- To compile, go to the Build menu and select Build. If the project successfully compiles, then your environment for development should be set up correctly.


Builds
======

- After a successful compilation, the output files (hex, elf, etc)  will be placed in the folder below:

  $ROOT/target/<board>/<soft_device>/ses/Outputs
  
Application
===========

The flow chart below gives a detailed overview of the embedded software operation:

![alt text](https://github.com/edgeos/safetynet-emb-gas-sensor/blob/master/misc/Gas%20Sensor%20Flow%20Chart.png)
