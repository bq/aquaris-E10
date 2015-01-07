WHAT IS THIS?
=============

Linux Kernel source code for the device bq Aquaris E10

BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

	$ git clone git@github.com:bq/aquaris-E6.git

After it, choose the version you would like to build:

	$ cd aquaris-E10
	$ git checkout aquaris-E10


Finally, build the kernel:

	$ ./makeMtk -t kaito n k

