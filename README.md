WHAT IS THIS?
=============

Linux Kernel source code for the device bq Aquaris E10

BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

	$ git clone git@github.com:bq/aquaris-E6.git

After it, choose the version you would like to build:

*Aquaris E10 wifi*

        $ cd aquaris-E10
        $ git checkout aquaris-E10-wifi

*Aquaris E10 3g*

        $ cd aquaris-E10/
        $ git checkout aquaris-E10-3g


Finally, build the kernel according the next table of product names:

| device                                                                                | product                                                               |
| --------------------------|-------------------------|
| bq aquaris E10 wifi                              | kaito_wifi                                      |
| bq aquaris E10 3g      | kaito                     |

        $ ./makeMtk -t {product} n k
