.. index:: install

Installation
============

Download the repo or unpack the tarball.

Execute the following from the directory of the pysenxor package:

* If in a virtual environtment (RECOMMENDED)

.. code:: bash

   pip3 install -e .


* If NOT in a virtual environtment (NOT recommended)

.. code:: bash

   pip3 install --user -e .


If you want to see images or video of the thermal data:

* install OpenCV (for video)

* pip3 install matplotlib (sufficient for single image acquisition and
  display)

.. note::
  Note that ``matplotlib`` and ``OpenCV`` are not dependencies for pysenxor,
  but needed for the examples.

