.. index:: utils

.. py:module:: senxor.utils

Utilities
====================

The ``senxor.utils`` module provides functions for handling the
thermal data and the header that comes along with each data frame, as
well as routines for its visualisation with OpneCV.

Transforming data from temperature to intensity and back
--------------------------------------------------------

``remap`` is critical for going from ``float16`` temperature to ``uint8`` 
intensity (on which many OpenCV filters operate) and back to
``float16`` temperature, without loosing accuracy.T
Throughout this process, it is mandatory to be aware of the range that
you're mapping from one domain to the other domain and then back.

.. autofunction:: remap

Displaying the image
--------------------

``cv_render`` helps to produce an coloured image out of the
temperature data, and can render it on the display, as well as return
the image ready to be saved (as an image or video stream).

.. autofunction:: cv_render

Recording data, images and video
--------------------------------

First, get a time-stamped filename, with the senxor ID (if known)
e.g.:

.. code:: python

   filename = get_default_outfile(src_id=mi48.get_camera_id(), ext='.mp4')

.. autofunction:: get_default_outfile


Second, make use of the returned object from ``cv_render``.

Distance Correction
-------------------

.. autofunction:: distance_correction

Data frame transformation and image flipping/orientation
--------------------------------------------------------

.. autofunction:: data_to_frame

