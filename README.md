What is Skeltrack
==================

Skeltrack is a Free and Open Source Software library for tracking
the human skeleton joints from depth images.

It is implemented with GLib and uses plain mathematics to detect
the human skeleton and although it does not use any database, it
was inspired by Andreas Baak's paper:
A Data-Driven Approach for Real-Time Full Body Pose Reconstruction
from a Depth Camera

What is Skeltrack-C++
======================

In this repository you will find a C++ version of Skeltrack.
It is garanteed that the output of the C++ implementation is the same
as the GLibC version.
It is not garanteed that the code is optimized.

How to use it
==============

Although it is device agnostic, a common way of using Skeltrack is
to use it together with a Kinect device, for which we recommend the
freenect library.

Checking out the Test class is a good start in order to learn how to 
use Skeltrack.


Documentation
==============

You can find the documentation for Skeltrack here:
http://people.igalia.com/jrocha/skeltrack/doc/latest/
