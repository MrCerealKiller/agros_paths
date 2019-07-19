# Known Issues

There are some known issues with the algorithm that solutions are being found for.

* If the AB line has an azimuth of 0&deg; (i.e. the points true north) it can't always choose the right direction to propagate the offsets
* Right now it assumes that the inner headlands border and the AB line do not intersect. This may cause some issues, but you can always translate it away for the time being