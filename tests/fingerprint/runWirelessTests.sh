#!/bin/sh
./fingerprints.py *.csv -m wireless -m Wireless -m adhoc -m mobileipv6 -m dymo -m manetrouting -m examples/aodv -m neighborcache -m objectcache -m geometry $*
