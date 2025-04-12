Linux NatNet (v2.5-3.0) Packet Client (Python and C++)
======================================================

Modified with data extraction for solid-body coordinates
and actual name and IP addresses used in ASTA

The Python version is not (pt.) included.

The code is compile-able on a raspberry pi - or any other (Linux) platform.
- as described below.

May 2021 / Christian Andersen (jca@elektro.dtu.dk)

Receive data from Optitrack Motive (v1.5-2.2)
by directly reading NatNet (2.5-3.1) UDP stream;
works under Ubuntu 14.04 and 16.04 with Python 3 or C++11.
This is a direct translation of the official Packet Client example from
[NatNet SDK](http://optitrack.com/downloads/developer-tools.html#natnet-sdk),
that originally works only on Windows.


Python 3
--------

Install requirements

```bash
pip3 install -r requirements.txt
```

Either run `python3 NatNetClient.py` or write your own script

```python
from NatNetClient import NatNetClient
streamingClient = NatNetClient(ver=(3, 0, 0, 0), quiet=False)
streamingClient.run()
```

Callbacks can be added to process rigid bodies, skeletons, etc.
See `NatNetClient.py` for details.


C++
---

```bash
mkdir build
cd build
ccmake ..
```

Press `c` to configure, again `c` to confirm, and finally `g` to generate.

```bash
make
./PacketClient
```

Note that the NatNet protocol version is hard-coded in `PacketClient.cpp`.
