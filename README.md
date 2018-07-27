A lightweight and modular implementation of the RRT\* algorithm.

# TODO

- Implement the distance transform.

# Dependencies

This library requires the [libpng](http://www.libpng.org/pub/png/libpng.html) library.
On Ubuntu it can be installed with:

    sudo apt-get install libpng-dev

Alternatively run:

    git clone git://git.code.sf.net/p/libpng/code libpng
    cd libpng
    ./autogen.sh
    ./configure --prefix=[INSTALL_PATH]
    make check
    make install

# Building

    cd motion_planning
    mkdir build
    cmake ..
    make
